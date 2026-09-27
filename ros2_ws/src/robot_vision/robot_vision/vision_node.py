"""vision_node : détecte un objet et publie sa pose 3D dans le repère du robot.

Sources d'image (paramètre `source`) :
  'topic'      : s'abonne à `image_topic` (défaut /camera/image_raw, driver ROS externe)
  'opencv'     : webcam USB ou pipeline GStreamer
  'picamera2'  : Pi Camera directement (Camera Module 3, mise au point fixe)

Détecteurs (paramètre `detector`) :
  'aruco' : marqueur ArUco -> pose 6D par solvePnP (taille du marqueur connue)
  'hsv'   : blob coloré -> position par intersection du rayon optique avec le plan de la table

Publications :
  /target_pose               PoseStamped (base_link, reliable)  pose de saisie (outil vers le bas)
  /target_visible            Bool
  /camera/image_annotated    CompressedImage (JPEG, best effort) image + détection + coordonnées 3D
  /vision/marker_in_camera   PoseStamped (repère optique)  pose brute du marqueur (calibration main-œil)
Diffuse aussi le TF statique  base_link -> camera_optical_frame  (fichier hand_eye.yaml).
"""
import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

from robot_kinematics.ros_utils import (QOS_RELIABLE, QOS_SENSOR, matrix_to_pose_stamped,
                                        matrix_to_transform_stamped)
from . import vision_core as vc
from .frame_source import open_source


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        share = get_package_share_directory('robot_vision')
        self.declare_parameter('source', 'topic')                 # topic | opencv | picamera2
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('device', '0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)                       # cadence de traitement
        self.declare_parameter('lens_position', 2.0)              # dioptries (Pi Cam v3, mise au point fixe)
        self.declare_parameter('detector', 'aruco')               # aruco | hsv
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.04)             # m : côté du carré NOIR du marqueur
        self.declare_parameter('target_id', -1)                   # -1 : n'importe quel marqueur
        self.declare_parameter('hsv_lower', [170, 120, 70])       # rouge par défaut (H enjambe 0)
        self.declare_parameter('hsv_upper', [10, 255, 255])
        self.declare_parameter('min_area', 300.0)
        self.declare_parameter('plane_z', 0.02)                   # m : hauteur du centre de l'objet (HSV)
        self.declare_parameter('grasp_z_offset', 0.0)             # m : décalage vertical de la saisie (ArUco)
        self.declare_parameter('smoothing', 5)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('camera_file', os.path.join(share, 'config', 'camera.yaml'))
        self.declare_parameter('hand_eye_file', os.path.join(share, 'config', 'hand_eye.yaml'))

        self.K, self.dist, size, cam_ok = vc.load_camera_yaml(self.get_parameter('camera_file').value)
        self.T_base_cam, self.base_frame, self.cam_frame, he_ok = vc.load_hand_eye_yaml(
            self.get_parameter('hand_eye_file').value)
        if not cam_ok:
            self.get_logger().warn('Caméra NON calibrée (camera.yaml par défaut) : la précision sera faible')
        if not he_ok:
            self.get_logger().warn('Calibration main-œil ABSENTE (hand_eye.yaml par défaut) : '
                                   'les positions 3D ne sont pas fiables (voir docs/CALIBRATION.md)')

        self.smoother = vc.PoseSmoother(int(self.get_parameter('smoothing').value))
        self._miss = 0
        self._visible = False
        self._last_proc = 0.0

        self.pub_pose = self.create_publisher(PoseStamped, '/target_pose', QOS_RELIABLE)
        self.pub_vis = self.create_publisher(Bool, '/target_visible', QOS_RELIABLE)
        self.pub_img = self.create_publisher(CompressedImage, '/camera/image_annotated', QOS_SENSOR)
        self.pub_marker = self.create_publisher(PoseStamped, '/vision/marker_in_camera', QOS_SENSOR)

        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(matrix_to_transform_stamped(
            self.T_base_cam, self.base_frame, self.cam_frame, self.get_clock().now().to_msg()))

        source = self.get_parameter('source').value
        self.grabber = None
        if source == 'topic':
            self.create_subscription(Image, self.get_parameter('image_topic').value,
                                     self.on_image, QOS_SENSOR)
        else:
            self.grabber = open_source(source, self.get_parameter('device').value,
                                       self.get_parameter('width').value, self.get_parameter('height').value,
                                       self.get_parameter('fps').value, self.get_parameter('lens_position').value)
            self.create_timer(1.0 / self.get_parameter('fps').value, self.on_timer)
        self.get_logger().info(f"vision_node prêt (source={source}, détecteur={self.get_parameter('detector').value})")

    # ------------------------------------------------------------------ acquisition --
    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_proc < 1.0 / self.get_parameter('fps').value:
            return                                   # limite la charge CPU du Raspberry Pi
        self._last_proc = now
        frame = vc.image_to_bgr(bytes(msg.data), msg.height, msg.width, msg.step, msg.encoding)
        if frame is None:
            self.get_logger().warn(f'Encodage non géré : {msg.encoding}', throttle_duration_sec=5.0)
            return
        self.process(frame)

    def on_timer(self):
        frame = self.grabber.read()
        if frame is not None:
            self.process(frame)

    # ------------------------------------------------------------------ traitement ----
    def process(self, frame: np.ndarray):
        stamp = self.get_clock().now().to_msg()
        detector = self.get_parameter('detector').value
        T_obj: Optional[np.ndarray] = None
        label = 'aucune cible'

        if detector == 'aruco':
            T_obj, label = self._detect_aruco(frame, stamp)
        else:
            T_obj, label = self._detect_hsv(frame)

        if T_obj is not None:
            self._miss = 0
            T_s = self.smoother.update(T_obj)
            self.pub_pose.publish(matrix_to_pose_stamped(T_s, self.base_frame, stamp))
            x, y, z = T_s[:3, 3]
            text = f'{label} | X={x * 100:.1f} Y={y * 100:.1f} Z={z * 100:.1f} cm'
            colour = (0, 220, 0)
            self._set_visible(True)
        else:
            self._miss += 1
            if self._miss > 5:
                self.smoother.reset()
                self._set_visible(False)
            text, colour = label, (0, 0, 255)

        cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(frame, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1)
        self._publish_image(frame, stamp)

    def _detect_aruco(self, frame, stamp):
        dets = vc.detect_aruco(frame, self.get_parameter('aruco_dict').value)
        want = int(self.get_parameter('target_id').value)
        dets = [d for d in dets if want < 0 or d.marker_id == want]
        if not dets:
            return None, 'aucun marqueur ArUco'
        d = dets[0]
        cv2.polylines(frame, [d.corners.astype(np.int32)], True, (0, 255, 0), 2)
        L = float(self.get_parameter('marker_length').value)
        pose = vc.marker_pose_camera(d.corners, L, self.K, self.dist)
        if pose is None:
            return None, f'ArUco {d.marker_id} : pose impossible'
        rvec, tvec = pose
        cv2.drawFrameAxes(frame, self.K, self.dist, rvec, tvec, L * 0.75)
        # pose brute dans la caméra (pour la calibration main-œil)
        R, _ = cv2.Rodrigues(rvec)
        T_cm = np.eye(4)
        T_cm[:3, :3], T_cm[:3, 3] = R, tvec.reshape(3)
        self.pub_marker.publish(matrix_to_pose_stamped(T_cm, self.cam_frame, stamp))
        # pose dans la base -> saisie par le dessus, orientation dérivée du yaw du marqueur
        T_bm = vc.marker_pose_base(rvec, tvec, self.T_base_cam)
        yaw = float(np.arctan2(T_bm[1, 0], T_bm[0, 0]))
        T = np.eye(4)
        T[:3, :3] = vc.top_down_grasp_rotation(yaw)
        T[:3, 3] = T_bm[:3, 3] + np.array([0.0, 0.0, self.get_parameter('grasp_z_offset').value])
        return T, f'ArUco {d.marker_id}'

    def _detect_hsv(self, frame):
        det = vc.detect_hsv(frame, self.get_parameter('hsv_lower').value,
                            self.get_parameter('hsv_upper').value, self.get_parameter('min_area').value)
        if det is None:
            return None, 'aucun objet de la couleur cible'
        cv2.drawContours(frame, [det.contour], -1, (0, 255, 0), 2)
        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        T = vc.color_pose_base(det, self.K, self.dist, self.T_base_cam, self.get_parameter('plane_z').value)
        if T is None:
            return None, 'objet détecté mais projection impossible'
        return T, 'objet HSV'

    def _set_visible(self, v: bool):
        self._visible = v
        self.pub_vis.publish(Bool(data=v))         # à chaque image : robuste aux connexions tardives

    def _publish_image(self, frame, stamp):
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(self.get_parameter('jpeg_quality').value)])
        if ok:
            msg = CompressedImage()
            msg.header.stamp, msg.header.frame_id = stamp, self.cam_frame
            msg.format, msg.data = 'jpeg', buf.tobytes()
            self.pub_img.publish(msg)

    def destroy_node(self):
        if self.grabber is not None:
            self.grabber.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
