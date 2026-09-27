"""Calibration main-œil (hand-eye) « eye-to-hand » : caméra FIXE, marqueur ArUco collé sur l'outil.

But : trouver T_base_cam, la pose de la caméra dans le repère de la base.

Principe : le marqueur est rigide par rapport à l'outil. Pour N poses du robot on connaît
  * T_base_tool_i  (cinématique directe, /end_effector_pose)
  * T_cam_marker_i (vision, /vision/marker_in_camera)
et  T_base_cam · T_cam_marker_i = T_base_tool_i · T_tool_marker  (T_tool_marker inconnue, constante).
C'est un problème  AX = XB  résolu par cv2.calibrateHandEye. En configuration eye-to-hand, on lui
fournit la pose INVERSE de l'outil (base -> outil) : le résultat est alors la pose caméra -> base.

Procédure (le robot doit être PRÊT, vision_node lancé avec le détecteur 'aruco' et target_id = celui du marqueur) :
  ros2 run robot_vision hand_eye_calibration --out hand_eye.yaml
  -> amener le robot (IHM, sliders) à une pose où le marqueur est bien visible, IMMOBILE, puis Entrée.
  -> répéter pour 12-20 poses TRÈS VARIÉES (positions et surtout ORIENTATIONS différentes du poignet).
  -> taper « c » pour calculer et enregistrer, « d » pour effacer la dernière pose, « q » pour quitter.
"""
import argparse
import sys
import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from robot_kinematics.ros_utils import QOS_SENSOR, pose_to_matrix_msg
from robot_kinematics.transforms import make_transform, matrix_to_quat

from .vision_core import average_rotation, hand_eye_eye_to_hand


class Collector(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration')
        self.T_bt: Optional[np.ndarray] = None
        self.marker_msgs: List[np.ndarray] = []
        self.create_subscription(PoseStamped, '/end_effector_pose', self.on_tool, QOS_SENSOR)
        self.create_subscription(PoseStamped, '/vision/marker_in_camera', self.on_marker, QOS_SENSOR)

    def on_tool(self, msg):
        self.T_bt = pose_to_matrix_msg(msg.pose)

    def on_marker(self, msg):
        self.marker_msgs.append(pose_to_matrix_msg(msg.pose))
        self.marker_msgs = self.marker_msgs[-60:]

    def grab_sample(self, n: int = 15, timeout: float = 5.0):
        """Moyenne n mesures fraîches du marqueur (robot immobile) et lit la pose de l'outil."""
        self.marker_msgs.clear()
        t_end = time.time() + timeout
        while len(self.marker_msgs) < n and time.time() < t_end:
            time.sleep(0.02)
        if len(self.marker_msgs) < n or self.T_bt is None:
            return None
        ms = list(self.marker_msgs)
        t = np.median([m[:3, 3] for m in ms], axis=0)
        spread_mm = float(np.max(np.std([m[:3, 3] for m in ms], axis=0)) * 1e3)
        return self.T_bt.copy(), make_transform(average_rotation([m[:3, :3] for m in ms]), t), spread_mm


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='hand_eye.yaml')
    ap.add_argument('--min-poses', type=int, default=10)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Collector()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    samples = []
    print(__doc__.split('Procédure')[0])
    print('Entrée = enregistrer la pose | c = calculer | d = effacer la dernière | q = quitter')
    try:
        while True:
            cmd = input(f'[{len(samples)} poses] > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'd' and samples:
                samples.pop()
                continue
            if cmd == 'c':
                if len(samples) < args.min_poses:
                    print(f'Il faut au moins {args.min_poses} poses (12-20 recommandé).')
                    continue
                T, t_mm, r_deg = hand_eye_eye_to_hand(samples)
                print('T_base_cam =\n', np.array2string(T, precision=4, suppress_small=True))
                print(f'Cohérence : dispersion de T_outil_marqueur = {t_mm:.1f} mm, {r_deg:.1f}°  '
                      f'({"bon" if t_mm < 5 and r_deg < 3 else "moyen" if t_mm < 10 else "MAUVAIS : refaire avec des poses plus variées"})')
                q = matrix_to_quat(T[:3, :3])
                with open(args.out, 'w', encoding='utf-8') as f:
                    yaml.safe_dump({
                        'parent_frame': 'base_link', 'camera_frame': 'camera_optical_frame',
                        'translation': [float(x) for x in T[:3, 3]],
                        'quaternion_xyzw': [float(x) for x in q],
                        'calibrated': True, 'residual_translation_mm': t_mm, 'residual_rotation_deg': r_deg,
                    }, f, sort_keys=False)
                print(f'Écrit : {args.out}  (à copier dans robot_vision/config/hand_eye.yaml)')
                continue
            s = node.grab_sample()
            if s is None:
                print('Échec : marqueur non détecté ou pas de /end_effector_pose (vision_node lancé ? marqueur visible ?)')
                continue
            Tbt, Tcm, spread = s
            if spread > 2.0:
                print(f'Marqueur instable ({spread:.1f} mm) : le robot bouge encore ? Pose ignorée.')
                continue
            samples.append((Tbt, Tcm))
            print(f'Pose {len(samples)} enregistrée (bruit vision {spread:.2f} mm)')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
