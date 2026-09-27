"""ik_node : cinématique inverse.

Abonnements : /target_pose   (geometry_msgs/PoseStamped, reliable) : pose cible de l'OUTIL
              /joint_states  (sensor_msgs/JointState, best effort) : sert de référence (config la plus proche)
Publications : /joint_goal   (trajectory_msgs/JointTrajectory, reliable) : BUT articulaire (1 point)
               /ik_result    (std_msgs/String JSON, reliable) : résultat lisible par l'IHM
Service      : /solve_ik     (robot_interfaces/SolveIK) : prévisualisation sans mouvement

Ce nœud ne publie PAS sur /joint_commands : la consigne lissée (vitesse/accélération limitées)
est produite uniquement par trajectory_node.
"""
import json
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from robot_interfaces.srv import SolveIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config import load_robot_config
from .kinematics import ArmKinematics, IKResult
from .ros_utils import QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays, pose_to_matrix_msg, \
    transform_stamped_to_matrix


class IkNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('reject_near_singularity', False)  # True : refuse les poses singulières
        self.declare_parameter('tol_position', 1e-4)               # m
        self.declare_parameter('tol_orientation', 1e-3)            # rad

        self.cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.arm = ArmKinematics(self.cfg)
        self.q_meas: Optional[np.ndarray] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_goal = self.create_publisher(JointTrajectory, '/joint_goal', QOS_RELIABLE)
        self.pub_result = self.create_publisher(String, '/ik_result', QOS_RELIABLE)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, QOS_SENSOR)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target, QOS_RELIABLE)
        self.create_service(SolveIK, '/solve_ik', self.on_solve_srv)
        self.get_logger().info('ik_node prêt (analytique + repli Levenberg-Marquardt)')

    # ------------------------------------------------------------------ entrées --
    def on_joint_states(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.cfg.joint_names)
        if arrays is not None:
            self.q_meas = arrays[0]

    def _pose_in_base(self, ps: PoseStamped) -> Optional[np.ndarray]:
        """Pose de l'outil (4x4) exprimée dans base_link ; None si le TF est indisponible."""
        T = pose_to_matrix_msg(ps.pose)
        frame = ps.header.frame_id
        if frame in ('', self.cfg.base_frame):
            return T
        try:
            ts = self.tf_buffer.lookup_transform(self.cfg.base_frame, frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.2))
        except Exception as exc:  # noqa: BLE001 - tf2 lève plusieurs types d'exceptions
            self.get_logger().warn(f'TF {frame} -> {self.cfg.base_frame} indisponible : {exc}')
            return None
        return transform_stamped_to_matrix(ts) @ T

    # ------------------------------------------------------------------- calcul --
    def _solve(self, T: np.ndarray, seed) -> IKResult:
        if T[2, 3] < self.cfg.z_min:
            return IKResult(False, None, 'none',
                            f'Cible sous le plan de sécurité (z = {T[2, 3]:.3f} m < {self.cfg.z_min:.3f} m)')
        res = self.arm.solve(T, seed,
                             tol_pos=self.get_parameter('tol_position').value,
                             tol_ori=self.get_parameter('tol_orientation').value)
        if res.success and res.near_singularity:
            res.message += ' — ATTENTION proche d\'une singularité : ' + '; '.join(res.singularity_kinds)
            if self.get_parameter('reject_near_singularity').value:
                res.success = False
                res.message = 'Refusée : ' + res.message
        return res

    def _publish_result(self, res: IKResult):
        payload = {
            'success': bool(res.success), 'message': res.message, 'method': res.method,
            'near_singularity': bool(res.near_singularity), 'singularity': res.singularity_kinds,
            'q': [] if res.q is None else [float(x) for x in res.q],
            'pos_err': None if np.isnan(res.pos_err) else res.pos_err,
            'ori_err': None if np.isnan(res.ori_err) else res.ori_err,
        }
        self.pub_result.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def on_target(self, msg: PoseStamped):
        T = self._pose_in_base(msg)
        if T is None:
            self._publish_result(IKResult(False, None, 'none', 'Transformation TF indisponible'))
            return
        seed = self.q_meas if self.q_meas is not None else self.cfg.rest
        res = self._solve(T, seed)
        self._publish_result(res)
        if not res.success:
            self.get_logger().warn('IK : ' + res.message)
            return
        goal = JointTrajectory()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.joint_names = list(self.cfg.joint_names)
        goal.points = [JointTrajectoryPoint(positions=[float(x) for x in res.q])]
        self.pub_goal.publish(goal)
        self.get_logger().info(f'IK ({res.method}) -> but articulaire publié')

    def on_solve_srv(self, request: SolveIK.Request, response: SolveIK.Response):
        T = self._pose_in_base(request.target)
        if T is None:
            response.reachable, response.message = False, 'Transformation TF indisponible'
            return response
        seed = np.array(request.seed) if len(request.seed) == 6 else (
            self.q_meas if self.q_meas is not None else self.cfg.rest)
        res = self._solve(T, seed)
        response.reachable = bool(res.success)
        response.near_singularity = bool(res.near_singularity)
        response.message = res.message
        response.joint_positions = [] if res.q is None else [float(x) for x in res.q]
        response.position_error = 0.0 if np.isnan(res.pos_err) else float(res.pos_err)
        response.orientation_error = 0.0 if np.isnan(res.ori_err) else float(res.ori_err)
        response.method = res.method
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IkNode()
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
