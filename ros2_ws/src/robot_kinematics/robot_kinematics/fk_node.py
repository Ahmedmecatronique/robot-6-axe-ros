"""fk_node : cinématique directe temps réel.

Abonnements : /joint_states (sensor_msgs/JointState, best effort)  <- positions RÉELLES (encodeurs)
Publications : /end_effector_pose (geometry_msgs/PoseStamped, best effort, <= 50 Hz)
               /tf : repères DH  dh_1 .. dh_6  et  tool_fk  (parent = base_link)

Les TF des liens (link_i) sont diffusés par robot_state_publisher depuis l'URDF ; les repères
"dh_*" et "tool_fk" sont calculés par NOTRE cinématique. Dans RViz, ils doivent coïncider avec
le robot : c'est un contrôle croisé URDF <-> DH.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from .config import load_robot_config
from .kinematics import ArmKinematics
from .ros_utils import (QOS_SENSOR, joint_state_to_arrays, matrix_to_pose_stamped,
                        matrix_to_transform_stamped)


class FkNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('publish_rate', 50.0)        # Hz max pour /end_effector_pose
        self.declare_parameter('broadcast_dh_frames', True)  # diffuse dh_1..dh_6 + tool_fk

        cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.cfg = cfg
        self.arm = ArmKinematics(cfg)
        self.min_period = 1.0 / max(self.get_parameter('publish_rate').value, 1.0)
        self.broadcast_dh = self.get_parameter('broadcast_dh_frames').value
        self._last_pub = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(PoseStamped, '/end_effector_pose', QOS_SENSOR)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, QOS_SENSOR)
        self.get_logger().info('fk_node prêt (portée %.2f m)' % cfg.reach)

    def on_joint_states(self, msg: JointState):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_pub < self.min_period:
            return                                   # limite le débit de sortie
        arrays = joint_state_to_arrays(msg, self.cfg.joint_names)
        if arrays is None:
            return
        q = arrays[0]
        self._last_pub = now
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        frames = self.arm.forward_frames(q)          # T_0^0 .. T_0^6
        T_tool = frames[6]
        self.pub_pose.publish(matrix_to_pose_stamped(T_tool, self.cfg.base_frame, stamp))

        if self.broadcast_dh:
            tfs = [matrix_to_transform_stamped(frames[i], self.cfg.base_frame, f'dh_{i}', stamp)
                   for i in range(1, 7)]
            tfs.append(matrix_to_transform_stamped(T_tool, self.cfg.base_frame, 'tool_fk', stamp))
            self.tf_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = FkNode()
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
