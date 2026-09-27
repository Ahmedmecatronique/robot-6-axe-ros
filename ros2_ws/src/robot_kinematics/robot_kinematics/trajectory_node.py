"""trajectory_node : interpolation entre la position actuelle et le but, à 100 Hz.

Abonnements : /joint_goal    (trajectory_msgs/JointTrajectory, reliable) : but(s) articulaire(s)
              /joint_states  (sensor_msgs/JointState, best effort)        : position mesurée
              /stm32/status  (std_msgs/Int32MultiArray)                    : état du robot
              /emergency_stop (std_msgs/Bool, reliable)                    : annule la trajectoire
Publications : /joint_commands  (sensor_msgs/JointState, reliable, 100 Hz) : consigne lissée
                                 (position + vitesse, vitesse = préaction pour le PID du STM32)
               /trajectory_active (std_msgs/Bool)                          : mouvement en cours

Unique publisher de /joint_commands. Publie EN PERMANENCE (même à l'arrêt) : ce flux sert aussi de
heartbeat au watchdog du STM32.
Tant que le robot n'est pas PRÊT (non référencé, désactivé, arrêt d'urgence...), la consigne
suit passivement la mesure et les buts sont refusés.
"""
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from trajectory_msgs.msg import JointTrajectory

from .config import load_robot_config
from .kinematics import ArmKinematics
from .ros_utils import QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays, make_joint_state
from .status_codes import ST_STATE, STATE_READY
from .trajectory import plan_segment


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('rate', 100.0)                # Hz
        self.declare_parameter('profile', 'quintic')         # 'quintic' | 'trapezoid'
        self.declare_parameter('speed_scale', 1.0)           # 0.05 .. 1.0 (réglable à chaud)
        self.declare_parameter('require_ready', True)        # refuse les buts si l'état != PRÊT
        self.declare_parameter('limit_margin', 0.002)        # rad de tolérance sur les limites

        self.cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.arm = ArmKinematics(self.cfg)
        self.names = self.cfg.joint_names

        self.q_meas: Optional[np.ndarray] = None
        self.q_cmd: Optional[np.ndarray] = None
        self.v_cmd = np.zeros(6)
        self.robot_state = -1
        self.segment = None
        self.seg_t0 = 0.0
        self.queue: List[np.ndarray] = []
        self.queue_min_dur: List[float] = []
        self._active_pub = False
        self._active_t = 0.0

        self.pub_cmd = self.create_publisher(JointState, '/joint_commands', QOS_RELIABLE)
        self.pub_active = self.create_publisher(Bool, '/trajectory_active', QOS_RELIABLE)
        self.create_subscription(JointTrajectory, '/joint_goal', self.on_goal, QOS_RELIABLE)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, QOS_SENSOR)
        self.create_subscription(Int32MultiArray, '/stm32/status', self.on_status, QOS_SENSOR)
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, QOS_RELIABLE)
        self.add_on_set_parameters_callback(self.on_params)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info('trajectory_node prêt')

    # ------------------------------------------------------------------ helpers --
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ready(self) -> bool:
        return (not self.get_parameter('require_ready').value) or self.robot_state == STATE_READY

    def _limits(self):
        k = float(np.clip(self.get_parameter('speed_scale').value, 0.02, 1.0))
        return self.cfg.v_max * k, self.cfg.a_max * k

    def on_params(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            if p.name == 'profile' and p.value not in ('quintic', 'trapezoid'):
                return SetParametersResult(successful=False, reason='profile = quintic | trapezoid')
            if p.name == 'speed_scale' and not (0.0 < float(p.value) <= 1.0):
                return SetParametersResult(successful=False, reason='speed_scale dans ]0, 1]')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ entrées ---
    def on_joint_states(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is not None:
            self.q_meas = arrays[0]

    def on_status(self, msg: Int32MultiArray):
        if len(msg.data) > ST_STATE:
            self.robot_state = int(msg.data[ST_STATE])

    def on_estop(self, msg: Bool):
        if msg.data:
            self._cancel('arrêt d\'urgence')

    def _cancel(self, why: str):
        if self.segment is not None or self.queue:
            self.get_logger().warn(f'Trajectoire annulée : {why}')
        self.segment, self.queue, self.queue_min_dur = None, [], []
        self.v_cmd = np.zeros(6)
        if self.q_meas is not None:
            self.q_cmd = self.q_meas.copy()      # on repart de la position réelle

    def on_goal(self, msg: JointTrajectory):
        """Reçoit un ou plusieurs buts (waypoints) ; arrêt à chaque point."""
        if self.q_meas is None:
            self.get_logger().warn('But refusé : aucune position mesurée reçue (/joint_states)')
            return
        if not self._ready():
            self.get_logger().warn('But refusé : robot non prêt (référencer / activer d\'abord)')
            return
        if len(msg.points) == 0:
            return
        order = list(msg.joint_names) if msg.joint_names else list(self.names)
        try:
            idx = [order.index(n) for n in self.names]
        except ValueError:
            self.get_logger().error(f'But refusé : noms d\'articulations inattendus {order}')
            return

        waypoints, durations, prev_t = [], [], 0.0
        for pt in msg.points:
            if len(pt.positions) != len(order):
                self.get_logger().error('But refusé : nombre de positions incorrect')
                return
            q = np.array([pt.positions[i] for i in idx], dtype=float)
            ok, bad = self.arm.within_limits(q, self.get_parameter('limit_margin').value)
            if not ok:
                self.get_logger().error('But refusé : hors limites articulaires ('
                                        + ', '.join(self.names[i] for i in bad) + ')')
                return
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            durations.append(max(t - prev_t, 0.0))
            prev_t = max(t, prev_t)
            waypoints.append(q)

        # Le nouveau but remplace l'ancien ; on repart de la consigne courante (vitesse conservée)
        self.queue, self.queue_min_dur = waypoints, durations
        self.segment = None
        self._start_next()

    def _start_next(self):
        """Démarre le segment suivant depuis (q_cmd, v_cmd)."""
        while self.queue:
            target = self.queue.pop(0)
            min_dur = self.queue_min_dur.pop(0)
            v_max, a_max = self._limits()
            seg = plan_segment(self.q_cmd, target, v_max, a_max, v0=self.v_cmd,
                               profile=self.get_parameter('profile').value, min_duration=min_dur)
            if seg is None:                     # déjà au but
                continue
            self.segment, self.seg_t0 = seg, self._now()
            return
        self.segment = None

    # ------------------------------------------------------------------ boucle 100 Hz
    def tick(self):
        if self.q_meas is None:
            return
        if not self._ready():
            # Suivi passif : la consigne colle à la mesure, aucun mouvement autorisé
            self.q_cmd = self.q_meas.copy()
            self.v_cmd = np.zeros(6)
            self.segment, self.queue, self.queue_min_dur = None, [], []
        else:
            if self.q_cmd is None:
                self.q_cmd = self.q_meas.copy()
            if self.segment is not None:
                t = self._now() - self.seg_t0
                q, v, _ = self.segment.sample(t)
                self.q_cmd, self.v_cmd = q, v
                if t >= self.segment.duration:
                    self.v_cmd = np.zeros(6)
                    self.segment = None
                    self._start_next()

        stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(make_joint_state(self.names, self.q_cmd, self.v_cmd, stamp=stamp))

        active = self.segment is not None or bool(self.queue)
        now = self._now()
        if active != self._active_pub or now - self._active_t > 0.2:   # sur changement + 5 Hz
            self._active_pub, self._active_t = active, now
            self.pub_active.publish(Bool(data=active))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
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
