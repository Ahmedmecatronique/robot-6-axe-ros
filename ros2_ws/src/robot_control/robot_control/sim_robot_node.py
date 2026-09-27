"""sim_robot_node : simulateur du STM32 (mode simulation, sans matériel).

Expose EXACTEMENT la même interface ROS que le firmware, donc tout le reste du système
(trajectory_node, safety_node, IHM...) fonctionne sans le savoir :

  Abonnements : /joint_commands (JointState, reliable)  consigne (position + vitesse de préaction)
                /emergency_stop (Bool, reliable)         arrêt d'urgence + heartbeat
                /stm32/command  (Int32)                  ENABLE / DISABLE / HOME / ZERO_HERE
                /gripper_command (Float32)               (affiché seulement)
  Publications : /joint_states  (JointState, best effort, 100 Hz)   positions "encodeurs" simulées
                 /stm32/status  (Int32MultiArray, 10 Hz)            même format que le firmware

Modèle : chaque axe est un suiveur de position à vitesse et accélération bornées (comme le PID 1 kHz
+ générateur de pas du firmware). La machine d'états et le watchdog reproduisent ceux du STM32.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray

from robot_kinematics.config import load_robot_config
from robot_kinematics.ros_utils import QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays, make_joint_state
from robot_kinematics.status_codes import (CMD_DISABLE, CMD_ENABLE, CMD_HOME, CMD_ZERO_HERE,
                                           FAULT_COMM, FAULT_ESTOP, ST_LEN, STATE_DISABLED,
                                           STATE_ESTOP, STATE_FAULT, STATE_HOMING, STATE_IDLE,
                                           STATE_READY)

ALL_HOMED = 0b111111


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('start_ready', True)     # False : démarre désactivé, non référencé
        self.declare_parameter('kp', 8.0)               # gain du suiveur (1/s)
        self.declare_parameter('watchdog_ms', 300.0)
        self.declare_parameter('homing_time_per_axis', 0.6)   # s (simulation accélérée)

        self.cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.names = self.cfg.joint_names
        hw = self.cfg.raw['hardware']
        # Position de chaque fin de course (côté min ou max selon home_dir)
        self.home_pos = np.where(np.array(hw['home_dir']) < 0, self.cfg.q_min, self.cfg.q_max)
        self.home_order = list(hw['home_order'])

        start_ready = self.get_parameter('start_ready').value
        self.q = self.cfg.rest.copy() if start_ready else np.zeros(6)
        self.v = np.zeros(6)
        self.state = STATE_READY if start_ready else STATE_DISABLED
        self.homed = ALL_HOMED if start_ready else 0
        self.fault = 0
        self.q_cmd = None
        self.v_cmd = np.zeros(6)
        self.t_cmd = None
        self.t_hb = None
        self.max_ferr = 0.0
        self.t_start = self._now()
        self._home_t0 = 0.0
        self._home_from = self.q.copy()

        self.pub_js = self.create_publisher(JointState, '/joint_states', QOS_SENSOR)
        self.pub_status = self.create_publisher(Int32MultiArray, '/stm32/status', QOS_SENSOR)
        self.create_subscription(JointState, '/joint_commands', self.on_cmd, QOS_RELIABLE)
        self.create_subscription(Bool, '/emergency_stop', self.on_estop, QOS_RELIABLE)
        self.create_subscription(Int32, '/stm32/command', self.on_command, QOS_RELIABLE)
        self.create_subscription(Float32, '/gripper_command', self.on_gripper, QOS_RELIABLE)

        self._dt = 1.0 / self.get_parameter('rate').value
        self._last = self._now()
        self.create_timer(self._dt, self.tick)
        self.create_timer(0.1, self.publish_status)
        self.get_logger().info('sim_robot_node : robot simulé (état initial : '
                               + ('PRÊT' if start_ready else 'DÉSACTIVÉ') + ')')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ entrées -----
    def on_cmd(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is not None:
            self.q_cmd, self.v_cmd, _ = arrays
            self.t_cmd = self._now()

    def on_estop(self, msg: Bool):
        self.t_hb = self._now()
        if msg.data and self.state not in (STATE_ESTOP, STATE_DISABLED):
            self.state, self.fault = STATE_ESTOP, FAULT_ESTOP
            self.v[:] = 0.0                       # arrêt immédiat (couple de maintien conservé)
            self.get_logger().warn('ARRÊT D\'URGENCE reçu : mouvement stoppé')

    def on_command(self, msg: Int32):
        c = msg.data
        if c == CMD_ENABLE:
            self.fault = 0
            self.t_hb = self._now()
            self.state = STATE_READY if self.homed == ALL_HOMED else STATE_IDLE
            self.q_cmd = None
        elif c == CMD_DISABLE:
            self.state, self.v[:] = STATE_DISABLED, 0.0
        elif c == CMD_HOME and self.state in (STATE_IDLE, STATE_READY):
            self.state, self.homed = STATE_HOMING, 0
            self._home_t0, self._home_from = self._now(), self.q.copy()
            self.get_logger().info('Homing simulé démarré')
        elif c == CMD_ZERO_HERE and self.state in (STATE_IDLE, STATE_READY):
            self.homed, self.state = ALL_HOMED, STATE_READY

    def on_gripper(self, msg: Float32):
        self.get_logger().info(f'Pince : {msg.data:.2f} ({"ouverte" if msg.data > 0.5 else "fermée"})')

    # ------------------------------------------------------------------ simulation ---
    def tick(self):
        now = self._now()
        dt = min(max(now - self._last, 1e-4), 0.05)
        self._last = now
        wd = self.get_parameter('watchdog_ms').value * 1e-3

        if self.state == STATE_READY:
            # Watchdog : consigne ou heartbeat manquant -> défaut (comme le firmware)
            stale_cmd = self.t_cmd is None or now - self.t_cmd > wd
            stale_hb = self.t_hb is not None and now - self.t_hb > wd
            if (stale_cmd and self.t_cmd is not None) or stale_hb:
                self.state, self.fault, self.v[:] = STATE_FAULT, FAULT_COMM, 0.0
                self.get_logger().error('Watchdog : liaison perdue -> DÉFAUT')
            elif self.q_cmd is not None:
                kp = self.get_parameter('kp').value
                v_des = np.clip(self.v_cmd + kp * (self.q_cmd - self.q), -self.cfg.v_max, self.cfg.v_max)
                dv = np.clip(v_des - self.v, -self.cfg.a_max * dt, self.cfg.a_max * dt)
                self.v = self.v + dv
                self.q = np.clip(self.q + self.v * dt, self.cfg.q_min, self.cfg.q_max)
                self.max_ferr = float(np.max(np.abs(self.q_cmd - self.q)))
        elif self.state == STATE_HOMING:
            self._simulate_homing(now)
        else:
            self.v[:] = 0.0

        eff = self.v / self.cfg.v_max      # "effort" = commande normalisée (pas de mesure de courant)
        self.pub_js.publish(make_joint_state(self.names, self.q, self.v, eff,
                                             stamp=self.get_clock().now().to_msg()))

    def _simulate_homing(self, now: float):
        """Chaque axe rejoint son fin de course, l'un après l'autre (ordre home_order)."""
        per = self.get_parameter('homing_time_per_axis').value
        k = int((now - self._home_t0) / per)
        for n, axis in enumerate(self.home_order):
            if n < k:
                self.q[axis] = self.home_pos[axis]
                self.homed |= 1 << axis
            elif n == k:
                frac = ((now - self._home_t0) - n * per) / per
                self.q[axis] = self._home_from[axis] + frac * (self.home_pos[axis] - self._home_from[axis])
        if k >= len(self.home_order):
            self.state, self.q_cmd = STATE_READY, None
            self.get_logger().info('Homing simulé terminé')

    def publish_status(self):
        st = [0] * ST_LEN
        st[0] = self.state
        st[1] = self.homed
        st[2] = 0
        st[3] = self.fault
        st[4] = int(self.max_ferr * 1000)
        st[5] = int(self.get_parameter('rate').value)
        st[6] = 0 if self.t_cmd is None else int((self._now() - self.t_cmd) * 1e3)
        st[7] = int(self._now() - self.t_start)
        self.pub_status.publish(Int32MultiArray(data=st))


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
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
