"""safety_node : arrêt d'urgence, limites articulaires, watchdog, diagnostics, supervision.

Abonnements : /emergency_stop (Bool, reliable)   demande d'arrêt (IHM, safety_node lui-même, autre)
              /joint_states   (JointState, best effort)  mesure -> limites, watchdog de liaison
              /joint_commands (JointState, reliable)      consigne -> écart de poursuite
              /stm32/status   (Int32MultiArray, best effort)
Publications : /emergency_stop (Bool, reliable, 10 Hz) : HEARTBEAT (False = tout va bien, True = arrêt).
                                Le STM32 s'arrête dès qu'il reçoit True OU si le heartbeat disparaît.
               /diagnostics (DiagnosticArray, 2 Hz)
               /stm32/command (Int32) : ENABLE / DISABLE / HOME vers le STM32
               /robot_mode (String, latched) : "real" ou "simulation"
Services     : /robot/home   (robot_interfaces/Home)
               /robot/enable (robot_interfaces/SetEnable) : efface le verrou d'arrêt d'urgence

L'arrêt d'urgence est VERROUILLÉ : une fois déclenché il reste actif jusqu'à /robot/enable(true).
IMPORTANT : ceci est un arrêt logiciel (les moteurs gardent leur couple de maintien pour que le bras ne
tombe pas). L'arrêt de sécurité réel doit être un bouton physique coupant l'alimentation moteurs.
"""
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from robot_interfaces.srv import Home, SetEnable
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String

from robot_kinematics.config import load_robot_config
from robot_kinematics.ros_utils import QOS_LATCHED, QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays
from robot_kinematics.status_codes import (CMD_DISABLE, CMD_ENABLE, CMD_HOME, ST_CMD_AGE_MS,
                                           ST_FAULT_CODE, ST_HOMED_MASK, ST_LIMIT_MASK, ST_LOOP_HZ,
                                           ST_MAX_FERR, ST_STATE, STATE_DISABLED, STATE_ESTOP,
                                           STATE_FAULT, STATE_HOMING, STATE_IDLE, STATE_NAMES,
                                           STATE_READY, decode_faults)


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('mode', 'real')                    # 'real' | 'simulation'
        self.declare_parameter('heartbeat_rate', 10.0)            # Hz
        self.declare_parameter('joint_state_timeout', 0.3)        # s sans /joint_states -> arrêt
        self.declare_parameter('limit_margin_deg', 2.0)           # tolérance sur les limites
        self.declare_parameter('following_error_max_deg', 10.0)   # écart consigne/mesure toléré
        self.declare_parameter('following_error_time', 0.5)       # s au-delà du seuil -> arrêt

        self.cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.names = self.cfg.joint_names

        self.latched = False
        self.latch_reason = ''
        self.q_meas = None
        self.q_cmd = None
        self.t_js = None            # dernière réception /joint_states
        self.t_cmd = None
        self.js_rate = 0.0          # fréquence estimée (Hz)
        self.status = None          # dernier /stm32/status
        self.t_status = None
        self.ferr_since = None
        self.max_ferr_deg = 0.0
        self._enable_timer = None

        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', QOS_RELIABLE)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', QOS_RELIABLE)
        self.pub_cmd = self.create_publisher(Int32, '/stm32/command', QOS_RELIABLE)
        self.pub_mode = self.create_publisher(String, '/robot_mode', QOS_LATCHED)
        self.pub_pong = self.create_publisher(Float64, '/gui/pong', QOS_RELIABLE)

        self.create_subscription(Bool, '/emergency_stop', self.on_estop, QOS_RELIABLE)
        # l'IHM mesure ainsi son VRAI aller-retour réseau, sans dépendre de la synchronisation des horloges
        self.create_subscription(Float64, '/gui/ping', lambda m: self.pub_pong.publish(m), QOS_RELIABLE)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, QOS_SENSOR)
        self.create_subscription(JointState, '/joint_commands', self.on_joint_commands, QOS_RELIABLE)
        self.create_subscription(Int32MultiArray, '/stm32/status', self.on_status, QOS_SENSOR)

        self.create_service(Home, '/robot/home', self.on_home)
        self.create_service(SetEnable, '/robot/enable', self.on_enable)

        self.create_timer(1.0 / self.get_parameter('heartbeat_rate').value, self.heartbeat)
        self.create_timer(0.05, self.check)          # vérifications à 20 Hz
        self.create_timer(0.5, self.publish_diagnostics)
        self.create_timer(1.0, self.publish_mode)   # republié : une IHM qui se connecte plus tard le reçoit
        self.publish_mode()
        self.get_logger().info(f"safety_node prêt (mode : {self.get_parameter('mode').value})")

    # ----------------------------------------------------------------- outils ------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def latch(self, reason: str):
        """Déclenche l'arrêt d'urgence verrouillé."""
        if not self.latched:
            self.latched = True
            self.latch_reason = reason
            self.get_logger().error(f'ARRÊT D\'URGENCE : {reason}')
        self.pub_estop.publish(Bool(data=True))       # immédiat, sans attendre le heartbeat

    @property
    def state(self) -> int:
        return -1 if self.status is None else int(self.status[ST_STATE])

    # ------------------------------------------------------------------ entrées -----
    def on_estop(self, msg: Bool):
        if msg.data and not self.latched:
            self.latch('demande d\'arrêt reçue sur /emergency_stop')

    def on_joint_states(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is None:
            return
        now = self._now()
        if self.t_js is not None and now > self.t_js:
            inst = 1.0 / (now - self.t_js)
            self.js_rate = inst if self.js_rate == 0.0 else 0.9 * self.js_rate + 0.1 * inst
        self.t_js, self.q_meas = now, arrays[0]

    def on_joint_commands(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is not None:
            self.t_cmd, self.q_cmd = self._now(), arrays[0]

    def on_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 8:
            self.status, self.t_status = list(msg.data), self._now()

    # -------------------------------------------------------------- services --------
    def on_enable(self, request: SetEnable.Request, response: SetEnable.Response):
        if not request.enable:
            self.pub_cmd.publish(Int32(data=CMD_DISABLE))
            response.success, response.message = True, 'Moteurs désactivés'
            return response
        # 1) on efface le verrou (le heartbeat repasse à False), 2) puis on ordonne l'activation
        self.latched, self.latch_reason = False, ''
        self.ferr_since = None
        self.pub_estop.publish(Bool(data=False))
        if self._enable_timer is not None:
            self._enable_timer.cancel()
        self._enable_timer = self.create_timer(0.3, self._send_enable)
        response.success, response.message = True, 'Arrêt d\'urgence effacé, activation demandée'
        return response

    def _send_enable(self):
        self._enable_timer.cancel()
        self.pub_cmd.publish(Int32(data=CMD_ENABLE))

    def on_home(self, request: Home.Request, response: Home.Response):
        if self.latched:
            response.success, response.message = False, 'Arrêt d\'urgence actif : appeler /robot/enable d\'abord'
        elif self.state in (STATE_DISABLED, STATE_ESTOP, STATE_FAULT):
            response.success, response.message = False, \
                f'État {STATE_NAMES.get(self.state, self.state)} : activer les moteurs d\'abord'
        else:
            self.pub_cmd.publish(Int32(data=CMD_HOME))
            response.success, response.message = True, 'Homing lancé'
        return response

    # ------------------------------------------------------------ boucles ------------
    def publish_mode(self):
        self.pub_mode.publish(String(data=str(self.get_parameter('mode').value)))

    def heartbeat(self):
        self.pub_estop.publish(Bool(data=self.latched))

    def check(self):
        now = self._now()
        # Liaison STM32 : on ne juge qu'une fois la première mesure reçue
        timeout = self.get_parameter('joint_state_timeout').value
        if self.t_js is not None and now - self.t_js > timeout and not self.latched:
            self.latch(f'perte de /joint_states depuis {now - self.t_js:.2f} s (liaison STM32 ?)')

        if self.q_meas is None or self.state != STATE_READY:
            self.ferr_since = None
            return

        # Limites articulaires (mesurées)
        margin = np.deg2rad(self.get_parameter('limit_margin_deg').value)
        bad = np.where((self.q_meas < self.cfg.q_min - margin) | (self.q_meas > self.cfg.q_max + margin))[0]
        if len(bad) and not self.latched:
            self.latch('limite articulaire dépassée : ' + ', '.join(self.names[i] for i in bad))

        # Écart de poursuite consigne/mesure
        if self.q_cmd is not None and self.t_cmd is not None and now - self.t_cmd < 0.3:
            err = np.abs(self.q_cmd - self.q_meas)
            self.max_ferr_deg = float(np.rad2deg(np.max(err)))
            if self.max_ferr_deg > self.get_parameter('following_error_max_deg').value:
                self.ferr_since = self.ferr_since or now
                if now - self.ferr_since > self.get_parameter('following_error_time').value \
                        and not self.latched:
                    self.latch(f'écart de poursuite {self.max_ferr_deg:.1f}° (moteur bloqué / décroché ?)')
            else:
                self.ferr_since = None

    def publish_diagnostics(self):
        now = self._now()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        def status(name, level, message, **values):
            s = DiagnosticStatus(name=name, level=level, message=message, hardware_id='arm6')
            s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            return s

        # 1) liaison STM32
        age = None if self.t_js is None else now - self.t_js
        if age is None:
            arr.status.append(status('arm6/liaison_stm32', DiagnosticStatus.ERROR, 'aucune mesure reçue',
                                     rate_hz=0.0))
        elif age > self.get_parameter('joint_state_timeout').value:
            arr.status.append(status('arm6/liaison_stm32', DiagnosticStatus.ERROR,
                                     f'liaison perdue ({age:.2f} s)', rate_hz=0.0, age_ms=int(age * 1e3)))
        else:
            arr.status.append(status('arm6/liaison_stm32', DiagnosticStatus.OK, 'connecté',
                                     rate_hz=f'{self.js_rate:.1f}', age_ms=int(age * 1e3)))

        # 2) état du robot (STM32)
        if self.status is None:
            arr.status.append(status('arm6/etat_robot', DiagnosticStatus.WARN, 'pas de /stm32/status'))
        else:
            st = self.status
            level = {STATE_READY: DiagnosticStatus.OK, STATE_HOMING: DiagnosticStatus.OK,
                     STATE_DISABLED: DiagnosticStatus.WARN, STATE_IDLE: DiagnosticStatus.WARN,
                     STATE_ESTOP: DiagnosticStatus.ERROR, STATE_FAULT: DiagnosticStatus.ERROR
                     }.get(st[ST_STATE], DiagnosticStatus.WARN)
            arr.status.append(status(
                'arm6/etat_robot', level, STATE_NAMES.get(st[ST_STATE], str(st[ST_STATE])),
                homed_mask=st[ST_HOMED_MASK], limit_switch_mask=st[ST_LIMIT_MASK],
                fault=decode_faults(st[ST_FAULT_CODE]), max_following_error_mrad=st[ST_MAX_FERR],
                stm32_loop_hz=st[ST_LOOP_HZ], cmd_age_ms=st[ST_CMD_AGE_MS]))

        # 3) sécurité logicielle
        if self.latched:
            arr.status.append(status('arm6/securite', DiagnosticStatus.ERROR, 'ARRÊT D\'URGENCE',
                                     raison=self.latch_reason))
        else:
            arr.status.append(status('arm6/securite', DiagnosticStatus.OK, 'OK',
                                     max_following_error_deg=f'{self.max_ferr_deg:.2f}',
                                     mode=self.get_parameter('mode').value))

        # 4) écart consigne/mesure par articulation (pour les courbes/alarmes de l'IHM)
        if self.q_meas is not None and self.q_cmd is not None:
            err = np.rad2deg(self.q_cmd - self.q_meas)
            arr.status.append(status('arm6/articulations', DiagnosticStatus.OK, 'écarts consigne-mesure',
                                     **{f'err_deg_{i + 1}': f'{err[i]:.3f}' for i in range(6)}))
        self.pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
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
