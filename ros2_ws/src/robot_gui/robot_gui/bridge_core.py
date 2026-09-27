"""Noyau commun des ponts IHM <-> robot (AUCUNE dépendance à ROS : fonctionne aussi sous Windows).

Deux transports l'utilisent :
  * RosBridge (ros_bridge.py) : ROS 2 natif (rclpy) — Ubuntu ;
  * WebBridge (web_bridge.py) : WebSocket rosbridge (roslibpy) — Windows, aucune installation de ROS 2.

Le noyau range les DERNIÈRES valeurs reçues dans des tampons protégés par un verrou : l'interface Qt lit
snapshot() (jamais bloquant) et drain_events() ; les callbacks réseau n'appellent que feed_*().
Toutes les commandes sont exprimées en dictionnaires « JSON ROS » et envoyées par _publish() / _call(),
que chaque transport implémente.
"""
from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from robot_kinematics.config import RobotConfig, load_robot_config
from robot_kinematics.kinematics import ArmKinematics
from robot_kinematics.status_codes import ST_STATE, STATE_NAMES
from robot_kinematics.transforms import matrix_to_pose, pose_to_matrix

HISTORY_SECONDS = 30.0


# =============================================================================
#  Conversions dict <-> tableaux (format JSON de rosbridge)
# =============================================================================
def joint_state_dict_to_arrays(d: dict, names: Sequence[str]
                               ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(position, vitesse, effort) dans l'ordre `names` depuis un JointState en dict ; None si incomplet."""
    n = len(names)
    dn = list(d.get('name') or [])
    pos = d.get('position') or []
    if not pos:
        return None
    if dn:
        try:
            idx = [dn.index(nm) for nm in names]
        except ValueError:
            return None
    else:
        if len(pos) < n:
            return None
        idx = list(range(n))

    def pick(key):
        arr = d.get(key) or []
        try:
            return np.array([arr[i] for i in idx], dtype=float) if len(arr) else np.zeros(n)
        except IndexError:
            return np.zeros(n)

    return pick('position'), pick('velocity'), pick('effort')


def pose_dict_to_matrix(pose: dict) -> np.ndarray:
    p, o = pose['position'], pose['orientation']
    return pose_to_matrix((p['x'], p['y'], p['z']), (o['x'], o['y'], o['z'], o['w']))


def matrix_to_pose_stamped_dict(T: np.ndarray, frame_id: str) -> dict:
    (x, y, z), (qx, qy, qz, qw) = matrix_to_pose(T)
    return {'header': {'frame_id': frame_id, 'stamp': {'sec': 0, 'nanosec': 0}},
            'pose': {'position': {'x': x, 'y': y, 'z': z},
                     'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}}


def stamp_latency_ms(header: dict) -> Optional[float]:
    """Âge d'un message (ms) d'après son horodatage ; None si absent ou horloges non synchronisées."""
    st = (header or {}).get('stamp') or {}
    sec, nsec = st.get('sec', 0), st.get('nanosec', 0)
    if sec == 0 and nsec == 0:
        return None
    lat = (time.time() - (sec + nsec * 1e-9)) * 1e3
    return lat if abs(lat) < 5000 else None


class BridgeCore:
    """Tampons, instantané, historique, événements et commandes — indépendants du transport."""

    def __init__(self, cfg: Optional[RobotConfig] = None):
        self.cfg = cfg or load_robot_config()
        self.arm = ArmKinematics(self.cfg)
        self.names = self.cfg.joint_names
        self._lock = threading.Lock()
        self._events: 'queue.Queue[tuple]' = queue.Queue()
        self.source = 'live'                       # 'live' | 'replay'

        self.q_meas: Optional[np.ndarray] = None
        self.v_meas = np.zeros(6)
        self.effort = np.zeros(6)
        self.t_meas = 0.0
        self.js_rate = 0.0
        self.latency_ms: Optional[float] = None
        self.q_cmd: Optional[np.ndarray] = None
        self.v_cmd = np.zeros(6)
        self.t_cmd = 0.0
        self.ee_T: Optional[np.ndarray] = None
        self.target_T: Optional[np.ndarray] = None
        self.t_target = 0.0
        self.target_visible = False
        self.status: Optional[list] = None
        self.t_status = 0.0
        self.mode = 'inconnu'
        self.diagnostics: List[dict] = []
        self.traj_active = False
        self._image: Tuple[int, Optional[bytes]] = (0, None)
        # Historique en tampon ANNEAU DOUBLÉ (chaque échantillon écrit en i et i+N) : la fenêtre ordonnée des
        # derniers échantillons est toujours une tranche contiguë -> ajout en O(1), lecture sans réordonnancement.
        self._hN = int(HISTORY_SECONDS * 120)
        self._ht = np.zeros(2 * self._hN)
        self._hq = np.zeros((2 * self._hN, 6))
        self._hc = np.zeros((2 * self._hN, 6))
        self._hi = 0
        self._hcount = 0
        self.rtt_ms: Optional[float] = None          # aller-retour mesuré par ping/pong (indépendant des horloges)
        self._last_state = None
        self._diag_levels: dict = {}
        self.bag_status = {'recording': False, 'playing': False, 'current': '', 'names': []}
        self._replay_grace = 0.0

    # ==================================================== alimentation (transports) =====
    def push_event(self, *ev):
        self._events.put(ev)

    def feed_joint_states(self, src: str, q, v, e, latency_ms: Optional[float] = None):
        if src != self.source:
            return
        now = time.time()
        with self._lock:
            if self.t_meas > 0.0 and now > self.t_meas:
                inst = 1.0 / (now - self.t_meas)
                self.js_rate = inst if self.js_rate == 0.0 else 0.9 * self.js_rate + 0.1 * inst
            self.q_meas, self.v_meas, self.effort = q, v, e
            self.t_meas = now
            if latency_ms is not None:
                self.latency_ms = latency_ms if self.latency_ms is None else 0.9 * self.latency_ms + 0.1 * latency_ms
            c = self.q_cmd if self.q_cmd is not None else q
            i, n = self._hi, self._hN
            self._ht[i] = self._ht[i + n] = now
            self._hq[i] = self._hq[i + n] = q
            self._hc[i] = self._hc[i + n] = c
            self._hi = (i + 1) % n
            self._hcount = min(self._hcount + 1, n)

    def feed_command(self, src: str, q, v):
        if src == self.source:
            with self._lock:
                self.q_cmd, self.v_cmd, self.t_cmd = q, v, time.time()

    def feed_ee(self, src: str, T: np.ndarray):
        if src == self.source:
            with self._lock:
                self.ee_T = T

    def feed_target(self, src: str, T: np.ndarray):
        if src == self.source:
            with self._lock:
                self.target_T, self.t_target = T, time.time()

    def feed_image(self, src: str, data: bytes):
        if src == self.source:
            with self._lock:
                self._image = (self._image[0] + 1, data)

    def feed_visible(self, v: bool):
        with self._lock:
            self.target_visible = bool(v)

    def feed_active(self, a: bool):
        with self._lock:
            self.traj_active = bool(a)

    def feed_mode(self, mode: str):
        with self._lock:
            self.mode = mode

    def feed_status(self, data: Sequence[int]):
        if len(data) < 8:
            return
        with self._lock:
            self.status, self.t_status = [int(x) for x in data], time.time()
        state = int(data[ST_STATE])
        if state != self._last_state:
            self._last_state = state
            level = 'error' if state in (5, 6) else 'info'
            self.push_event('alarm' if level == 'error' else 'log',
                            f'État du robot : {STATE_NAMES.get(state, state)}', level)

    def feed_diagnostics(self, parsed: List[dict]):
        for st in parsed:
            lvl, prev = st['level'], self._diag_levels.get(st['name'], 0)
            if lvl >= 2 and prev < 2:                       # nouvelle alarme (niveau ERREUR)
                self.push_event('alarm', f"{st['name']} : {st['message']}", 'error')
            self._diag_levels[st['name']] = lvl
        with self._lock:
            self.diagnostics = parsed

    def feed_ik_result(self, d: dict):
        self.push_event('ik_result', d)

    def feed_pick_status(self, d: dict):
        if d.get('done'):
            self.push_event('pick_done', bool(d.get('success')), d.get('message', ''))
        else:
            self.push_event('pick_feedback', d.get('phase', ''), float(d.get('progress', 0.0)))

    def feed_bag_status(self, d: dict):
        with self._lock:
            self.bag_status = {'recording': bool(d.get('recording')), 'playing': bool(d.get('playing')),
                               'current': d.get('current', ''), 'names': list(d.get('names') or [])}

    # ================================================================== lecture ========
    def snapshot(self) -> dict:
        """Copie cohérente de l'état courant (appelée par le thread Qt)."""
        now = time.time()
        with self._lock:
            state = -1 if self.status is None else int(self.status[ST_STATE])
            return {
                'q_meas': None if self.q_meas is None else self.q_meas.copy(),
                'v_meas': self.v_meas.copy(), 'effort': self.effort.copy(),
                'q_cmd': None if self.q_cmd is None else self.q_cmd.copy(), 'v_cmd': self.v_cmd.copy(),
                'meas_age': now - self.t_meas if self.t_meas > 0 else float('inf'),
                'cmd_age': now - self.t_cmd if self.t_cmd > 0 else float('inf'),
                'js_rate': self.js_rate if now - self.t_meas < 1.0 else 0.0,
                'latency_ms': self.latency_ms,
                'ee_T': None if self.ee_T is None else self.ee_T.copy(),
                'target_T': None if self.target_T is None else self.target_T.copy(),
                'target_age': now - self.t_target if self.t_target > 0 else float('inf'),
                'target_visible': self.target_visible,
                'state': state, 'status': None if self.status is None else list(self.status),
                'status_age': now - self.t_status if self.t_status > 0 else float('inf'),
                'mode': self.mode, 'diagnostics': list(self.diagnostics),
                'traj_active': self.traj_active, 'source': self.source,
                'image_seq': self._image[0], 'rtt_ms': self.rtt_ms,
            }

    def image_jpeg(self) -> Tuple[int, Optional[bytes]]:
        with self._lock:
            return self._image

    def history(self, seconds: float = 10.0):
        """(t, mesure, consigne) sur les dernières `seconds` ; t est relatif à maintenant (<= 0)."""
        now = time.time()
        with self._lock:
            n = self._hcount
            if n == 0:
                return np.zeros(0), np.zeros((0, 6)), np.zeros((0, 6))
            start = self._hi + self._hN - n            # plus ancien échantillon, dans le tampon doublé
            t_all = self._ht[start:start + n]
            k = int(np.searchsorted(t_all, now - seconds))
            lo, hi = start + k, start + n
            return self._ht[lo:hi] - now, self._hq[lo:hi].copy(), self._hc[lo:hi].copy()

    def drain_events(self) -> list:
        out = []
        while True:
            try:
                out.append(self._events.get_nowait())
            except queue.Empty:
                return out

    def set_source(self, source: str):
        with self._lock:
            self.source = source
            self.q_meas = self.q_cmd = self.ee_T = None
            self.t_meas = self.t_cmd = 0.0
            self.js_rate = 0.0
            self._hi = self._hcount = 0

    # ====================================================== latence réelle (ping) =======
    def send_ping(self):
        """Ping horodaté ; safety_node le renvoie sur /gui/pong. Donne le VRAI aller-retour réseau."""
        self._publish('/gui/ping', 'std_msgs/msg/Float64', {'data': time.time()})

    def feed_pong(self, sent: float):
        rtt = (time.time() - float(sent)) * 1e3
        if 0.0 <= rtt < 5000.0:
            with self._lock:
                self.rtt_ms = rtt if self.rtt_ms is None else 0.8 * self.rtt_ms + 0.2 * rtt

    # =================================================== transport (à implémenter) ======
    def _publish(self, topic: str, msg_type: str, payload: dict) -> None:
        raise NotImplementedError

    def _call(self, service: str, srv_type: str, request: dict, on_result: Callable[[Optional[dict]], None]) -> None:
        """Appel de service asynchrone ; on_result(dict) ou on_result(None) en cas d'échec."""
        raise NotImplementedError

    def shutdown(self) -> None:
        pass

    # ================================================================= commandes ========
    def send_joint_goal(self, waypoints: Sequence[Sequence[float]]):
        """Un ou plusieurs buts articulaires (rad). trajectory_node applique les limites de vitesse."""
        self._publish('/joint_goal', 'trajectory_msgs/msg/JointTrajectory', {
            'joint_names': list(self.names),
            'points': [{'positions': [float(x) for x in q], 'time_from_start': {'sec': 0, 'nanosec': 0}}
                       for q in waypoints]})

    def send_target_pose(self, T: np.ndarray):
        """Pose cible (base_link) : ik_node calcule l'IK et publie le but."""
        self._publish('/target_pose', 'geometry_msgs/msg/PoseStamped',
                      matrix_to_pose_stamped_dict(T, self.cfg.base_frame))

    def send_estop(self):
        """ARRÊT D'URGENCE : publié plusieurs fois (fiable) pour garantir la réception."""
        for _ in range(3):
            self._publish('/emergency_stop', 'std_msgs/msg/Bool', {'data': True})
        self.push_event('alarm', 'ARRÊT D\'URGENCE déclenché depuis l\'IHM', 'error')

    def soft_stop(self):
        """Arrêt doux : nouveau but = position mesurée -> décélération planifiée."""
        q = self.snapshot()['q_meas']
        if q is not None:
            self.send_joint_goal([q])

    def go_rest(self):
        self.send_joint_goal([self.cfg.rest])

    def send_gripper(self, opening: float):
        self._publish('/gripper_command', 'std_msgs/msg/Float32', {'data': float(opening)})

    def set_speed_scale(self, k: float):
        def done(r):
            if r is None:
                self.push_event('log', 'trajectory_node injoignable : vitesse inchangée', 'warn')
        self._call('/trajectory_node/set_parameters', 'rcl_interfaces/srv/SetParameters',
                   {'parameters': [{'name': 'speed_scale', 'value': {'type': 3, 'double_value': float(k)}}]}, done)

    def _service(self, service: str, srv_type: str, request: dict, label: str):
        def done(r):
            if r is None:
                self.push_event('log', f'Service {label} indisponible', 'warn')
            else:
                self.push_event('service', label, bool(r.get('success')), r.get('message', ''))
        self._call(service, srv_type, request, done)

    def call_enable(self, enable: bool):
        self._service('/robot/enable', 'robot_interfaces/srv/SetEnable', {'enable': bool(enable)},
                      'activation' if enable else 'désactivation')

    def call_home(self):
        self._service('/robot/home', 'robot_interfaces/srv/Home', {}, 'homing')

    def call_solve_ik(self, T: np.ndarray):
        """Prévisualisation IK sans mouvement ; résultat via l'événement ('ik_preview', dict | None)."""
        self._call('/solve_ik', 'robot_interfaces/srv/SolveIK',
                   {'target': matrix_to_pose_stamped_dict(T, self.cfg.base_frame), 'seed': []},
                   lambda r: self.push_event('ik_preview', r))

    # ---- saisie d'objet (services /pick/*, progression sur /pick_status)
    def pick_detected(self):
        def done(r):
            if r is None:
                self.push_event('pick_done', False, 'service /pick/start indisponible (pick_node lancé ?)')
            elif not r.get('success'):
                self.push_event('pick_done', False, r.get('message', 'refusé'))
        self._call('/pick/start', 'std_srvs/srv/Trigger', {}, done)

    def cancel_pick(self):
        self._call('/pick/cancel', 'std_srvs/srv/Trigger', {}, lambda r: None)

    # ---- rosbag2 : exécuté CÔTÉ ROBOT par bag_node (l'IHM n'a pas besoin de ROS 2)
    def _bag(self, command: str, name: str = '', on_fail: Optional[Callable[[], None]] = None):
        def done(r):
            if r is None or not r.get('success'):
                self.push_event('log', 'rosbag : ' + ('bag_node injoignable' if r is None else r.get('message', '')),
                                'warn')
                if on_fail:
                    on_fail()
            else:
                self.push_event('log', 'rosbag : ' + r.get('message', ''), 'info')
        self._call('/bag/command', 'robot_interfaces/srv/BagCommand', {'command': command, 'name': name}, done)

    def bag_record_start(self) -> Tuple[bool, str]:
        self._bag('record_start')
        return True, 'demande envoyée au robot'

    def bag_record_stop(self):
        self._bag('record_stop')

    def bag_refresh(self):
        self._bag('list')

    def bag_recording(self) -> bool:
        return bool(self.bag_status['recording'])

    def bag_playing(self) -> bool:
        return bool(self.bag_status['playing'])

    def list_bags(self) -> List[str]:
        return list(self.bag_status['names'])

    def bag_replay_start(self, name: str) -> Tuple[bool, str]:
        """Rejoue une session redirigée vers /replay/... : le robot ne reçoit RIEN."""
        self.set_source('replay')
        self._replay_grace = time.time() + 4.0       # le temps que /bag/status confirme « playing »
        self._bag('play', name, on_fail=lambda: self.set_source('live'))
        return True, name

    def replay_finished(self) -> bool:
        """Vrai quand le rejeu s'est terminé tout seul (à ne pas confondre avec son démarrage)."""
        return self.source == 'replay' and not self.bag_playing() and time.time() > self._replay_grace

    def bag_replay_stop(self):
        self._bag('play_stop')
        self.set_source('live')
