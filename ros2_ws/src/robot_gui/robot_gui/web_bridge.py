"""Transport WEB (rosbridge WebSocket + roslibpy) de l'IHM — pour Windows, SANS installer ROS 2.

Robot (Pi ou simulation) : `rosbridge_websocket` écoute sur le port 9090 (lancé par core.launch.py).
PC : `pip install roslibpy` ; l'IHM se connecte à ws://<ip du Pi>:9090.

roslibpy exécute son propre thread réseau (Twisted) : les callbacks n'appellent que feed_*() du noyau.
Les messages sont des dictionnaires JSON ; les octets (image JPEG) arrivent en base64.

⚠ rosbridge n'a AUCUNE authentification : n'exposer le port 9090 qu'au réseau local (pare-feu).
"""
from __future__ import annotations

import base64
import json
from typing import Callable, Optional

import roslibpy

from .bridge_core import (BridgeCore, joint_state_dict_to_arrays, pose_dict_to_matrix, stamp_latency_ms)


def _bytes(data) -> bytes:
    """Octets d'un champ uint8[] : base64 (rosbridge par défaut) ou liste d'entiers."""
    if isinstance(data, str):
        return base64.b64decode(data)
    return bytes(bytearray(data))


class WebBridge(BridgeCore):
    def __init__(self, host: str, port: int = 9090, state_rate_hz: float = 50.0):
        super().__init__()
        self.host, self.port = host, port
        # Débit des flux rapides côté rosbridge : 50 Hz suffisent à l'affichage et divisent par 2 le travail
        # de décodage JSON (0 = tous les messages, 100 Hz).
        self._thr = int(1000.0 / state_rate_hz) if state_rate_hz > 0 else 0
        self.ros = roslibpy.Ros(host=host, port=port)
        self._pubs = {}
        self._subs = []
        self._subscribed = False
        self.ros.on_ready(self._on_ready, run_in_thread=True)
        self.ros.on('close', self._on_close)
        self.ros.run()                                  # non bloquant : le réseau tourne dans un thread

    # ---------------------------------------------------------------- connexion ---------
    @property
    def connected(self) -> bool:
        return bool(self.ros.is_connected)

    def _on_ready(self):
        self.push_event('log', f'Connecté à rosbridge ({self.host}:{self.port})', 'info')
        if self._subscribed:            # reconnexion : roslibpy ré-abonne lui-même les topics existants
            return
        self._subscribed = True

        def add(topic: str, msg_type: str, handler: Callable[[dict], None], throttle_ms: int = 0, queue_length: int = 0):
            t = roslibpy.Topic(self.ros, topic, msg_type, throttle_rate=throttle_ms, queue_length=queue_length)
            t.subscribe(handler)
            self._subs.append(t)

        for src, prefix in (('live', ''), ('replay', '/replay')):
            add(prefix + '/joint_states', 'sensor_msgs/msg/JointState', lambda m, s=src: self._js(s, m),
                throttle_ms=self._thr)
            add(prefix + '/joint_commands', 'sensor_msgs/msg/JointState', lambda m, s=src: self._cmd(s, m),
                throttle_ms=self._thr)
            add(prefix + '/end_effector_pose', 'geometry_msgs/msg/PoseStamped',
                lambda m, s=src: self.feed_ee(s, pose_dict_to_matrix(m['pose'])), throttle_ms=20)
            add(prefix + '/target_pose', 'geometry_msgs/msg/PoseStamped',
                lambda m, s=src: self.feed_target(s, pose_dict_to_matrix(m['pose'])))
            add(prefix + '/camera/image_annotated', 'sensor_msgs/msg/CompressedImage',
                lambda m, s=src: self.feed_image(s, _bytes(m['data'])), throttle_ms=66, queue_length=1)
        add('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', self._diag)
        add('/stm32/status', 'std_msgs/msg/Int32MultiArray', lambda m: self.feed_status(m.get('data') or []))
        add('/robot_mode', 'std_msgs/msg/String', lambda m: self.feed_mode(m.get('data', '')))
        add('/gui/pong', 'std_msgs/msg/Float64', lambda m: self.feed_pong(m.get('data', 0.0)))
        add('/ik_result', 'std_msgs/msg/String', lambda m: self._json(self.feed_ik_result, m))
        add('/pick_status', 'std_msgs/msg/String', lambda m: self._json(self.feed_pick_status, m))
        add('/bag/status', 'std_msgs/msg/String', lambda m: self._json(self.feed_bag_status, m))
        add('/target_visible', 'std_msgs/msg/Bool', lambda m: self.feed_visible(m.get('data', False)))
        add('/trajectory_active', 'std_msgs/msg/Bool', lambda m: self.feed_active(m.get('data', False)))

    def _on_close(self, *_):
        self.push_event('alarm', f'Liaison rosbridge perdue ({self.host}:{self.port}) — reconnexion automatique…',
                        'error')

    def shutdown(self):
        try:
            self.ros.terminate()
        except Exception:  # noqa: BLE001 - à la fermeture on ne veut jamais lever d'exception
            pass

    # ---------------------------------------------------------------- callbacks ---------
    def _js(self, src: str, m: dict):
        arrays = joint_state_dict_to_arrays(m, self.names)
        if arrays is not None:
            self.feed_joint_states(src, *arrays, stamp_latency_ms(m.get('header')))

    def _cmd(self, src: str, m: dict):
        arrays = joint_state_dict_to_arrays(m, self.names)
        if arrays is not None:
            self.feed_command(src, arrays[0], arrays[1])

    @staticmethod
    def _json(feed: Callable[[dict], None], m: dict):
        try:
            feed(json.loads(m.get('data', '{}')))
        except json.JSONDecodeError:
            pass

    def _diag(self, m: dict):
        parsed = []
        for st in m.get('status') or []:
            lvl = st.get('level', 0)
            parsed.append({'name': st.get('name', ''), 'level': lvl if isinstance(lvl, int) else ord(lvl),
                           'message': st.get('message', ''),
                           'values': {kv['key']: kv['value'] for kv in st.get('values') or []}})
        self.feed_diagnostics(parsed)

    # ---------------------------------------------------------------- transport ---------
    def _publish(self, topic: str, msg_type: str, payload: dict) -> None:
        if not self.connected:
            if topic != '/gui/ping':                    # le ping est silencieux hors connexion
                self.push_event('log', f'Non connecté : commande {topic} perdue', 'warn')
            return
        t = self._pubs.get(topic)
        if t is None:
            t = self._pubs[topic] = roslibpy.Topic(self.ros, topic, msg_type)
        t.publish(roslibpy.Message(payload))

    def _call(self, service: str, srv_type: str, request: dict, on_result: Callable[[Optional[dict]], None]) -> None:
        if not self.connected:
            on_result(None)
            return
        svc = roslibpy.Service(self.ros, service, srv_type)
        svc.call(roslibpy.ServiceRequest(request),
                 lambda response: on_result(dict(response)),
                 lambda error: on_result(None))
