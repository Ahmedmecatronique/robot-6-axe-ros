"""Transport ROS 2 NATIF (rclpy) de l'IHM — pour Ubuntu (ou toute machine avec ROS 2 Humble).

Sous Windows, utiliser WebBridge (web_bridge.py) : aucune installation de ROS 2.

Le nœud « gui_node » tourne dans un thread séparé ; ses callbacks ne font que ranger les dernières valeurs
dans le noyau (BridgeCore). Les commandes (publications, services) passent par _publish() / _call().

Abonnements : /joint_states, /joint_commands, /end_effector_pose, /camera/image_annotated, /target_pose,
              /diagnostics, /stm32/status, /robot_mode, /ik_result, /target_visible, /trajectory_active,
              /pick_status, /bag/status
En rejeu de rosbag, les flux de données sont lus sur /replay/... (jamais envoyés au robot).
"""
from __future__ import annotations

import json
import threading
from functools import partial
from typing import Callable, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64, Int32MultiArray, String

from robot_kinematics.ros_utils import (QOS_LATCHED, QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays,
                                        pose_to_matrix_msg)

from .bridge_core import BridgeCore

_KNOWN_PUBLISHERS = [
    ('/joint_goal', 'trajectory_msgs/msg/JointTrajectory'),
    ('/target_pose', 'geometry_msgs/msg/PoseStamped'),
    ('/emergency_stop', 'std_msgs/msg/Bool'),
    ('/gripper_command', 'std_msgs/msg/Float32'),
    ('/gui/ping', 'std_msgs/msg/Float64'),
]
_KNOWN_SERVICES = [
    ('/robot/home', 'robot_interfaces/srv/Home'),
    ('/robot/enable', 'robot_interfaces/srv/SetEnable'),
    ('/solve_ik', 'robot_interfaces/srv/SolveIK'),
    ('/trajectory_node/set_parameters', 'rcl_interfaces/srv/SetParameters'),
    ('/pick/start', 'std_srvs/srv/Trigger'),
    ('/pick/cancel', 'std_srvs/srv/Trigger'),
    ('/bag/command', 'robot_interfaces/srv/BagCommand'),
]


def _level_int(x) -> int:
    return x if isinstance(x, int) else ord(x)


class RosBridge(BridgeCore):
    def __init__(self):
        super().__init__()
        self.node: Node = rclpy.create_node('gui_node')
        self._pubs = {}
        self._clients = {}
        self._thread: Optional[threading.Thread] = None
        node = self.node

        # ---- flux de données : direct + rejeu
        for src, prefix in (('live', ''), ('replay', '/replay')):
            node.create_subscription(JointState, prefix + '/joint_states', partial(self._on_js, src), QOS_SENSOR)
            node.create_subscription(JointState, prefix + '/joint_commands', partial(self._on_cmd, src), QOS_RELIABLE)
            node.create_subscription(PoseStamped, prefix + '/end_effector_pose', partial(self._on_ee, src), QOS_SENSOR)
            node.create_subscription(PoseStamped, prefix + '/target_pose', partial(self._on_target, src), QOS_RELIABLE)
            node.create_subscription(CompressedImage, prefix + '/camera/image_annotated',
                                     partial(self._on_image, src), QOS_SENSOR)
        node.create_subscription(DiagnosticArray, '/diagnostics', self._on_diag, QOS_RELIABLE)
        node.create_subscription(Int32MultiArray, '/stm32/status', lambda m: self.feed_status(m.data), QOS_SENSOR)
        node.create_subscription(String, '/robot_mode', lambda m: self.feed_mode(m.data), QOS_LATCHED)
        node.create_subscription(String, '/ik_result', partial(self._on_json, self.feed_ik_result), QOS_RELIABLE)
        node.create_subscription(String, '/pick_status', partial(self._on_json, self.feed_pick_status), QOS_RELIABLE)
        node.create_subscription(String, '/bag/status', partial(self._on_json, self.feed_bag_status), QOS_SENSOR)
        node.create_subscription(Bool, '/target_visible', lambda m: self.feed_visible(m.data), QOS_RELIABLE)
        node.create_subscription(Float64, '/gui/pong', lambda m: self.feed_pong(m.data), QOS_RELIABLE)
        node.create_subscription(Bool, '/trajectory_active', lambda m: self.feed_active(m.data), QOS_RELIABLE)

        # ---- publishers et clients créés d'avance (découverte DDS plus rapide)
        for topic, type_str in _KNOWN_PUBLISHERS:
            self._get_pub(topic, type_str)
        for name, type_str in _KNOWN_SERVICES:
            self._get_client(name, type_str)

    # ---------------------------------------------------------------- cycle de vie ------
    def start(self):
        self._thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()

    def shutdown(self):
        self.node.destroy_node()

    # ---------------------------------------------------------------- transport ---------
    def _get_pub(self, topic: str, type_str: str):
        if topic not in self._pubs:
            cls = get_message(type_str)
            self._pubs[topic] = (self.node.create_publisher(cls, topic, QOS_RELIABLE), cls)
        return self._pubs[topic]

    def _get_client(self, name: str, type_str: str):
        if name not in self._clients:
            srv = get_service(type_str)
            self._clients[name] = (self.node.create_client(srv, name), srv)
        return self._clients[name]

    def _publish(self, topic: str, msg_type: str, payload: dict) -> None:
        pub, cls = self._get_pub(topic, msg_type)
        msg = cls()
        set_message_fields(msg, payload)
        pub.publish(msg)

    def _call(self, service: str, srv_type: str, request: dict, on_result: Callable[[Optional[dict]], None]) -> None:
        client, srv = self._get_client(service, srv_type)
        if not client.service_is_ready():
            on_result(None)
            return
        req = srv.Request()
        set_message_fields(req, request)
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: on_result(dict(message_to_ordereddict(f.result()))))

    # ---------------------------------------------------------------- callbacks ---------
    def _stamp_latency(self, msg) -> Optional[float]:
        s = msg.header.stamp
        if s.sec == 0 and s.nanosec == 0:
            return None
        lat = (self.node.get_clock().now().nanoseconds - (s.sec * 10 ** 9 + s.nanosec)) / 1e6
        return lat if abs(lat) < 5000 else None    # horloges non synchronisées : valeur ignorée

    def _on_js(self, src: str, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is not None:
            self.feed_joint_states(src, *arrays, self._stamp_latency(msg))

    def _on_cmd(self, src: str, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.names)
        if arrays is not None:
            self.feed_command(src, arrays[0], arrays[1])

    def _on_ee(self, src: str, msg: PoseStamped):
        self.feed_ee(src, pose_to_matrix_msg(msg.pose))

    def _on_target(self, src: str, msg: PoseStamped):
        self.feed_target(src, pose_to_matrix_msg(msg.pose))

    def _on_image(self, src: str, msg: CompressedImage):
        self.feed_image(src, bytes(msg.data))

    def _on_json(self, feed: Callable[[dict], None], msg: String):
        try:
            feed(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _on_diag(self, msg: DiagnosticArray):
        self.feed_diagnostics([
            {'name': st.name, 'level': _level_int(st.level), 'message': st.message,
             'values': {kv.key: kv.value for kv in st.values}} for st in msg.status])
