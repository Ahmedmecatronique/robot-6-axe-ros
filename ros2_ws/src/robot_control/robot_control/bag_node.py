"""bag_node : enregistre / rejoue des sessions rosbag2 CÔTÉ ROBOT, pilotable à distance par l'IHM.

Service   /bag/command (robot_interfaces/BagCommand) : record_start | record_stop | play | play_stop | list
Publie    /bag/status (std_msgs/String JSON, 1 Hz) : {"recording", "playing", "current", "names": [...]}

Le rejeu est REDIRIGÉ vers /replay/<topic> : le robot ne reçoit jamais de commande rejouée, et le TF
en direct n'est pas perturbé (/tf n'est pas rejoué). L'IHM affiche alors les flux /replay/....
"""
import json
import os
import signal
import subprocess
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import BagCommand
from std_msgs.msg import String

RECORD_TOPICS = ['/joint_states', '/joint_commands', '/end_effector_pose', '/target_pose', '/tf', '/tf_static',
                 '/diagnostics', '/stm32/status', '/camera/image_annotated', '/ik_result']
# Flux rejoués (chacun est remappé sous /replay) — jamais /emergency_stop, /joint_goal, /stm32/command...
REPLAY_TOPICS = ['/joint_states', '/joint_commands', '/end_effector_pose', '/target_pose',
                 '/camera/image_annotated']


class BagNode(Node):
    def __init__(self):
        super().__init__('bag_node')
        self.declare_parameter('bag_dir', os.path.expanduser('~/robot_bags'))
        self.bag_dir = os.path.expanduser(self.get_parameter('bag_dir').value)
        os.makedirs(self.bag_dir, exist_ok=True)
        self._rec: Optional[subprocess.Popen] = None
        self._play: Optional[subprocess.Popen] = None
        self._current = ''
        self.pub = self.create_publisher(String, '/bag/status', 1)
        self.create_service(BagCommand, '/bag/command', self.on_command)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'bag_node prêt (dossier : {self.bag_dir})')

    # ------------------------------------------------------------------ état ---------
    def names(self) -> List[str]:
        return sorted(d for d in os.listdir(self.bag_dir) if os.path.isdir(os.path.join(self.bag_dir, d)))

    def recording(self) -> bool:
        return self._rec is not None and self._rec.poll() is None

    def playing(self) -> bool:
        return self._play is not None and self._play.poll() is None

    def publish_status(self):
        self.pub.publish(String(data=json.dumps({
            'recording': self.recording(), 'playing': self.playing(),
            'current': self._current, 'names': self.names()}, ensure_ascii=False)))

    @staticmethod
    def _stop(proc: subprocess.Popen):
        try:
            proc.send_signal(signal.SIGINT)          # arrêt propre : rosbag2 finalise metadata.yaml
            proc.wait(timeout=8)
        except Exception:  # noqa: BLE001
            proc.kill()

    # ------------------------------------------------------------------ service ------
    def on_command(self, req: BagCommand.Request, res: BagCommand.Response):
        cmd = req.command
        try:
            if cmd == 'record_start':
                if self.recording():
                    res.success, res.message = False, 'enregistrement déjà en cours'
                else:
                    name = req.name or time.strftime('session_%Y%m%d_%H%M%S')
                    path = os.path.join(self.bag_dir, os.path.basename(name))   # basename : pas d'évasion de dossier
                    self._rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', path] + RECORD_TOPICS,
                                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self._current = os.path.basename(path)
                    res.success, res.message = True, f'enregistrement : {path}'
            elif cmd == 'record_stop':
                if self.recording():
                    self._stop(self._rec)
                self._rec, self._current = None, ''
                res.success, res.message = True, 'enregistrement arrêté'
            elif cmd == 'play':
                path = os.path.join(self.bag_dir, os.path.basename(req.name))
                if self.playing():
                    res.success, res.message = False, 'rejeu déjà en cours'
                elif not os.path.isdir(path):
                    res.success, res.message = False, f'session introuvable : {req.name}'
                else:
                    remap = [f'{t}:=/replay{t}' for t in REPLAY_TOPICS]
                    self._play = subprocess.Popen(['ros2', 'bag', 'play', path, '--topics'] + REPLAY_TOPICS
                                                  + ['--remap'] + remap,
                                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self._current = os.path.basename(path)
                    res.success, res.message = True, f'rejeu : {path}'
            elif cmd == 'play_stop':
                if self.playing():
                    self._stop(self._play)
                self._play, self._current = None, ''
                res.success, res.message = True, 'rejeu arrêté'
            elif cmd == 'list':
                res.success, res.message = True, f'{len(self.names())} session(s)'
            else:
                res.success, res.message = False, f'commande inconnue : {cmd}'
        except OSError as exc:
            res.success, res.message = False, f'ros2 bag indisponible : {exc}'
        self.publish_status()
        return res

    def destroy_node(self):
        for proc in (self._rec, self._play):
            if proc is not None and proc.poll() is None:
                self._stop(proc)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagNode()
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
