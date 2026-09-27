"""pick_node : « Aller saisir l'objet détecté ».

Séquence : ouvrir la pince -> approche (au-dessus de la cible) -> descente -> fermer la pince
           -> remontée -> retour à la pose de repos.

Trois façons de la lancer (même séquence) :
  * action  /pick_object (robot_interfaces/PickObject)  : clients ROS 2 natifs, `ros2 action send_goal`
  * service /pick/start  (std_srvs/Trigger)             : IHM distante (rosbridge, Windows) ; démarre en tâche de fond
  * service /pick/cancel (std_srvs/Trigger)
Le service publie sa progression sur /pick_status (std_msgs/String JSON) :
  {"phase": str, "progress": 0..1, "done": bool, "success": bool, "message": str}

Utilise /solve_ik pour chaque waypoint (config de départ = waypoint précédent : continuité articulaire),
publie des buts sur /joint_goal (exécutés par trajectory_node avec ses limites de vitesse/accélération)
et /gripper_command (Float32, 1 = ouverte). Hypothèse : pince à servo (0 fermée .. 1 ouverte).
"""
import copy
import json
import threading
import time
from typing import Callable, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import PickObject
from robot_interfaces.srv import SolveIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_kinematics.config import load_robot_config
from robot_kinematics.ros_utils import QOS_RELIABLE, QOS_SENSOR, joint_state_to_arrays


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
        self.declare_parameter('config_path', '')
        self.declare_parameter('approach_height', 0.10)     # m au-dessus de la cible
        self.declare_parameter('grasp_z_offset', 0.0)       # m : décalage de la pose de saisie
        self.declare_parameter('gripper_time', 1.0)         # s d'attente après ouverture/fermeture
        self.declare_parameter('move_timeout', 60.0)        # s max par mouvement
        self.declare_parameter('position_tolerance', 0.02)  # rad : "arrivé" si |q_mesure - but| < tol
        self.declare_parameter('return_to_rest', True)

        self.cfg = load_robot_config(self.get_parameter('config_path').value or None)
        self.q_meas: Optional[np.ndarray] = None
        self.latest_target: Optional[PoseStamped] = None
        self._busy = False
        self._cancel = threading.Event()
        cb = ReentrantCallbackGroup()

        self.pub_goal = self.create_publisher(JointTrajectory, '/joint_goal', QOS_RELIABLE)
        self.pub_gripper = self.create_publisher(Float32, '/gripper_command', QOS_RELIABLE)
        self.pub_status = self.create_publisher(String, '/pick_status', QOS_RELIABLE)
        self.create_subscription(JointState, '/joint_states', self.on_js, QOS_SENSOR, callback_group=cb)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target, QOS_RELIABLE, callback_group=cb)
        self.ik = self.create_client(SolveIK, '/solve_ik', callback_group=cb)
        self.server = ActionServer(self, PickObject, 'pick_object', execute_callback=self.execute,
                                   goal_callback=lambda g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=cb)
        self.create_service(Trigger, '/pick/start', self.on_start, callback_group=cb)
        self.create_service(Trigger, '/pick/cancel', self.on_cancel, callback_group=cb)
        self.get_logger().info('pick_node prêt (action /pick_object, services /pick/start et /pick/cancel)')

    def on_js(self, msg: JointState):
        arrays = joint_state_to_arrays(msg, self.cfg.joint_names)
        if arrays is not None:
            self.q_meas = arrays[0]

    def on_target(self, msg: PoseStamped):
        self.latest_target = msg

    # ------------------------------------------------------------------ primitives ----
    def _solve(self, pose: PoseStamped, seed: Optional[np.ndarray]):
        req = SolveIK.Request()
        req.target = pose
        req.seed = [] if seed is None else [float(x) for x in seed]
        fut = self.ik.call_async(req)
        while not fut.done():
            time.sleep(0.01)
        return fut.result()

    def _publish_goal(self, q: np.ndarray):
        goal = JointTrajectory()
        goal.joint_names = list(self.cfg.joint_names)
        goal.points = [JointTrajectoryPoint(positions=[float(x) for x in q])]
        self.pub_goal.publish(goal)

    def _wait_arrival(self, q: np.ndarray, cancelled: Callable[[], bool]) -> bool:
        """Attend que le robot atteigne q. Retourne False si annulé ou timeout."""
        tol = self.get_parameter('position_tolerance').value
        t_end = time.time() + self.get_parameter('move_timeout').value
        time.sleep(0.2)                       # laisse trajectory_node démarrer le segment
        while time.time() < t_end:
            if cancelled():
                if self.q_meas is not None:
                    self._publish_goal(self.q_meas)   # replanifie un arrêt en douceur
                return False
            if self.q_meas is not None and np.max(np.abs(self.q_meas - q)) < tol:
                return True
            time.sleep(0.02)
        return False

    def _gripper(self, opening: float):
        self.pub_gripper.publish(Float32(data=float(opening)))
        time.sleep(self.get_parameter('gripper_time').value)

    # ------------------------------------------------------------------ séquence ------
    def run_sequence(self, target: Optional[PoseStamped], approach_height: float,
                     feedback: Callable[[str, float], None],
                     cancelled: Callable[[], bool]) -> Tuple[bool, str, bool]:
        """Exécute la saisie. Retourne (succès, message, annulée)."""
        if target is None:
            return False, 'aucune cible disponible (/target_pose vide)', False
        if self.q_meas is None:
            return False, 'aucune mesure /joint_states', False
        if not self.ik.wait_for_service(timeout_sec=2.0):
            return False, 'service /solve_ik indisponible', False

        h = approach_height if approach_height > 0.0 else self.get_parameter('approach_height').value
        grasp = copy.deepcopy(target)            # copie : on ne modifie pas la cible d'origine
        grasp.pose.position.z += self.get_parameter('grasp_z_offset').value
        approach = copy.deepcopy(grasp)
        approach.pose.position.z += h

        # 1) calcul de toutes les configurations AVANT de bouger (échec = aucun mouvement)
        feedback('calcul cinématique inverse', 0.05)
        q_app = self._solve(approach, self.q_meas)
        if not q_app.reachable:
            return False, 'approche : ' + q_app.message, False
        q_gr = self._solve(grasp, np.array(q_app.joint_positions))
        if not q_gr.reachable:
            return False, 'saisie : ' + q_gr.message, False
        qa, qg = np.array(q_app.joint_positions), np.array(q_gr.joint_positions)

        def move(q):
            self._publish_goal(q)
            return self._wait_arrival(q, cancelled)

        steps = [
            ('ouverture de la pince', 0.10, lambda: self._gripper(1.0) or True),
            ('approche', 0.30, lambda: move(qa)),
            ('descente', 0.55, lambda: move(qg)),
            ('fermeture de la pince', 0.70, lambda: self._gripper(0.0) or True),
            ('remontée', 0.85, lambda: move(qa)),
        ]
        # 2) exécution
        for name, progress, action in steps:
            if cancelled():
                return False, 'annulée', True
            feedback(name, progress)
            if not action():
                if cancelled():
                    return False, 'annulée', True
                return False, f'{name} : but non atteint (timeout)', False

        if self.get_parameter('return_to_rest').value:
            feedback('retour au repos', 0.95)
            self._publish_goal(self.cfg.rest)
            self._wait_arrival(self.cfg.rest, cancelled)
        feedback('terminé', 1.0)
        return True, 'objet saisi', False

    # ------------------------------------------------------------------ action --------
    def execute(self, goal_handle):
        req = goal_handle.request
        result = PickObject.Result()
        fb = PickObject.Feedback()

        def feedback(phase: str, progress: float):
            fb.phase, fb.progress = phase, float(progress)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'Saisie : {phase}')

        target = self.latest_target if req.use_latest_target else req.target
        ok, msg, was_cancelled = self.run_sequence(target, req.approach_height, feedback,
                                                   lambda: goal_handle.is_cancel_requested)
        if ok:
            goal_handle.succeed()
        elif was_cancelled:
            goal_handle.canceled()
        else:
            self.get_logger().error('Saisie échouée : ' + msg)
            goal_handle.abort()
        result.success, result.message = ok, msg
        return result

    # ------------------------------------------------------------------ services -------
    def _status(self, phase: str, progress: float, done: bool = False, success: bool = False, message: str = ''):
        self.pub_status.publish(String(data=json.dumps(
            {'phase': phase, 'progress': float(progress), 'done': done, 'success': success, 'message': message},
            ensure_ascii=False)))

    def on_start(self, request: Trigger.Request, response: Trigger.Response):
        if self._busy:
            response.success, response.message = False, 'saisie déjà en cours'
            return response
        if self.latest_target is None:
            response.success, response.message = False, 'aucune cible disponible (/target_pose vide)'
            return response
        self._busy = True
        self._cancel.clear()
        target = copy.deepcopy(self.latest_target)

        def worker():
            try:
                ok, msg, _ = self.run_sequence(target, 0.0, lambda p, f: self._status(p, f), self._cancel.is_set)
            except Exception as exc:  # noqa: BLE001 - on veut toujours signaler la fin à l'IHM
                ok, msg = False, f'erreur interne : {exc}'
            self._status('terminé' if ok else 'échec', 1.0 if ok else 0.0, True, ok, msg)
            self._busy = False

        threading.Thread(target=worker, daemon=True).start()
        response.success, response.message = True, 'saisie démarrée'
        return response

    def on_cancel(self, request: Trigger.Request, response: Trigger.Response):
        self._cancel.set()
        response.success, response.message = True, 'annulation demandée'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PickNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
