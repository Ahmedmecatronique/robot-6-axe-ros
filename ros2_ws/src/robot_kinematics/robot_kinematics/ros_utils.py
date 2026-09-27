"""Conversions entre messages ROS 2 et tableaux NumPy / matrices 4x4 (partagé par tous les nœuds)."""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .transforms import matrix_to_pose, pose_to_matrix

# --- Profils QoS ----------------------------------------------------------------------------
# Données rapides / perdables (mesures, vidéo) : best effort, on ne garde que la dernière.
QOS_SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
# Commandes et arrêt d'urgence : fiables, aucune perte tolérée.
QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
# Valeurs d'état "latched" (ex. mode robot) : le dernier message est rejoué aux nouveaux abonnés.
QOS_LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def joint_state_to_arrays(msg: JointState, names: Sequence[str]
                          ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Extrait (position, vitesse, effort) dans l'ordre `names`. Retourne None si incomplet.

    Si le message n'a pas de noms (cas d'un firmware minimal), l'ordre est supposé identique.
    """
    n = len(names)
    if len(msg.name) == 0:
        if len(msg.position) < n:
            return None
        idx = list(range(n))
    else:
        try:
            idx = [list(msg.name).index(nm) for nm in names]
        except ValueError:
            return None

    def pick(arr):
        if len(arr) == 0:
            return np.zeros(n)
        try:
            return np.array([arr[i] for i in idx], dtype=float)
        except IndexError:
            return np.zeros(n)

    if len(msg.position) == 0:
        return None
    return pick(msg.position), pick(msg.velocity), pick(msg.effort)


def make_joint_state(names: Sequence[str], q, v=None, e=None, stamp: Optional[Time] = None) -> JointState:
    msg = JointState()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.name = list(names)
    msg.position = [float(x) for x in q]
    msg.velocity = [float(x) for x in (np.zeros(len(q)) if v is None else v)]
    msg.effort = [float(x) for x in (np.zeros(len(q)) if e is None else e)]
    return msg


def matrix_to_pose_stamped(T: np.ndarray, frame_id: str, stamp: Time) -> PoseStamped:
    (x, y, z), (qx, qy, qz, qw) = matrix_to_pose(T)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
    msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
    return msg


def pose_to_matrix_msg(pose: Pose) -> np.ndarray:
    p, o = pose.position, pose.orientation
    return pose_to_matrix((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))


def matrix_to_transform_stamped(T: np.ndarray, parent: str, child: str, stamp: Time) -> TransformStamped:
    (x, y, z), (qx, qy, qz, qw) = matrix_to_pose(T)
    ts = TransformStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = x, y, z
    ts.transform.rotation.x, ts.transform.rotation.y = qx, qy
    ts.transform.rotation.z, ts.transform.rotation.w = qz, qw
    return ts


def transform_stamped_to_matrix(ts: TransformStamped) -> np.ndarray:
    t, r = ts.transform.translation, ts.transform.rotation
    return pose_to_matrix((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
