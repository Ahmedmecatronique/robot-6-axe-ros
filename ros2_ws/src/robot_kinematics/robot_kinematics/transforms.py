"""Outils de transformations rigides (NumPy pur, sans dépendance à tf_transformations).

Conventions : quaternion = (x, y, z, w) comme dans ROS ; angles RPY = rotations fixes
autour de X, Y, Z :  R = Rz(yaw) · Ry(pitch) · Rx(roll)  (convention URDF).
"""
from __future__ import annotations

import numpy as np


def wrap_angle(x):
    """Ramène un angle (ou un tableau) dans ]-pi, pi]."""
    return (np.asarray(x) + np.pi) % (2.0 * np.pi) - np.pi if np.ndim(x) else \
        float((x + np.pi) % (2.0 * np.pi) - np.pi)


def angle_diff(a, b):
    """Différence angulaire a-b ramenée dans ]-pi, pi]."""
    return wrap_angle(np.asarray(a) - np.asarray(b))


def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def make_transform(R: np.ndarray, p) -> np.ndarray:
    """Matrice homogène 4x4 à partir d'une rotation 3x3 et d'une translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse d'une transformation rigide (plus stable que np.linalg.inv)."""
    R = T[:3, :3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R: np.ndarray):
    """Retourne (roll, pitch, yaw). Gère le blocage de cardan (pitch = ±90°)."""
    sy = -R[2, 0]
    pitch = float(np.arcsin(np.clip(sy, -1.0, 1.0)))
    if abs(sy) < 1.0 - 1e-9:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    else:  # cardan : on fixe le yaw à 0
        roll = float(np.arctan2(-R[1, 2], R[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray):
    """Retourne (x, y, z, w) avec w >= 0."""
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q / np.linalg.norm(q))


def rotation_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Vecteur rotation (axe · angle) d'une matrice de rotation, robuste près de 0 et de pi."""
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-9:
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if np.pi - angle < 1e-6:  # proche de pi : l'axe vient de la partie symétrique
        B = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(B)))
        axis = B[:, k] / np.sqrt(max(B[k, k], 1e-12))
        # signe : cohérent avec la partie antisymétrique s'il y en a une
        v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        if np.dot(axis, v) < 0:
            axis = -axis
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))
    return axis * angle


def pose_to_matrix(position, quat_xyzw) -> np.ndarray:
    return make_transform(quat_to_matrix(*quat_xyzw), position)


def matrix_to_pose(T: np.ndarray):
    """Retourne ((x, y, z), (qx, qy, qz, qw))."""
    return tuple(float(v) for v in T[:3, 3]), matrix_to_quat(T[:3, :3])
