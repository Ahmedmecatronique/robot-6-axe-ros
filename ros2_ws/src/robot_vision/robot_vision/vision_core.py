"""Cœur de la vision (sans ROS, donc testable) : détection ArUco / HSV et géométrie caméra -> base.

Repères :
  * caméra : repère OPTIQUE (x vers la droite de l'image, y vers le bas, z vers l'avant)
  * base   : base_link du robot
  T_base_cam (4x4) = pose de la caméra dans la base, fournie par la calibration main-œil
  (config/hand_eye.yaml).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from robot_kinematics.transforms import (invert_transform, make_transform, pose_to_matrix, rot_x,
                                         rot_z, wrap_angle)


# =============================================================================
#  Fichiers de calibration
# =============================================================================
def load_camera_yaml(path: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int], bool]:
    """Retourne (K 3x3, distorsion, (largeur, hauteur), calibrée?)."""
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix'], dtype=float).reshape(3, 3)
    dist = np.array(d.get('dist_coeffs', [0, 0, 0, 0, 0]), dtype=float).reshape(-1)
    return K, dist, (int(d['image_width']), int(d['image_height'])), bool(d.get('calibrated', False))


def load_hand_eye_yaml(path: str) -> Tuple[np.ndarray, str, str, bool]:
    """Retourne (T_base_cam 4x4, repère parent, repère caméra, calibrée?)."""
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    T = pose_to_matrix(d['translation'], d['quaternion_xyzw'])
    return T, d.get('parent_frame', 'base_link'), d.get('camera_frame', 'camera_optical_frame'), \
        bool(d.get('calibrated', False))


# =============================================================================
#  Conversion d'image ROS -> BGR (sans cv_bridge)
# =============================================================================
def image_to_bgr(data: bytes, height: int, width: int, step: int, encoding: str) -> Optional[np.ndarray]:
    """Convertit les champs d'un sensor_msgs/Image en tableau BGR uint8 (encodages courants)."""
    buf = np.frombuffer(data, dtype=np.uint8)
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(encoding)
    if channels is None:
        return None
    img = buf.reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    if encoding == 'bgr8':
        return img.copy()
    if encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2BGR)


# =============================================================================
#  Géométrie
# =============================================================================
def pixel_to_plane(u: float, v: float, K: np.ndarray, dist: np.ndarray, T_base_cam: np.ndarray,
                   plane_z: float) -> Optional[np.ndarray]:
    """Intersection du rayon optique du pixel (u, v) avec le plan horizontal z = plane_z (base).

    C'est la façon de localiser un objet sans profondeur : on suppose qu'il repose sur un plan connu.
    Retourne None si le rayon est parallèle au plan ou pointe derrière la caméra.
    """
    pt = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), K, dist)[0, 0]
    d_cam = np.array([pt[0], pt[1], 1.0])
    origin = T_base_cam[:3, 3]
    direction = T_base_cam[:3, :3] @ d_cam
    if abs(direction[2]) < 1e-9:
        return None
    s = (plane_z - origin[2]) / direction[2]
    if s <= 0.0:
        return None
    return origin + s * direction


def project_point(p_base: np.ndarray, K: np.ndarray, dist: np.ndarray, T_base_cam: np.ndarray
                  ) -> Tuple[float, float]:
    """Projette un point 3D de la base dans l'image (utile pour les tests et la superposition)."""
    T_cam_base = np.linalg.inv(T_base_cam)
    p_cam = T_cam_base[:3, :3] @ p_base + T_cam_base[:3, 3]
    px, _ = cv2.projectPoints(p_cam.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, dist)
    return float(px[0, 0, 0]), float(px[0, 0, 1])


def top_down_grasp_rotation(yaw: float) -> np.ndarray:
    """Orientation d'outil pour une saisie par le dessus : axe z de l'outil vers le BAS.

    R = Rz(yaw) · Rx(pi). Le yaw est ramené dans ]-90°, 90°] (la pince est symétrique à 180°),
    ce qui évite de faire tourner inutilement le poignet.
    """
    yaw = wrap_angle(yaw)
    if yaw > np.pi / 2:
        yaw -= np.pi
    elif yaw <= -np.pi / 2:
        yaw += np.pi
    return rot_z(yaw) @ rot_x(np.pi)


# =============================================================================
#  ArUco
# =============================================================================
@dataclass
class MarkerDetection:
    marker_id: int
    corners: np.ndarray                # (4, 2) pixels, ordre HG, HD, BD, BG


def _aruco_dictionary(name: str):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_aruco(image_bgr: np.ndarray, dict_name: str = 'DICT_4X4_50') -> List[MarkerDetection]:
    """Détecte les marqueurs ArUco (compatible OpenCV 4.5 et >= 4.7)."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = _aruco_dictionary(dict_name)
    if hasattr(cv2.aruco, 'ArucoDetector'):                # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:                                                  # OpenCV 4.5 (Ubuntu 22.04)
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return []
    return [MarkerDetection(int(i), c.reshape(4, 2)) for i, c in zip(ids.flatten(), corners)]


def marker_pose_camera(corners: np.ndarray, marker_length: float, K: np.ndarray, dist: np.ndarray
                       ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Pose du marqueur dans la caméra (rvec, tvec) par solvePnP (IPPE_SQUARE)."""
    h = marker_length / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float64), K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (rvec, tvec) if ok else None


def marker_pose_base(rvec, tvec, T_base_cam: np.ndarray) -> np.ndarray:
    """T_base_marker = T_base_cam · T_cam_marker."""
    R, _ = cv2.Rodrigues(rvec)
    return T_base_cam @ make_transform(R, np.asarray(tvec).reshape(3))


# =============================================================================
#  Détection par couleur HSV
# =============================================================================
@dataclass
class ColorDetection:
    contour: np.ndarray
    center: Tuple[float, float]        # pixels
    axis_angle: float                  # angle (rad) de l'axe principal dans l'image
    area: float


def detect_hsv(image_bgr: np.ndarray, lower, upper, min_area: float = 300.0) -> Optional[ColorDetection]:
    """Plus grand blob dont la couleur est dans [lower, upper] (HSV OpenCV : H 0-179, S/V 0-255).

    Le rouge « enjambe » H = 0/179 : si lower[0] > upper[0], les deux intervalles sont combinés.
    """
    hsv = cv2.cvtColor(cv2.GaussianBlur(image_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    lower, upper = np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        m1 = cv2.inRange(hsv, lower, np.array([179, upper[1], upper[2]], dtype=np.uint8))
        m2 = cv2.inRange(hsv, np.array([0, lower[1], lower[2]], dtype=np.uint8), upper)
        mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    (cx, cy), (w, h), angle_deg = cv2.minAreaRect(c)
    # axe long du rectangle
    axis = np.deg2rad(angle_deg if w >= h else angle_deg + 90.0)
    return ColorDetection(c, (cx, cy), float(axis), float(cv2.contourArea(c)))


def color_pose_base(det: ColorDetection, K, dist, T_base_cam, plane_z: float) -> Optional[np.ndarray]:
    """Pose 4x4 (saisie par le dessus) d'un objet coloré posé sur le plan z = plane_z."""
    p = pixel_to_plane(det.center[0], det.center[1], K, dist, T_base_cam, plane_z)
    if p is None:
        return None
    # yaw : direction de l'axe principal reprojetée sur le plan
    du, dv = np.cos(det.axis_angle), np.sin(det.axis_angle)
    p2 = pixel_to_plane(det.center[0] + 20 * du, det.center[1] + 20 * dv, K, dist, T_base_cam, plane_z)
    yaw = 0.0 if p2 is None else float(np.arctan2(p2[1] - p[1], p2[0] - p[0]))
    return make_transform(top_down_grasp_rotation(yaw), p)


# =============================================================================
#  Lissage temporel (évite que la cible « tremble »)
# =============================================================================
class PoseSmoother:
    """Médiane glissante de la position + moyenne circulaire du yaw sur N détections."""

    def __init__(self, size: int = 5):
        self.buf: Deque[Tuple[np.ndarray, float]] = deque(maxlen=size)

    def reset(self):
        self.buf.clear()

    def update(self, T: np.ndarray) -> np.ndarray:
        yaw = float(np.arctan2(T[1, 0], T[0, 0]))
        self.buf.append((T[:3, 3].copy(), yaw))
        pos = np.median(np.array([p for p, _ in self.buf]), axis=0)
        yaws = np.array([y for _, y in self.buf])
        yaw_m = float(np.arctan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))))
        return make_transform(top_down_grasp_rotation(yaw_m), pos)

    def __len__(self):
        return len(self.buf)


# =============================================================================
#  Calibration main-œil (eye-to-hand)
# =============================================================================
def average_rotation(rotations: List[np.ndarray]) -> np.ndarray:
    """Moyenne de rotations : moyenne arithmétique puis projection sur SO(3) (SVD)."""
    U, _, Vt = np.linalg.svd(np.mean(rotations, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def hand_eye_eye_to_hand(samples) -> Tuple[np.ndarray, float, float]:
    """Calibration eye-to-hand : caméra FIXE, marqueur rigidement lié à l'outil.

    samples : liste de (T_base_tool, T_cam_marker), 4x4 chacune.
    Équation : T_base_cam · T_cam_marker_i = T_base_tool_i · T_tool_marker (T_tool_marker constante).
    cv2.calibrateHandEye résout AX = XB ; en eye-to-hand on lui fournit la pose base -> outil
    (inverse de T_base_tool), et il retourne alors la pose caméra -> base = T_base_cam.
    Retourne (T_base_cam, dispersion translation en mm, dispersion rotation en degrés) de T_tool_marker.
    """
    R_g2b = [invert_transform(Tbt)[:3, :3] for Tbt, _ in samples]
    t_g2b = [invert_transform(Tbt)[:3, 3].reshape(3, 1) for Tbt, _ in samples]
    R_t2c = [Tcm[:3, :3] for _, Tcm in samples]
    t_t2c = [Tcm[:3, 3].reshape(3, 1) for _, Tcm in samples]
    R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_PARK)
    T_base_cam = make_transform(R, t.reshape(3))
    # Qualité : T_tool_marker doit être la même pour toutes les poses
    T_tm = [invert_transform(Tbt) @ T_base_cam @ Tcm for Tbt, Tcm in samples]
    trans = np.array([m[:3, 3] for m in T_tm])
    R_mean = average_rotation([m[:3, :3] for m in T_tm])
    rot_dev = [np.degrees(np.arccos(np.clip((np.trace(m[:3, :3].T @ R_mean) - 1.0) / 2.0, -1.0, 1.0)))
               for m in T_tm]
    return T_base_cam, float(np.max(np.std(trans, axis=0)) * 1e3), float(np.max(rot_dev))
