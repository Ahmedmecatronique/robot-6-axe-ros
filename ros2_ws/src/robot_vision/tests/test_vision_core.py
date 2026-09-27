"""Tests de la géométrie caméra->base et de la détection (ArUco, HSV), sur images synthétiques."""
import cv2
import numpy as np
import pytest

from robot_vision import vision_core as vc
from robot_kinematics.transforms import make_transform, rot_x, rot_z

K = np.array([[493.0, 0.0, 320.0], [0.0, 493.0, 240.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)
# caméra à (0.45, 0, 0.70), regardant vers le bas
T_BASE_CAM = make_transform(rot_x(np.pi), [0.45, 0.0, 0.70])


def test_pixel_to_plane_principal_point_hits_below_camera():
    p = vc.pixel_to_plane(320.0, 240.0, K, DIST, T_BASE_CAM, plane_z=0.0)
    np.testing.assert_allclose(p, [0.45, 0.0, 0.0], atol=1e-9)


def test_projection_backprojection_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        p = np.array([rng.uniform(0.25, 0.65), rng.uniform(-0.2, 0.2), 0.05])
        u, v = vc.project_point(p, K, DIST, T_BASE_CAM)
        q = vc.pixel_to_plane(u, v, K, DIST, T_BASE_CAM, plane_z=0.05)
        np.testing.assert_allclose(q, p, atol=1e-6)


def test_top_down_rotation_points_tool_down():
    for yaw in np.linspace(-3.0, 3.0, 13):
        R = vc.top_down_grasp_rotation(yaw)
        np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert abs(np.linalg.det(R) - 1.0) < 1e-12
        assert abs(np.arctan2(R[1, 0], R[0, 0])) <= np.pi / 2 + 1e-9   # yaw ramené dans ]-90°, 90°]


def _render_marker(marker_id=7, size=200):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        m = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        m = cv2.aruco.drawMarker(dictionary, marker_id, size)
    canvas = np.full((480, 640), 255, np.uint8)
    canvas[140:140 + size, 220:220 + size] = m
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_aruco_detection_finds_id():
    dets = vc.detect_aruco(_render_marker(7), 'DICT_4X4_50')
    assert [d.marker_id for d in dets] == [7]
    assert dets[0].corners.shape == (4, 2)


def test_marker_pose_recovered_from_projected_corners():
    """On projette un marqueur de pose connue, puis on retrouve cette pose par solvePnP."""
    L = 0.04
    R_true = rot_z(0.6) @ rot_x(np.pi)                    # marqueur à plat, vu par le dessus
    T_base_marker = make_transform(R_true, [0.50, 0.05, 0.02])
    T_cam_marker = np.linalg.inv(T_BASE_CAM) @ T_base_marker
    h = L / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T_cam_marker[:3, :3])
    px, _ = cv2.projectPoints(obj, rvec, T_cam_marker[:3, 3], K, DIST)
    rvec_est, tvec_est = vc.marker_pose_camera(px.reshape(4, 2), L, K, DIST)
    T_est = vc.marker_pose_base(rvec_est, tvec_est, T_BASE_CAM)
    np.testing.assert_allclose(T_est[:3, 3], T_base_marker[:3, 3], atol=1e-4)
    np.testing.assert_allclose(T_est[:3, :3], T_base_marker[:3, :3], atol=1e-3)


def test_hsv_detection_and_pose():
    img = np.full((480, 640, 3), 40, np.uint8)
    cv2.rectangle(img, (300, 200), (360, 260), (0, 0, 255), -1)          # carré rouge (BGR)
    det = vc.detect_hsv(img, lower=(170, 120, 70), upper=(10, 255, 255))   # rouge : H enjambe 0
    assert det is not None
    assert abs(det.center[0] - 330) < 3 and abs(det.center[1] - 230) < 3
    T = vc.color_pose_base(det, K, DIST, T_BASE_CAM, plane_z=0.0)
    np.testing.assert_allclose(T[:3, 2], [0, 0, -1], atol=1e-9)
    # le centre du carré est décalé de (+30, -10) px du point principal -> (x = 0.45 - ..., y ...)
    expected = vc.pixel_to_plane(330.0, 230.0, K, DIST, T_BASE_CAM, 0.0)
    np.testing.assert_allclose(T[:3, 3], expected, atol=2e-3)


def test_hsv_no_detection_returns_none():
    img = np.full((480, 640, 3), 40, np.uint8)
    assert vc.detect_hsv(img, (170, 120, 70), (10, 255, 255)) is None


def test_image_to_bgr_handles_rgb_and_padding():
    h, w = 4, 5
    rgb = np.zeros((h, w, 3), np.uint8)
    rgb[:, :, 0] = 255                                    # rouge en RGB
    padded = np.zeros((h, w * 3 + 3), np.uint8)           # step > width*3
    padded[:, :w * 3] = rgb.reshape(h, w * 3)
    bgr = vc.image_to_bgr(padded.tobytes(), h, w, w * 3 + 3, 'rgb8')
    assert bgr.shape == (h, w, 3)
    assert tuple(bgr[0, 0]) == (0, 0, 255)


def test_hand_eye_eye_to_hand_recovers_camera_pose():
    """Données synthétiques : on impose T_base_cam et T_outil_marqueur, on retrouve T_base_cam."""
    rng = np.random.default_rng(5)
    T_base_cam_true = make_transform(rot_x(np.pi) @ rot_z(0.3), [0.45, 0.10, 0.70])
    T_tool_marker = make_transform(rot_x(0.2) @ rot_z(-0.4), [0.01, -0.02, 0.05])
    samples = []
    for _ in range(15):
        # poses d'outil variées (positions ET orientations)
        R = rot_z(rng.uniform(-1.5, 1.5)) @ rot_x(rng.uniform(2.2, 4.0)) @ rot_z(rng.uniform(-1.0, 1.0))
        T_bt = make_transform(R, [rng.uniform(0.3, 0.6), rng.uniform(-0.2, 0.2), rng.uniform(0.15, 0.35)])
        T_cm = np.linalg.inv(T_base_cam_true) @ T_bt @ T_tool_marker
        samples.append((T_bt, T_cm))
    T_est, t_mm, r_deg = vc.hand_eye_eye_to_hand(samples)
    np.testing.assert_allclose(T_est[:3, 3], T_base_cam_true[:3, 3], atol=1e-3)
    np.testing.assert_allclose(T_est[:3, :3], T_base_cam_true[:3, :3], atol=1e-3)
    assert t_mm < 1.0 and r_deg < 0.5


def test_pose_smoother_rejects_outliers_with_median():
    sm = vc.PoseSmoother(5)
    base = make_transform(rot_x(np.pi), [0.5, 0.0, 0.02])
    for i in range(4):
        sm.update(base)
    outlier = base.copy()
    outlier[:3, 3] = [0.9, 0.4, 0.02]
    out = sm.update(outlier)
    np.testing.assert_allclose(out[:3, 3], [0.5, 0.0, 0.02], atol=1e-9)
