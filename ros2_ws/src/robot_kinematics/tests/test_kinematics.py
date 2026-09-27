"""Tests unitaires de la cinématique : FK(IK(pose)) == pose, configurations multiples, singularités."""
import numpy as np
import pytest

from robot_kinematics.config import load_robot_config
from robot_kinematics.kinematics import ArmKinematics
from robot_kinematics.transforms import (angle_diff, matrix_to_quat, matrix_to_rpy, quat_to_matrix,
                                         rotation_to_rotvec, rpy_to_matrix, rot_x, rot_z)

TOL_POS = 1e-6   # m
TOL_ORI = 1e-6   # rad


@pytest.fixture(scope='module')
def arm():
    return ArmKinematics(load_robot_config())


def random_regular_q(arm, rng):
    """Configuration aléatoire dans les limites, loin des singularités."""
    while True:
        q = rng.uniform(arm.q_min, arm.q_max)
        info = arm.singularity_info(q)
        if not info.near:
            return q


def test_fk_zero_pose(arm):
    """À q = 0 le bras est vertical : le centre du poignet est en (0, 0, d1 + a2 + a3)."""
    frames = arm.forward_frames(np.zeros(6))
    d1, a2, a3, d6 = arm.d[0], arm.a[1], arm.a[2], arm.d[5]
    np.testing.assert_allclose(frames[3][:3, 3], [0, 0, d1 + a2 + a3], atol=1e-12)
    # l'outil s'écarte du centre du poignet de d6 le long de z6
    T = frames[6]
    np.testing.assert_allclose(T[:3, 3], frames[3][:3, 3] + d6 * T[:3, 2], atol=1e-12)
    # matrice homogène valide
    np.testing.assert_allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(T[:3, :3]) - 1.0) < 1e-12


def test_jacobian_matches_finite_differences(arm):
    rng = np.random.default_rng(1)
    q = random_regular_q(arm, rng)
    J = arm.jacobian(q)
    eps = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        T1, T0 = arm.forward(q + dq), arm.forward(q - dq)
        dp = (T1[:3, 3] - T0[:3, 3]) / (2 * eps)
        dw = rotation_to_rotvec(T1[:3, :3] @ T0[:3, :3].T) / (2 * eps)
        np.testing.assert_allclose(J[:3, i], dp, atol=1e-6)
        np.testing.assert_allclose(J[3:, i], dw, atol=1e-6)


def test_fk_ik_roundtrip_analytic(arm):
    """FK(IK(pose)) == pose pour TOUTES les solutions analytiques, sur 200 poses aléatoires."""
    rng = np.random.default_rng(42)
    for _ in range(200):
        q = random_regular_q(arm, rng)
        T = arm.forward(q)
        sols = arm.ik_analytic_all(T, q)
        assert len(sols) >= 4, 'au moins les configurations face/coude/poignet doivent exister'
        for qs, label in sols:
            pe, oe = arm.fk_error(T, qs)
            assert pe < TOL_POS and oe < TOL_ORI, f'{label}: {pe}, {oe}'
        # la configuration d'origine doit faire partie des solutions
        dists = [np.max(np.abs(angle_diff(qs, q))) for qs, _ in sols]
        assert min(dists) < 1e-6


def test_solve_picks_closest_to_current_state(arm):
    rng = np.random.default_rng(7)
    for _ in range(100):
        q = random_regular_q(arm, rng)
        T = arm.forward(q)
        q_ref = q + rng.uniform(-0.05, 0.05, 6)
        res = arm.solve(T, q_ref, allow_numeric=False)
        assert res.success and res.method == 'analytical'
        np.testing.assert_allclose(angle_diff(res.q, q), 0.0, atol=1e-6)


def test_multiple_configurations_are_distinct(arm):
    q = np.deg2rad([20, 30, 50, 40, 60, -30])
    sols = arm.ik_analytic_all(arm.forward(q), q)
    labels = {label for _, label in sols}
    assert len(sols) == 8 and len(labels) == 8   # face/dos x coude+/- x poignet+/-


def test_unreachable_pose_reports_reason(arm):
    T = np.eye(4)
    T[:3, 3] = [2.0, 0.0, 0.5]
    res = arm.solve(T, allow_numeric=False)
    assert not res.success
    assert 'inatteignable' in res.message


def test_pose_reachable_but_out_of_limits(arm):
    """Un coude à 160° est atteignable géométriquement mais dépasse la limite de 150°."""
    q = np.deg2rad([0, -40, 160, 0, 45, 0])
    T = arm.forward(q)
    # on ne garde que les solutions analytiques hors limites : la pose ne doit alors pas être "valide"
    res = arm.solve(T, q, allow_numeric=False)
    if res.success:  # une autre configuration (ex. dos/coude-) peut rester dans les limites
        assert arm.within_limits(res.q)[0]


def test_numeric_ik_converges_from_perturbed_start(arm):
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = random_regular_q(arm, rng)
        T = arm.forward(q)
        q_num, ok = arm.ik_numeric(T, q + rng.uniform(-0.3, 0.3, 6))
        assert ok
        pe, oe = arm.fk_error(T, q_num)
        assert pe < 1e-4 and oe < 1e-3


def test_wrist_singularity_detected_and_solved(arm):
    q = np.deg2rad([10, 20, 40, 30, 0.0, 15])   # q5 = 0 : poignet singulier
    info = arm.singularity_info(q)
    assert info.near and any('poignet' in k for k in info.kinds)
    T = arm.forward(q)
    res = arm.solve(T, q)
    assert res.success
    pe, oe = arm.fk_error(T, res.q)
    assert pe < 1e-4 and oe < 1e-3
    assert res.near_singularity


def test_elbow_singularity_detected(arm):
    q = np.deg2rad([0, 30, 0.0, 20, 50, 10])    # bras tendu
    info = arm.singularity_info(q)
    assert info.near and any('coude' in k for k in info.kinds)


def test_regular_configuration_not_singular(arm):
    q = np.deg2rad([20, 30, 60, 40, 70, -20])
    assert not arm.singularity_info(q).near


def test_joint_limits(arm):
    ok, bad = arm.within_limits(arm.cfg.rest)
    assert ok and bad == []
    q = arm.q_max.copy()
    q[2] += 0.01
    ok, bad = arm.within_limits(q)
    assert not ok and bad == [2]


def test_link_frames_match_dh_frames(arm):
    """Le repère U_6 (URDF) coïncide avec le repère DH 6 car a6 = alpha6 = 0."""
    q = np.deg2rad([10, 20, 30, 40, 50, 60])
    np.testing.assert_allclose(arm.link_frames(q)[5], arm.forward(q), atol=1e-12)


def test_transform_helpers_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        r, p, y = rng.uniform(-3.0, 3.0, 3)
        p = np.clip(p, -1.4, 1.4)
        R = rpy_to_matrix(r, p, y)
        np.testing.assert_allclose(rpy_to_matrix(*matrix_to_rpy(R)), R, atol=1e-12)
        np.testing.assert_allclose(quat_to_matrix(*matrix_to_quat(R)), R, atol=1e-12)
    axis_rot = rot_z(0.3) @ rot_x(-1.1)
    rv = rotation_to_rotvec(axis_rot)
    assert abs(np.linalg.norm(rv) - np.arccos((np.trace(axis_rot) - 1) / 2)) < 1e-12
