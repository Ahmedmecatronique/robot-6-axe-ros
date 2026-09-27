"""Tests des profils de trajectoire : bornes de vitesse/accélération, continuité, synchronisation."""
import numpy as np
import pytest

from robot_kinematics.trajectory import plan_segment

V_MAX = np.deg2rad([30, 30, 30, 45, 45, 90.0])
A_MAX = np.deg2rad([60, 60, 60, 90, 90, 180.0])


def _check_limits(seg, tol=1.02):
    for t in np.linspace(0, seg.duration, 400):
        _, v, a = seg.sample(t)
        assert np.all(np.abs(v) <= V_MAX * tol), (t, v)
        assert np.all(np.abs(a) <= A_MAX * tol), (t, a)


@pytest.mark.parametrize('profile', ['quintic', 'trapezoid'])
def test_rest_to_rest_endpoints_and_limits(profile):
    q0 = np.zeros(6)
    q1 = np.deg2rad([60, -40, 90, 120, -30, 200])
    seg = plan_segment(q0, q1, V_MAX, A_MAX, profile=profile)
    q_start, v_start, _ = seg.sample(0.0)
    q_end, v_end, a_end = seg.sample(seg.duration)
    np.testing.assert_allclose(q_start, q0, atol=1e-9)
    np.testing.assert_allclose(v_start, 0.0, atol=1e-9)
    np.testing.assert_allclose(q_end, q1, atol=1e-9)
    np.testing.assert_allclose(v_end, 0.0, atol=1e-9)
    np.testing.assert_allclose(a_end, 0.0, atol=1e-9)
    _check_limits(seg)


@pytest.mark.parametrize('profile', ['quintic', 'trapezoid'])
def test_axes_are_synchronised(profile):
    """Tous les axes arrivent en même temps : le rapport parcouru/total est identique à chaque instant."""
    q0 = np.zeros(6)
    q1 = np.deg2rad([90, 45, -30, 10, 0, 60])
    seg = plan_segment(q0, q1, V_MAX, A_MAX, profile=profile)
    moving = np.abs(q1 - q0) > 1e-9
    for t in np.linspace(0, seg.duration, 25):
        q, _, _ = seg.sample(t)
        frac = ((q - q0)[moving] / (q1 - q0)[moving])
        assert np.ptp(frac) < 1e-9


def test_quintic_with_initial_velocity_is_continuous_and_bounded():
    q0 = np.zeros(6)
    v0 = np.deg2rad([10, -5, 0, 0, 0, 0])
    q1 = np.deg2rad([40, 20, 10, 0, 0, 0])
    seg = plan_segment(q0, q1, V_MAX, A_MAX, v0=v0, profile='quintic')
    _, v_start, _ = seg.sample(0.0)
    np.testing.assert_allclose(v_start, v0, atol=1e-9)
    _check_limits(seg, tol=1.05)


def test_no_motion_returns_none():
    q = np.deg2rad([10, 20, 30, 0, 0, 0])
    assert plan_segment(q, q, V_MAX, A_MAX) is None


def test_min_duration_is_respected():
    seg = plan_segment(np.zeros(6), np.deg2rad([1, 0, 0, 0, 0, 0]), V_MAX, A_MAX, min_duration=2.0)
    assert seg.duration >= 2.0 - 1e-9
