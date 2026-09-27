"""Le mode démo (robot simulé localement) : machine d'états, mouvements, arrêt d'urgence, homing, IK, saisie."""
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
import pytest

pytest.importorskip('cv2')

from robot_gui.demo_bridge import DemoBridge  # noqa: E402
from robot_kinematics.status_codes import STATE_ESTOP, STATE_IDLE, STATE_READY  # noqa: E402
from robot_kinematics.transforms import make_transform, rot_x  # noqa: E402


def _wait(pred, timeout=8.0):
    t = time.time()
    while time.time() - t < timeout:
        if pred():
            return True
        time.sleep(0.03)
    return False


def _events(b, sink):
    sink.extend(b.drain_events())
    return sink


@pytest.fixture()
def demo():
    b = DemoBridge(start_ready=True)
    assert _wait(lambda: b.snapshot()['q_meas'] is not None and b.snapshot()['state'] == STATE_READY)
    yield b
    b.shutdown()


def test_goal_is_reached_with_limits_and_ghost_leads(demo):
    b = demo
    b.set_speed_scale(1.0)
    goal = np.deg2rad([40, 30, -60, 50, 40, 20])
    b.send_joint_goal([goal])
    assert _wait(lambda: b.snapshot()['traj_active'] or np.max(np.abs(b.snapshot()['q_cmd'] - goal)) < 0.5)
    assert _wait(lambda: np.max(np.abs(b.snapshot()['q_meas'] - goal)) < 0.02, timeout=30)
    snap = b.snapshot()
    assert snap['status'][0] == STATE_READY
    assert np.all(snap['q_meas'] >= b.cfg.q_min - 1e-9) and np.all(snap['q_meas'] <= b.cfg.q_max + 1e-9)


def test_estop_is_latched_and_blocks_motion_until_enable(demo):
    b = demo
    b.send_estop()
    assert _wait(lambda: b.snapshot()['state'] == STATE_ESTOP)
    b.send_joint_goal([np.deg2rad([10, 10, 10, 10, 10, 10])])           # refusé : robot non prêt
    time.sleep(0.3)
    assert np.max(np.abs(b.snapshot()['q_meas'] - b.cfg.rest)) < 1e-6
    b.call_enable(True)
    assert _wait(lambda: b.snapshot()['state'] == STATE_READY)


def test_cartesian_ik_reachable_unreachable_and_singular(demo):
    b = demo
    ev = []
    b.call_solve_ik(make_transform(rot_x(np.pi), [0.30, 0.10, 0.30]))
    assert _wait(lambda: any(e[0] == 'ik_preview' and e[1]['reachable'] for e in _events(b, ev)))
    ev.clear()
    b.call_solve_ik(make_transform(rot_x(np.pi), [0.95, 0.0, 0.30]))
    assert _wait(lambda: any(e[0] == 'ik_preview' and not e[1]['reachable'] and 'inatteignable' in e[1]['message']
                             for e in _events(b, ev)))
    # « Aller à la pose » : /target_pose -> IK -> but -> le robot y va
    b.send_target_pose(make_transform(rot_x(np.pi), [0.30, 0.10, 0.30]))
    assert _wait(lambda: np.linalg.norm(b.snapshot()['ee_T'][:3, 3] - [0.30, 0.10, 0.30]) < 0.005, timeout=40)


def test_unhomed_flow_enable_then_home(demo):
    b = DemoBridge(start_ready=False)
    try:
        assert _wait(lambda: b.snapshot()['state'] == 1)                # DÉSACTIVÉ
        b.call_home()                                                    # refusé : moteurs désactivés
        ev = []
        assert _wait(lambda: any(e[0] == 'service' and e[1] == 'homing' and not e[2] for e in _events(b, ev)))
        b.call_enable(True)
        assert _wait(lambda: b.snapshot()['state'] == STATE_IDLE)
        b.call_home()
        assert _wait(lambda: b.snapshot()['state'] == 3)                # HOMING
        assert _wait(lambda: b.snapshot()['state'] == STATE_READY, timeout=15)
        assert b.snapshot()['status'][1] == 0b111111                     # 6 axes référencés
    finally:
        b.shutdown()


def test_pick_sequence_reports_progress_and_success(demo):
    b = demo
    b.set_speed_scale(1.0)
    b.pick_detected()
    ev = []
    assert _wait(lambda: any(e[0] == 'pick_done' for e in _events(b, ev)), timeout=120)
    phases = [e[1] for e in ev if e[0] == 'pick_feedback']
    assert 'approche' in phases and 'descente' in phases and 'remontée' in phases
    done = [e for e in ev if e[0] == 'pick_done'][-1]
    assert done[1] is True, done


def test_window_runs_on_demo_bridge():
    from pyqtgraph.Qt import QtWidgets
    pytest.importorskip('PySide6')
    from robot_gui.main_window import MainWindow
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    b = DemoBridge()
    try:
        win = MainWindow(b)
        win.show()
        t = time.time()
        while time.time() - t < 1.5:
            win._tick()
            app.processEvents()
            time.sleep(0.016)
        assert win.panel.badge.text() == 'SIMULATION'
        assert win.dashboard.table.item(0, 2).text() != '—'
        assert win.camera.video.pixmap() is not None                     # image de la caméra factice affichée
        win.close()
    finally:
        b.shutdown()
