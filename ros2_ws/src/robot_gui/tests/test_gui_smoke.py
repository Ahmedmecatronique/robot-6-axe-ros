"""Test de fumée de l'IHM SANS ROS : un faux pont simule un robot ; Qt tourne hors écran.

    QT_QPA_PLATFORM=offscreen python -m pytest tests -q

Vérifie que la fenêtre se construit, se rafraîchit sans exception, et que la logique de sécurité
de l'interface (blocage des commandes si le mode ne correspond pas, ARRÊT D'URGENCE) fonctionne.
"""
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import cv2
import numpy as np
import pytest

pytest.importorskip('PySide6')
pytest.importorskip('pyqtgraph')

from pyqtgraph.Qt import QtWidgets  # noqa: E402

from robot_kinematics.config import load_robot_config  # noqa: E402
from robot_kinematics.kinematics import ArmKinematics  # noqa: E402


class FakeBridge:
    """Même interface que RosBridge, données synthétiques."""

    def __init__(self, mode='simulation', state=4):
        self.cfg = load_robot_config()
        self.arm = ArmKinematics(self.cfg)
        self.mode, self.state = mode, state
        self.t0 = time.time()
        self.events, self.goals, self.targets, self.calls = [], [], [], []
        self.estops = 0
        self.source = 'live'
        ok, jpg = cv2.imencode('.jpg', np.full((240, 320, 3), 90, np.uint8))
        self._jpg = jpg.tobytes()

    def _q(self, phase=0.0):
        t = time.time() - self.t0
        return 0.4 * np.sin(0.7 * t + phase + np.arange(6)) * np.array([1, 0.8, 0.8, 1, 0.6, 1])

    def snapshot(self):
        q = self._q()
        c = self._q(0.05)
        return {
            'q_meas': q, 'v_meas': np.zeros(6), 'effort': np.zeros(6), 'q_cmd': c, 'v_cmd': np.zeros(6),
            'meas_age': 0.01, 'cmd_age': 0.01, 'js_rate': 99.5, 'latency_ms': 7.0,
            'ee_T': self.arm.forward(q), 'target_T': self.arm.forward(self._q(1.0)), 'target_age': 0.1,
            'target_visible': True, 'state': self.state,
            'status': [self.state, 63, 0, 0, 12, 1000, 8, 120], 'status_age': 0.05,
            'mode': self.mode, 'diagnostics': [{'name': 'arm6/test', 'level': 0, 'message': 'ok',
                                                'values': {'a': '1'}}],
            'traj_active': False, 'source': self.source, 'image_seq': int(time.time() * 15),
        }

    def history(self, seconds=10.0):
        t = np.linspace(-seconds, 0, 200)
        q = np.array([0.4 * np.sin(0.7 * tt + np.arange(6)) for tt in t])
        return t, q, q + 0.01

    def image_jpeg(self):
        return int(time.time() * 15), self._jpg

    def drain_events(self):
        ev, self.events = self.events, []
        return ev

    # commandes
    def send_joint_goal(self, wps): self.goals.append([np.array(w) for w in wps])
    def send_target_pose(self, T): self.targets.append(T)
    def send_estop(self): self.estops += 1
    def soft_stop(self): self.calls.append('soft_stop')
    def go_rest(self): self.calls.append('rest')
    def send_gripper(self, x): self.calls.append(('gripper', x))
    def set_speed_scale(self, k): self.calls.append(('speed', k))
    def call_enable(self, e): self.calls.append(('enable', e))
    def call_home(self): self.calls.append('home')
    def call_solve_ik(self, T): self.calls.append('ik')
    def pick_detected(self): self.calls.append('pick')
    def cancel_pick(self): self.calls.append('cancel_pick')
    def set_source(self, s): self.source = s
    def bag_record_start(self): return True, '/tmp/x'
    def bag_record_stop(self): pass
    def bag_recording(self): return False
    def list_bags(self): return ['session_a']
    def bag_replay_start(self, n): self.source = 'replay'; return True, n
    def bag_replay_stop(self): self.source = 'live'
    def bag_playing(self): return False
    def bag_refresh(self): pass
    def replay_finished(self): return False
    def shutdown(self): pass


@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _pump(app, seconds=0.2):
    t = time.time()
    while time.time() - t < seconds:
        app.processEvents()
        time.sleep(0.005)


def test_main_window_builds_and_refreshes(app):
    from robot_gui.main_window import MainWindow
    bridge = FakeBridge()
    win = MainWindow(bridge)
    win.show()
    for _ in range(80):                       # ~ 80 rafraîchissements de la boucle 60 Hz
        win._tick()
        app.processEvents()
    assert win.panel.badge.text() == 'SIMULATION'
    assert win.dashboard.table.item(0, 2).text() != '—'
    win.close()


def test_motion_blocked_when_mode_mismatch(app):
    from robot_gui.panels import ControlPanel
    bridge = FakeBridge(mode='real')            # le système est en RÉEL...
    panel = ControlPanel(bridge)                # ...mais l'IHM attend « Simulation » (défaut)
    panel.update_state(bridge.snapshot())
    assert not panel._motion_ok
    assert not panel.b_rest.isEnabled() and not panel.b_home.isEnabled()
    assert panel.b_estop.isEnabled()            # l'arrêt d'urgence reste TOUJOURS disponible
    panel.sliders[0].setValue(200)
    panel._send_joints()
    assert bridge.goals == []                   # rien n'est envoyé au vrai robot


def test_motion_enabled_when_mode_matches_and_ready(app):
    from robot_gui.panels import ControlPanel
    bridge = FakeBridge(mode='simulation', state=4)
    panel = ControlPanel(bridge)
    panel.update_state(bridge.snapshot())
    assert panel._motion_ok and panel.b_rest.isEnabled()
    panel.spins[1].setValue(30.0)
    panel._spin_edited(1)
    assert len(bridge.goals) == 1
    assert abs(np.rad2deg(bridge.goals[0][0][1]) - 30.0) < 0.11


def test_not_ready_disables_movement_but_not_homing(app):
    from robot_gui.panels import ControlPanel
    bridge = FakeBridge(mode='simulation', state=2)      # activé mais non référencé
    panel = ControlPanel(bridge)
    panel.update_state(bridge.snapshot())
    assert panel.b_home.isEnabled() and not panel.b_rest.isEnabled()


def test_estop_button_publishes(app):
    from robot_gui.panels import ControlPanel
    bridge = FakeBridge(mode='real', state=4)
    panel = ControlPanel(bridge)
    panel.b_estop.click()
    assert bridge.estops == 1


def test_cartesian_goto_and_preview(app):
    from robot_gui.panels import ControlPanel
    bridge = FakeBridge(mode='simulation', state=4)
    panel = ControlPanel(bridge)
    panel.update_state(bridge.snapshot())
    panel._preview_ik()
    panel._goto_pose()
    assert 'ik' in bridge.calls and len(bridge.targets) == 1
    T = bridge.targets[0]
    np.testing.assert_allclose(T[:3, 3], [0.25, 0.0, 0.40], atol=1e-9)   # X, Y, Z par défaut (mm -> m)


def test_teach_add_and_play(app):
    from robot_gui.teach_tab import TeachTab
    bridge = FakeBridge()
    tab = TeachTab(bridge)
    tab._add_pose()
    tab._add_pose()
    assert len(tab.steps) == 2 and tab.table.rowCount() == 2
    tab._toggle_sequence()
    assert tab._play_i == 0 and len(bridge.goals) == 1
    tab._toggle_sequence()                                # arrêt
    assert tab._play_i == -1


def test_camera_tab_pick_needs_fresh_target(app):
    from robot_gui.camera_tab import CameraTab
    bridge = FakeBridge()
    tab = CameraTab(bridge)
    tab.update_view(bridge.snapshot())
    tab._pick()
    assert 'pick' in bridge.calls
