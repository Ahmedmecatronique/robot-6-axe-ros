"""Panneau de contrôle : mode sim/réel, sliders articulaires, pose cartésienne (IK), boutons, ARRÊT D'URGENCE."""
from __future__ import annotations

import time
from typing import List

import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets

from robot_kinematics.status_codes import STATE_NAMES, STATE_READY
from robot_kinematics.transforms import make_transform, matrix_to_rpy, rpy_to_matrix

from . import theme


class ControlPanel(QtWidgets.QWidget):
    log = QtCore.Signal(str, str)          # (message, niveau : info | warn | error)

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.cfg = bridge.cfg
        self._last_edit = 0.0              # instant de la dernière action de l'utilisateur sur les curseurs
        self._last_send = 0.0
        self._motion_ok = False
        self._build()

    # =============================================================== construction ===
    def _build(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        # ---- Mode simulation / réel
        box = QtWidgets.QGroupBox('Mode')
        h = QtWidgets.QHBoxLayout(box)
        self.badge = QtWidgets.QLabel('MODE INCONNU')
        self.badge.setAlignment(QtCore.Qt.AlignCenter)
        self.badge.setMinimumWidth(190)
        h.addWidget(self.badge, 1)
        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItems(['Simulation', 'Réel'])
        self.combo_mode.activated.connect(self._on_mode_choice)
        h.addWidget(QtWidgets.QLabel('IHM attend :'))
        h.addWidget(self.combo_mode)
        root.addWidget(box)
        self.lbl_warn = QtWidgets.QLabel('')
        self.lbl_warn.setWordWrap(True)
        root.addWidget(self.lbl_warn)

        # ---- Sliders articulaires
        gj = QtWidgets.QGroupBox('Articulaire')
        gl_ = QtWidgets.QGridLayout(gj)
        self.sliders: List[QtWidgets.QSlider] = []
        self.spins: List[QtWidgets.QDoubleSpinBox] = []
        for i, name in enumerate(self.cfg.joint_names):
            lo, hi = float(np.rad2deg(self.cfg.q_min[i])), float(np.rad2deg(self.cfg.q_max[i]))
            sl = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            sl.setRange(int(lo * 10), int(hi * 10))
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(1)
            sp.setSuffix(' °')
            sp.setSingleStep(1.0)
            sl.valueChanged.connect(lambda v, k=i: self._slider_changed(k, v))
            sl.sliderReleased.connect(self._send_joints)
            sl.sliderPressed.connect(self._touch)
            sp.editingFinished.connect(lambda k=i: self._spin_edited(k))
            gl_.addWidget(QtWidgets.QLabel(f'J{i + 1}'), i, 0)
            gl_.addWidget(QtWidgets.QLabel(f'{lo:.0f}'), i, 1)
            gl_.addWidget(sl, i, 2)
            gl_.addWidget(QtWidgets.QLabel(f'{hi:.0f}'), i, 3)
            gl_.addWidget(sp, i, 4)
            self.sliders.append(sl)
            self.spins.append(sp)
        self.cb_live = QtWidgets.QCheckBox('Envoi continu pendant le glissement (5 Hz)')
        gl_.addWidget(self.cb_live, 6, 0, 1, 5)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('Vitesse'))
        self.sl_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_speed.setRange(5, 100)
        self.sl_speed.setValue(50)
        self.lbl_speed = QtWidgets.QLabel('50 %')
        self.sl_speed.valueChanged.connect(lambda v: self.lbl_speed.setText(f'{v} %'))
        self.sl_speed.sliderReleased.connect(lambda: self.bridge.set_speed_scale(self.sl_speed.value() / 100.0))
        row.addWidget(self.sl_speed, 1)
        row.addWidget(self.lbl_speed)
        gl_.addLayout(row, 7, 0, 1, 5)
        root.addWidget(gj)

        # ---- Cartésien
        gc = QtWidgets.QGroupBox('Cartésien (cinématique inverse)')
        g = QtWidgets.QGridLayout(gc)
        self.cart: List[QtWidgets.QDoubleSpinBox] = []
        specs = [('X', 'mm', -900, 900, 250.0), ('Y', 'mm', -900, 900, 0.0), ('Z', 'mm', 0, 1100, 400.0),
                 ('Roll', '°', -180, 180, 0.0), ('Pitch', '°', -180, 180, 180.0), ('Yaw', '°', -180, 180, 0.0)]
        for k, (lab, unit, lo, hi, val) in enumerate(specs):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(1)
            sp.setSuffix(' ' + unit)
            sp.setValue(val)
            g.addWidget(QtWidgets.QLabel(lab), k // 3 * 2, k % 3)
            g.addWidget(sp, k // 3 * 2 + 1, k % 3)
            self.cart.append(sp)
        b_cur = QtWidgets.QPushButton('Pose actuelle')
        b_prev = QtWidgets.QPushButton('Calculer IK')
        self.b_goto = QtWidgets.QPushButton('Aller à la pose')
        b_cur.clicked.connect(self._fill_current_pose)
        b_prev.clicked.connect(self._preview_ik)
        self.b_goto.clicked.connect(self._goto_pose)
        g.addWidget(b_cur, 4, 0)
        g.addWidget(b_prev, 4, 1)
        g.addWidget(self.b_goto, 4, 2)
        self.lbl_ik = QtWidgets.QLabel('—')
        self.lbl_ik.setWordWrap(True)
        g.addWidget(self.lbl_ik, 5, 0, 1, 3)
        root.addWidget(gc)

        # ---- Boutons
        gb = QtWidgets.QGroupBox('Commandes')
        gg = QtWidgets.QGridLayout(gb)
        self.b_enable = QtWidgets.QPushButton('Activer / Démarrer')
        self.b_disable = QtWidgets.QPushButton('Désactiver / Arrêter')
        self.b_home = QtWidgets.QPushButton('Homing')
        self.b_rest = QtWidgets.QPushButton('Position de repos')
        self.b_soft = QtWidgets.QPushButton('Arrêt doux')
        self.b_open = QtWidgets.QPushButton('Pince : ouvrir')
        self.b_close = QtWidgets.QPushButton('Pince : fermer')
        self.b_enable.clicked.connect(lambda: self.bridge.call_enable(True))
        self.b_disable.clicked.connect(lambda: self.bridge.call_enable(False))
        self.b_home.clicked.connect(self.bridge.call_home)
        self.b_rest.clicked.connect(self.bridge.go_rest)
        self.b_soft.clicked.connect(self.bridge.soft_stop)
        self.b_open.clicked.connect(lambda: self.bridge.send_gripper(1.0))
        self.b_close.clicked.connect(lambda: self.bridge.send_gripper(0.0))
        for n, b in enumerate((self.b_enable, self.b_disable, self.b_home, self.b_rest, self.b_soft,
                               self.b_open, self.b_close)):
            gg.addWidget(b, n // 2, n % 2)
        root.addWidget(gb)
        self._motion_widgets = [self.b_home, self.b_enable, self.b_disable]
        self._move_widgets = [self.b_rest, self.b_soft, self.b_goto, self.b_open, self.b_close,
                              *self.sliders, *self.spins]

        # ---- ARRÊT D'URGENCE
        self.b_estop = QtWidgets.QPushButton('⛔  ARRÊT D\'URGENCE')
        self.b_estop.setStyleSheet(theme.ESTOP_STYLE)
        self.b_estop.setMinimumHeight(70)
        self.b_estop.clicked.connect(self._estop)
        # b_estop et lbl_state sont placés par MainWindow SOUS la zone défilante : l'arrêt d'urgence
        # doit rester visible en permanence, quelle que soit la position du défilement.
        self.lbl_state = QtWidgets.QLabel('')
        self.lbl_state.setWordWrap(True)
        root.addStretch(1)

    # ================================================================ actions ========
    def _on_mode_choice(self, _idx: int):
        if self.combo_mode.currentText() == 'Réel':
            r = QtWidgets.QMessageBox.warning(
                self, 'Mode RÉEL', 'Les commandes vont piloter le VRAI robot.\n'
                'Dégagez la zone de travail et gardez l\'arrêt d\'urgence physique à portée de main.\n\nContinuer ?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if r != QtWidgets.QMessageBox.Yes:
                self.combo_mode.setCurrentText('Simulation')

    def _estop(self):
        self.bridge.send_estop()
        self.log.emit('ARRÊT D\'URGENCE envoyé', 'error')

    def _touch(self):
        self._last_edit = time.time()

    def _slider_changed(self, k: int, v: int):
        self.spins[k].blockSignals(True)
        self.spins[k].setValue(v / 10.0)
        self.spins[k].blockSignals(False)
        self._touch()
        if self.sliders[k].isSliderDown() and self.cb_live.isChecked() and time.time() - self._last_send > 0.2:
            self._send_joints()

    def _spin_edited(self, k: int):
        self.sliders[k].blockSignals(True)
        self.sliders[k].setValue(int(round(self.spins[k].value() * 10)))
        self.sliders[k].blockSignals(False)
        self._touch()
        self._send_joints()

    def _send_joints(self):
        if not self._motion_ok:
            return
        self._last_send = time.time()
        self._touch()
        q = np.deg2rad([sp.value() for sp in self.spins])
        self.bridge.send_joint_goal([q])

    def _pose_matrix(self) -> np.ndarray:
        x, y, z, r, p, yw = (sp.value() for sp in self.cart)
        return make_transform(rpy_to_matrix(*np.deg2rad([r, p, yw])), np.array([x, y, z]) / 1000.0)

    def _fill_current_pose(self):
        T = self.bridge.snapshot()['ee_T']
        if T is None:
            self.log.emit('Pas de /end_effector_pose', 'warn')
            return
        r, p, y = np.rad2deg(matrix_to_rpy(T[:3, :3]))
        for sp, val in zip(self.cart, [*(T[:3, 3] * 1000.0), r, p, y]):
            sp.setValue(float(val))

    def _preview_ik(self):
        self.lbl_ik.setText('Calcul…')
        self.bridge.call_solve_ik(self._pose_matrix())

    def _goto_pose(self):
        if not self._motion_ok:
            return
        self.lbl_ik.setText('Envoi de la pose à ik_node…')
        self.bridge.send_target_pose(self._pose_matrix())

    # ================================================================ événements =====
    def on_event(self, ev: tuple):
        kind = ev[0]
        if kind == 'ik_preview':
            r = ev[1]
            if r is None:
                self.lbl_ik.setText('Pas de réponse du service /solve_ik')
                return
            self._show_ik(bool(r.get('reachable')), bool(r.get('near_singularity')), r.get('message', ''))
        elif kind == 'ik_result':
            d = ev[1]
            self._show_ik(bool(d.get('success')), bool(d.get('near_singularity')), d.get('message', ''))
            if d.get('success') and d.get('q'):
                for sp, sl, v in zip(self.spins, self.sliders, np.rad2deg(d['q'])):
                    sl.blockSignals(True)
                    sl.setValue(int(round(float(v) * 10)))
                    sl.blockSignals(False)
                    sp.blockSignals(True)
                    sp.setValue(float(v))
                    sp.blockSignals(False)
                self._touch()

    def _show_ik(self, ok: bool, singular: bool, msg: str):
        if not ok:
            self.lbl_ik.setStyleSheet(f'color: {theme.ERR};')
            self.lbl_ik.setText('✖ ' + msg)
        elif singular:
            self.lbl_ik.setStyleSheet(f'color: {theme.WARN};')
            self.lbl_ik.setText('⚠ ' + msg)
        else:
            self.lbl_ik.setStyleSheet(f'color: {theme.OK};')
            self.lbl_ik.setText('✔ ' + msg)

    # ============================================================ rafraîchissement ===
    def update_state(self, snap: dict):
        # -- mode affiché / attendu
        actual = snap['mode']
        expected = 'simulation' if self.combo_mode.currentText() == 'Simulation' else 'real'
        if actual == 'simulation':
            self.badge.setText('SIMULATION')
            self.badge.setStyleSheet(theme.badge('#04260f', theme.OK))
        elif actual == 'real':
            self.badge.setText('RÉEL — MOTEURS ACTIFS')
            self.badge.setStyleSheet(theme.badge('white', '#c62828'))
        else:
            self.badge.setText('MODE INCONNU')
            self.badge.setStyleSheet(theme.badge(theme.TEXT, theme.BORDER))
        mismatch = actual in ('simulation', 'real') and actual != expected
        if actual == 'inconnu':
            self.lbl_warn.setText('Aucun /robot_mode reçu (safety_node lancé ? même ROS_DOMAIN_ID ?). '
                                  'Commandes bloquées.')
            self.lbl_warn.setStyleSheet(f'color: {theme.WARN};')
        elif mismatch:
            self.lbl_warn.setText(f'⚠ L\'IHM attend « {self.combo_mode.currentText()} » mais le système est en '
                                  f'« {actual} ». Commandes bloquées : choisissez le bon mode.')
            self.lbl_warn.setStyleSheet(f'color: {theme.ERR};')
        else:
            self.lbl_warn.setText('')
        replay = snap['source'] == 'replay'
        self._motion_ok = (actual == expected) and not replay

        # -- état du robot -> quels boutons sont utilisables
        state = snap['state']
        ready = state == STATE_READY
        for w in self._motion_widgets:
            w.setEnabled(self._motion_ok)
        for w in self._move_widgets:
            w.setEnabled(self._motion_ok and ready)
        if replay:
            self.lbl_state.setText('Mode REJEU : les commandes sont désactivées.')
        elif state < 0:
            self.lbl_state.setText('État du robot inconnu (pas de /stm32/status).')
        elif not ready:
            self.lbl_state.setText(f'Robot « {STATE_NAMES.get(state, state)} » : '
                                   'activez puis lancez le Homing pour pouvoir bouger.')
        else:
            self.lbl_state.setText('Robot PRÊT.')

        # -- les curseurs suivent la position mesurée tant que l'utilisateur ne les manipule pas
        q = snap['q_meas']
        idle = time.time() - self._last_edit > 1.5 and not snap['traj_active']
        if q is not None and idle:
            for sl, sp, v in zip(self.sliders, self.spins, np.rad2deg(q)):
                if sl.isSliderDown() or sp.hasFocus():
                    continue
                sl.blockSignals(True)
                sl.setValue(int(round(float(v) * 10)))
                sl.blockSignals(False)
                sp.blockSignals(True)
                sp.setValue(float(v))
                sp.blockSignals(False)
