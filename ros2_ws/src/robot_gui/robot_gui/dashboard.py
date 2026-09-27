"""Tableau de bord temps réel : valeurs par axe, pose de l'effecteur (FK), courbes consigne/mesure
(pour régler les PID), état de la liaison STM32, watchdog et alarmes."""
from __future__ import annotations

import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from robot_kinematics.status_codes import (FAULT_COMM, ST_CMD_AGE_MS, ST_FAULT_CODE, ST_HOMED_MASK,
                                           ST_LIMIT_MASK, ST_LOOP_HZ, ST_MAX_FERR, STATE_FAULT,
                                           STATE_NAMES, decode_faults)
from robot_kinematics.transforms import matrix_to_rpy

from . import theme

COLUMNS = ['Axe', 'Consigne (°)', 'Mesure (°)', 'Erreur (°)', 'Vitesse (°/s)', 'Effort (cmd)', 'Plage (°)']


class Dashboard(QtWidgets.QWidget):
    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.cfg = bridge.cfg
        self._active_alarms = set()
        self._paused = False
        self._build()

    # =============================================================== construction ===
    def _build(self):
        root = QtWidgets.QHBoxLayout(self)

        # ---------- colonne gauche : table + effecteur + liaison
        left = QtWidgets.QVBoxLayout()
        self.table = QtWidgets.QTableWidget(6, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        for i in range(6):
            lo, hi = np.rad2deg(self.cfg.q_min[i]), np.rad2deg(self.cfg.q_max[i])
            self.table.setItem(i, 0, QtWidgets.QTableWidgetItem(f'J{i + 1}'))
            self.table.setItem(i, 6, QtWidgets.QTableWidgetItem(f'{lo:.0f} … {hi:.0f}'))
            for c in range(1, 6):
                self.table.setItem(i, c, QtWidgets.QTableWidgetItem('—'))
        self.table.setMinimumHeight(218)
        left.addWidget(self.table)

        g = QtWidgets.QGroupBox('Effecteur (cinématique directe)')
        gl_ = QtWidgets.QVBoxLayout(g)
        self.lbl_ee = QtWidgets.QLabel('—')
        self.lbl_ee.setStyleSheet('font-family: monospace; font-size: 14px;')
        gl_.addWidget(self.lbl_ee)
        row_info = QtWidgets.QHBoxLayout()
        row_info.addWidget(g, 1)

        gc = QtWidgets.QGroupBox('Liaison STM32 / sécurité')
        f = QtWidgets.QFormLayout(gc)
        self.lbl_link = QtWidgets.QLabel('—')
        self.lbl_lat = QtWidgets.QLabel('—')
        self.lbl_wd = QtWidgets.QLabel('—')
        self.lbl_state = QtWidgets.QLabel('—')
        self.lbl_fault = QtWidgets.QLabel('—')
        self.lbl_homed = QtWidgets.QLabel('—')
        self.lbl_loop = QtWidgets.QLabel('—')
        for label, w in (('Liaison', self.lbl_link), ('Latence', self.lbl_lat), ('Watchdog', self.lbl_wd),
                         ('État', self.lbl_state), ('Défaut', self.lbl_fault), ('Référencés', self.lbl_homed),
                         ('Boucle STM32', self.lbl_loop)):
            f.addRow(label, w)
        row_info.addWidget(gc, 1)
        left.addLayout(row_info)
        root.addLayout(left, 5)

        # ---------- colonne droite : courbes + alarmes
        right = QtWidgets.QVBoxLayout()
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel('Axe :'))
        self.combo_axis = QtWidgets.QComboBox()
        self.combo_axis.addItems([f'J{i + 1}' for i in range(6)])
        bar.addWidget(self.combo_axis)
        bar.addWidget(QtWidgets.QLabel('Fenêtre :'))
        self.spin_window = QtWidgets.QSpinBox()
        self.spin_window.setRange(2, 30)
        self.spin_window.setValue(10)
        self.spin_window.setSuffix(' s')
        bar.addWidget(self.spin_window)
        self.cb_pause = QtWidgets.QCheckBox('Figer')
        self.cb_pause.toggled.connect(lambda v: setattr(self, '_paused', v))
        bar.addWidget(self.cb_pause)
        bar.addStretch(1)
        right.addLayout(bar)

        pg.setConfigOptions(antialias=True)
        self.plot_pos = pg.PlotWidget(title='Consigne vs mesure (°)')
        self.plot_err = pg.PlotWidget(title='Erreur consigne − mesure (°)')
        for p in (self.plot_pos, self.plot_err):
            p.setBackground(theme.BG)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.addLegend(offset=(5, 5))
        self.plot_err.setXLink(self.plot_pos)
        self.curve_cmd = self.plot_pos.plot(pen=pg.mkPen('#ff9a26', width=2, style=QtCore.Qt.DashLine),
                                            name='consigne')
        self.curve_meas = self.plot_pos.plot(pen=pg.mkPen('#3399ff', width=2), name='mesure')
        self.curve_err = self.plot_err.plot(pen=pg.mkPen('#f85149', width=2), name='erreur')
        for curve in (self.curve_cmd, self.curve_meas, self.curve_err):   # peu de points réellement tracés
            curve.setDownsampling(auto=True, method='peak')
            curve.setClipToView(True)
        self._last_window = None
        self.plot_pos.setMinimumHeight(90)
        self.plot_err.setMinimumHeight(70)
        right.addWidget(self.plot_pos, 3)
        right.addWidget(self.plot_err, 2)

        self.alarms = QtWidgets.QListWidget()
        self.alarms.setMaximumHeight(90)
        right.addWidget(QtWidgets.QLabel('Alarmes'))
        right.addWidget(self.alarms)
        root.addLayout(right, 6)

    # ================================================================ alarmes ========
    def add_alarm(self, text: str, level: str = 'warn'):
        item = QtWidgets.QListWidgetItem(f'{time.strftime("%H:%M:%S")}  {text}')
        item.setForeground(QtGui.QColor(theme.ERR if level == 'error' else theme.WARN if level == 'warn'
                                        else theme.TEXT))
        self.alarms.insertItem(0, item)
        while self.alarms.count() > 200:
            self.alarms.takeItem(self.alarms.count() - 1)

    def alarm_count(self) -> int:
        return len(self._active_alarms)

    def _raise(self, key: str, text: str, level: str = 'warn'):
        """Alarme avec anti-répétition : levée une seule fois tant que la condition persiste."""
        if key not in self._active_alarms:
            self._active_alarms.add(key)
            self.add_alarm(text, level)

    def _check_alarms(self, snap: dict):
        q, c = snap['q_meas'], snap['q_cmd']
        wanted = set()
        if q is not None:
            near = np.deg2rad(3.0)
            for i in range(6):
                if q[i] < self.cfg.q_min[i] + near or q[i] > self.cfg.q_max[i] - near:
                    wanted.add(f'limit{i}')
                    self._raise(f'limit{i}', f'J{i + 1} proche de sa limite ({np.rad2deg(q[i]):.1f}°)')
            if c is not None:
                for i in range(6):
                    if abs(c[i] - q[i]) > np.deg2rad(5.0):
                        wanted.add(f'ferr{i}')
                        self._raise(f'ferr{i}', f'J{i + 1} : écart consigne/mesure {np.rad2deg(c[i] - q[i]):.1f}°')
        st = snap['status']
        if st is not None and st[ST_FAULT_CODE]:
            wanted.add('fault')
            self._raise('fault', 'Défaut STM32 : ' + decode_faults(st[ST_FAULT_CODE]), 'error')
        if snap['meas_age'] > 0.5 and snap['q_meas'] is not None and snap['source'] == 'live':
            wanted.add('link')
            self._raise('link', 'Liaison STM32 perdue (plus de /joint_states)', 'error')
        self._active_alarms &= wanted            # les conditions disparues peuvent se relever plus tard

    # ============================================================ rafraîchissement ===
    def update_table(self, snap: dict):
        if not self.isVisible():                       # onglet masqué : on ne met à jour que les alarmes
            self._check_alarms(snap)
            return
        q, c, v, e = snap['q_meas'], snap['q_cmd'], snap['v_meas'], snap['effort']
        for i in range(6):
            vals = [
                '—' if c is None else f'{np.rad2deg(c[i]):8.2f}',
                '—' if q is None else f'{np.rad2deg(q[i]):8.2f}',
                '—' if (q is None or c is None) else f'{np.rad2deg(c[i] - q[i]):7.2f}',
                f'{np.rad2deg(v[i]):7.1f}',
                f'{e[i]:6.2f}',
            ]
            for col, text in enumerate(vals, start=1):
                item = self.table.item(i, col)
                if item.text() != text:                # pas de repeinture si rien n'a changé
                    item.setText(text)
            err = 0.0 if (q is None or c is None) else abs(np.rad2deg(c[i] - q[i]))
            colour = theme.ERR if err > 5 else theme.WARN if err > 2 else theme.TEXT
            self.table.item(i, 3).setForeground(QtGui.QColor(colour))

        T = snap['ee_T']
        if T is None:
            self.lbl_ee.setText('—')
        else:
            r, p, y = np.rad2deg(matrix_to_rpy(T[:3, :3]))
            x_, y_, z_ = T[:3, 3] * 1000.0
            self.lbl_ee.setText(f'X {x_:7.1f}  Y {y_:7.1f}  Z {z_:7.1f} mm\n'
                                f'R {r:7.1f}  P {p:7.1f}  Y {y:7.1f} °')

        live = snap['meas_age'] < 0.3 and snap['js_rate'] > 0
        if snap['source'] == 'replay':
            self.lbl_link.setText(f'● REJEU ({snap["js_rate"]:.0f} Hz)')
            self.lbl_link.setStyleSheet(f'color: {theme.INFO};')
        elif live:
            self.lbl_link.setText(f'● connecté — {snap["js_rate"]:.1f} Hz')
            self.lbl_link.setStyleSheet(f'color: {theme.OK};')
        else:
            self.lbl_link.setText('● déconnecté')
            self.lbl_link.setStyleSheet(f'color: {theme.ERR};')
        lat, rtt = snap['latency_ms'], snap.get('rtt_ms')
        if rtt is not None:
            self.lbl_lat.setText(f'{rtt:.0f} ms aller-retour (≈ {rtt / 2:.0f} ms par sens)')
        else:
            self.lbl_lat.setText('n/d (horloges non synchronisées ?)' if lat is None else f'≈ {lat:.0f} ms (réseau)')

        st = snap['status']
        if st is None:
            for w in (self.lbl_wd, self.lbl_state, self.lbl_fault, self.lbl_homed, self.lbl_loop):
                w.setText('—')
        else:
            comm_fault = bool(st[ST_FAULT_CODE] & FAULT_COMM)
            self.lbl_wd.setText('DÉCLENCHÉ' if comm_fault else f'OK (dernière consigne il y a {st[ST_CMD_AGE_MS]} ms)')
            self.lbl_wd.setStyleSheet(f'color: {theme.ERR if comm_fault else theme.OK};')
            state = st[0]
            self.lbl_state.setText(STATE_NAMES.get(state, str(state)))
            self.lbl_state.setStyleSheet(f'color: {theme.ERR if state >= 5 else theme.OK if state == 4 else theme.WARN};')
            self.lbl_fault.setText(decode_faults(st[ST_FAULT_CODE]))
            homed = [str(i + 1) for i in range(6) if st[ST_HOMED_MASK] & (1 << i)]
            lim = [str(i + 1) for i in range(6) if st[ST_LIMIT_MASK] & (1 << i)]
            self.lbl_homed.setText((' '.join(homed) or 'aucun') + (f'   |  fins de course actifs : {" ".join(lim)}' if lim else ''))
            self.lbl_loop.setText(f'{st[ST_LOOP_HZ]} Hz — écart max {st[ST_MAX_FERR] / 1000.0 * 57.2958:.2f}°')
        self._check_alarms(snap)

    def update_plots(self):
        if self._paused or not self.isVisible():        # figé ou onglet masqué : rien à dessiner
            return
        t, q, c = self.bridge.history(float(self.spin_window.value()))
        if len(t) < 2:
            return
        k = self.combo_axis.currentIndex()
        meas, cmd = np.rad2deg(q[:, k]), np.rad2deg(c[:, k])
        self.curve_meas.setData(t, meas, skipFiniteCheck=True)
        self.curve_cmd.setData(t, cmd, skipFiniteCheck=True)
        self.curve_err.setData(t, cmd - meas, skipFiniteCheck=True)
        window = float(self.spin_window.value())
        if window != self._last_window:                 # la plage X ne change que si l'utilisateur la modifie
            self._last_window = window
            self.plot_pos.setXRange(-window, 0, padding=0)
