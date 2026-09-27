"""Fenêtre principale : barre d'état, vue 3D + panneau de contrôle, onglets (tableau de bord, caméra,
enregistrement/apprentissage, diagnostics). L'IHM tourne sur le PC portable, pas sur le Raspberry Pi."""
from __future__ import annotations

import time

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from robot_kinematics.status_codes import STATE_NAMES

from . import theme
from .camera_tab import CameraTab
from .dashboard import Dashboard
from .panels import ControlPanel
from .teach_tab import TeachTab
from .view3d import RobotView3D

LEVEL_NAMES = {0: 'OK', 1: 'ATTENTION', 2: 'ERREUR', 3: 'PÉRIMÉ'}
LEVEL_COLORS = {0: theme.OK, 1: theme.WARN, 2: theme.ERR, 3: theme.MUTED}


class DiagnosticsTab(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['Composant', 'Niveau', 'Message', 'Valeurs'])
        self.table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.logbox = QtWidgets.QPlainTextEdit()
        self.logbox.setReadOnly(True)
        self.logbox.setMaximumBlockCount(500)
        lay.addWidget(self.table, 3)
        lay.addWidget(self.logbox, 2)

    def update_diagnostics(self, diags: list):
        self.table.setRowCount(len(diags))
        for r, d in enumerate(diags):
            vals = '  '.join(f'{k}={v}' for k, v in d['values'].items())
            for c, text in enumerate((d['name'], LEVEL_NAMES.get(d['level'], '?'), d['message'], vals)):
                it = QtWidgets.QTableWidgetItem(text)
                if c == 1:
                    it.setForeground(QtGui.QColor(LEVEL_COLORS.get(d['level'], theme.TEXT)))
                self.table.setItem(r, c, it)

    def add_log(self, text: str, level: str):
        colour = {'error': theme.ERR, 'warn': theme.WARN}.get(level, theme.TEXT)
        self.logbox.appendHtml(f'<span style="color:{theme.MUTED}">{time.strftime("%H:%M:%S")}</span> '
                               f'<span style="color:{colour}">{text}</span>')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle('Bras 6 axes — supervision / jumeau numérique')

        # ---------- barre d'état
        bar = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        self.b_mode = QtWidgets.QLabel('MODE ?')
        self.b_source = QtWidgets.QLabel('DIRECT')
        self.b_link = QtWidgets.QLabel('● STM32 : —')
        self.b_state = QtWidgets.QLabel('État : —')
        self.b_wd = QtWidgets.QLabel('Watchdog : —')
        self.b_alarms = QtWidgets.QLabel('⚠ 0 alarme')
        self.b_lat = QtWidgets.QLabel('Latence : —')
        self.b_perf = QtWidgets.QLabel('IHM : —')
        for w in (self.b_mode, self.b_source, self.b_link, self.b_state, self.b_wd, self.b_alarms, self.b_lat,
                  self.b_perf):
            h.addWidget(w)
            h.addSpacing(14)
        h.addStretch(1)

        # ---------- corps
        self.view3d = RobotView3D(bridge.arm)
        self.panel = ControlPanel(bridge)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(self.panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(430)
        right = QtWidgets.QWidget()                  # colonne de droite : contrôles défilants + ARRÊT D'URGENCE fixe
        rl = QtWidgets.QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addWidget(scroll, 1)
        rl.addWidget(self.panel.lbl_state)
        rl.addWidget(self.panel.b_estop)
        top = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        top.addWidget(self.view3d)
        top.addWidget(right)
        top.setStretchFactor(0, 3)
        top.setStretchFactor(1, 1)

        self.dashboard = Dashboard(bridge)
        self.camera = CameraTab(bridge)
        self.teach = TeachTab(bridge)
        self.diag = DiagnosticsTab()
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.dashboard, 'Tableau de bord')
        tabs.addTab(self.camera, 'Caméra')
        tabs.addTab(self.teach, 'Enregistrement / Apprentissage')
        tabs.addTab(self.diag, 'Diagnostics')

        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        split.addWidget(top)
        split.addWidget(tabs)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)

        central = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(bar)
        lay.addWidget(split, 1)
        self.setCentralWidget(central)
        top.setSizes([1000, 460])
        split.setSizes([600, 380])
        self.resize(1500, 1000)

        for w in (self.panel, self.camera, self.teach):
            w.log.connect(self.log)

        # ---------- boucle d'affichage : 60 Hz pour la 3D, fréquences plus basses pour le reste
        self._n = 0
        self._tick_ms = 0.0                           # coût moyen d'une itération (affiché)
        self._frames, self._fps, self._t_fps = 0, 0.0, time.time()
        self._t_ping = 0.0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)

    # ------------------------------------------------------------------ journal ---------
    def log(self, text: str, level: str = 'info'):
        self.diag.add_log(text, level)
        if level in ('warn', 'error'):
            self.dashboard.add_alarm(text, level)
        self.statusBar().showMessage(text, 5000)

    def _dispatch(self, ev: tuple):
        kind = ev[0]
        if kind == 'alarm':
            self.dashboard.add_alarm(ev[1], ev[2])
            self.diag.add_log(ev[1], ev[2])
        elif kind == 'log':
            self.log(ev[1], ev[2])
        elif kind == 'service':
            self.log(f'{ev[1]} : {ev[3]}', 'info' if ev[2] else 'warn')
        elif kind in ('ik_preview', 'ik_result'):
            self.panel.on_event(ev)
        elif kind in ('pick_feedback', 'pick_done'):
            self.camera.on_event(ev)

    # ------------------------------------------------------------------ boucle ----------
    def _tick(self):
        t0 = time.perf_counter()
        snap = self.bridge.snapshot()
        self.view3d.update_view(snap)              # 60 Hz, seulement si la pose a changé
        self._n += 1
        for ev in self.bridge.drain_events():
            self._dispatch(ev)
        # Les cadences sont décalées pour ne jamais cumuler tous les travaux sur la même image
        if self._n % 4 == 1:
            self.dashboard.update_plots()          # 15 Hz (ignoré si l'onglet est masqué)
        if self._n % 4 == 3:
            self.camera.update_view(snap)          # 15 Hz (ignoré si l'onglet est masqué)
        if self._n % 6 == 0:                       # 10 Hz
            self.panel.update_state(snap)
            self.dashboard.update_table(snap)
            self.teach.update_state(snap)
            self._update_bar(snap)
        if self._n % 10 == 5:
            self.diag.update_diagnostics(snap['diagnostics'])
        now = time.time()
        if now - self._t_ping >= 0.5:              # ping aller-retour : 2 fois par seconde
            self._t_ping = now
            ping = getattr(self.bridge, 'send_ping', None)
            if ping is not None:
                ping()
        # indicateurs de performance de l'IHM elle-même
        self._tick_ms += 0.05 * ((time.perf_counter() - t0) * 1e3 - self._tick_ms)
        self._frames += 1
        if now - self._t_fps >= 1.0:
            self._fps, self._frames, self._t_fps = self._frames / (now - self._t_fps), 0, now

    def _update_bar(self, snap: dict):
        mode = snap['mode']
        if mode == 'simulation':
            self.b_mode.setText('SIMULATION')
            self.b_mode.setStyleSheet(theme.badge('#04260f', theme.OK))
        elif mode == 'real':
            self.b_mode.setText('RÉEL')
            self.b_mode.setStyleSheet(theme.badge('white', '#c62828'))
        else:
            self.b_mode.setText('MODE ?')
            self.b_mode.setStyleSheet(theme.badge(theme.TEXT, theme.BORDER))
        if snap['source'] == 'replay':
            self.b_source.setText('REJEU')
            self.b_source.setStyleSheet(theme.badge('white', theme.INFO))
        else:
            self.b_source.setText('DIRECT')
            self.b_source.setStyleSheet(theme.badge(theme.TEXT, theme.PANEL))
        live = snap['meas_age'] < 0.3 and snap['js_rate'] > 0
        self.b_link.setText(f'● STM32 : {"connecté " + format(snap["js_rate"], ".0f") + " Hz" if live else "déconnecté"}')
        self.b_link.setStyleSheet(f'color: {theme.OK if live else theme.ERR};')
        st = snap['state']
        self.b_state.setText('État : ' + (STATE_NAMES.get(st, '—') if st >= 0 else '—'))
        self.b_state.setStyleSheet(f'color: {theme.OK if st == 4 else theme.ERR if st >= 5 else theme.WARN};')
        self.b_wd.setText('Watchdog : ' + ('DÉCLENCHÉ' if (snap['status'] and snap['status'][3] & 1) else 'OK'))
        n = self.dashboard.alarm_count()
        self.b_alarms.setText(f'⚠ {n} alarme{"s" if n > 1 else ""}')
        self.b_alarms.setStyleSheet(f'color: {theme.ERR if n else theme.MUTED};')
        lat, rtt = snap['latency_ms'], snap.get('rtt_ms')
        if rtt is not None:
            self.b_lat.setText(f'Latence : {rtt:.0f} ms A/R')
            self.b_lat.setStyleSheet(f'color: {theme.OK if rtt < 100 else theme.WARN if rtt < 250 else theme.ERR};')
        else:
            self.b_lat.setText('Latence : ' + ('n/d' if lat is None else f'≈ {lat:.0f} ms'))
        self.b_perf.setText(f'IHM : {self._fps:.0f} img/s · {self._tick_ms:.1f} ms')

    def closeEvent(self, event):
        self.timer.stop()
        self.bridge.shutdown()
        super().closeEvent(event)
