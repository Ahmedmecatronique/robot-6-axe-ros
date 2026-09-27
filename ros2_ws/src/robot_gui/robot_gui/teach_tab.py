"""Enregistrement / rejeu de sessions (rosbag2) et mode apprentissage (teach) :
mémoriser des poses articulaires puis les rejouer en séquence (pauses et pince par pose)."""
from __future__ import annotations

import json
import os
import time
from typing import List, Optional

import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets

from robot_kinematics.status_codes import STATE_READY

from . import theme

DEFAULT_TEACH_FILE = os.path.expanduser('~/robot_teach.json')


class TeachTab(QtWidgets.QWidget):
    log = QtCore.Signal(str, str)

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.steps: List[dict] = []          # {name, q (rad), gripper: None|0.0|1.0, dwell (s)}
        self._play_i = -1                    # index de la pose en cours (-1 : arrêté)
        self._phase = 'idle'                 # idle | moving | dwell
        self._t_phase = 0.0
        self._loop = False
        self._bag_names: List[str] = []
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    # =============================================================== construction ===
    def _build(self):
        root = QtWidgets.QHBoxLayout(self)

        # ---------------- rosbag2
        gb = QtWidgets.QGroupBox('Enregistrement / rejeu de session (rosbag2)')
        v = QtWidgets.QVBoxLayout(gb)
        self.btn_rec = QtWidgets.QPushButton('● Enregistrer la session')
        self.btn_rec.clicked.connect(self._toggle_record)
        self.lbl_rec = QtWidgets.QLabel('')
        self.list_bags = QtWidgets.QListWidget()
        b_ref = QtWidgets.QPushButton('Actualiser la liste')
        b_ref.clicked.connect(self._refresh_bags)
        self.btn_play = QtWidgets.QPushButton('▶ Rejouer dans l\'IHM')
        self.btn_play.clicked.connect(self._toggle_replay)
        self.lbl_play = QtWidgets.QLabel('Le rejeu est redirigé vers /replay/… : aucune commande n\'est envoyée au robot.')
        self.lbl_play.setWordWrap(True)
        self.lbl_play.setStyleSheet(f'color: {theme.MUTED};')
        for w in (self.btn_rec, self.lbl_rec, QtWidgets.QLabel('Sessions enregistrées :'), self.list_bags,
                  b_ref, self.btn_play, self.lbl_play):
            v.addWidget(w)
        root.addWidget(gb, 2)
        self._refresh_bags()

        # ---------------- teach
        gt = QtWidgets.QGroupBox('Mode apprentissage (teach)')
        t = QtWidgets.QVBoxLayout(gt)
        row = QtWidgets.QHBoxLayout()
        b_add = QtWidgets.QPushButton('＋ Mémoriser la pose actuelle')
        b_add.clicked.connect(self._add_pose)
        b_del = QtWidgets.QPushButton('Supprimer')
        b_del.clicked.connect(self._del_pose)
        b_up = QtWidgets.QPushButton('▲')
        b_up.clicked.connect(lambda: self._move(-1))
        b_dn = QtWidgets.QPushButton('▼')
        b_dn.clicked.connect(lambda: self._move(1))
        for b in (b_add, b_del, b_up, b_dn):
            row.addWidget(b)
        t.addLayout(row)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['Nom', 'Articulations (°)', 'Pince', 'Pause (s)'])
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.itemChanged.connect(self._table_edited)
        t.addWidget(self.table, 1)

        row2 = QtWidgets.QHBoxLayout()
        b_goto = QtWidgets.QPushButton('Aller à la pose')
        b_goto.clicked.connect(self._goto_selected)
        self.btn_seq = QtWidgets.QPushButton('▶ Jouer la séquence')
        self.btn_seq.clicked.connect(self._toggle_sequence)
        self.cb_loop = QtWidgets.QCheckBox('Boucle')
        b_save = QtWidgets.QPushButton('Enregistrer')
        b_save.clicked.connect(self._save)
        b_load = QtWidgets.QPushButton('Charger')
        b_load.clicked.connect(self._load)
        for w in (b_goto, self.btn_seq, self.cb_loop, b_save, b_load):
            row2.addWidget(w)
        t.addLayout(row2)
        self.lbl_seq = QtWidgets.QLabel('')
        t.addWidget(self.lbl_seq)
        root.addWidget(gt, 4)

    # ============================================================== rosbag2 ==========
    def _toggle_record(self):
        if self.bridge.bag_recording():
            self.bridge.bag_record_stop()
            self.log.emit('Enregistrement arrêté', 'info')
        else:
            ok, msg = self.bridge.bag_record_start()
            self.log.emit(('Enregistrement : ' if ok else 'Échec enregistrement : ') + msg, 'info' if ok else 'error')
        self._refresh_bags()

    def _refresh_bags(self):
        self.bridge.bag_refresh()                    # demande la liste au robot (bag_node)
        self._fill_bags()

    def _fill_bags(self):
        self._bag_names = self.bridge.list_bags()
        self.list_bags.clear()
        self.list_bags.addItems(self._bag_names)

    def _toggle_replay(self):
        if self.bridge.bag_playing():
            self.bridge.bag_replay_stop()
            self.log.emit('Rejeu arrêté : retour au direct', 'info')
            return
        item = self.list_bags.currentItem()
        if item is None:
            self.log.emit('Sélectionnez une session à rejouer', 'warn')
            return
        ok, msg = self.bridge.bag_replay_start(item.text())
        self.log.emit(('Rejeu : ' if ok else 'Échec rejeu : ') + msg, 'info' if ok else 'error')

    # ================================================================== teach ========
    def _add_pose(self):
        q = self.bridge.snapshot()['q_meas']
        if q is None:
            self.log.emit('Pas de position mesurée', 'warn')
            return
        self.steps.append({'name': f'Pose {len(self.steps) + 1}', 'q': [float(x) for x in q],
                           'gripper': None, 'dwell': 0.5})
        self._refresh_table()

    def _refresh_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.steps))
        for r, s in enumerate(self.steps):
            self.table.setItem(r, 0, QtWidgets.QTableWidgetItem(s['name']))
            it = QtWidgets.QTableWidgetItem(', '.join(f'{v:.1f}' for v in np.rad2deg(s['q'])))
            it.setFlags(it.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(r, 1, it)
            g = {None: 'inchangée', 1.0: 'ouvrir', 0.0: 'fermer'}[s['gripper']]
            self.table.setItem(r, 2, QtWidgets.QTableWidgetItem(g))
            self.table.setItem(r, 3, QtWidgets.QTableWidgetItem(f'{s["dwell"]:.1f}'))
        self.table.blockSignals(False)

    def _table_edited(self, item):
        r, c = item.row(), item.column()
        if r >= len(self.steps):
            return
        try:
            if c == 0:
                self.steps[r]['name'] = item.text()
            elif c == 2:
                self.steps[r]['gripper'] = {'ouvrir': 1.0, 'fermer': 0.0}.get(item.text().strip().lower())
            elif c == 3:
                self.steps[r]['dwell'] = max(0.0, float(item.text().replace(',', '.')))
        except ValueError:
            pass
        self._refresh_table()

    def _selected(self) -> Optional[int]:
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _del_pose(self):
        r = self._selected()
        if r is not None:
            del self.steps[r]
            self._refresh_table()

    def _move(self, d: int):
        r = self._selected()
        if r is None or not 0 <= r + d < len(self.steps):
            return
        self.steps[r], self.steps[r + d] = self.steps[r + d], self.steps[r]
        self._refresh_table()
        self.table.selectRow(r + d)

    def _goto_selected(self):
        r = self._selected()
        if r is not None:
            self.bridge.send_joint_goal([self.steps[r]['q']])

    def _save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Enregistrer la séquence', DEFAULT_TEACH_FILE, 'JSON (*.json)')
        if path:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(self.steps, f, indent=2, ensure_ascii=False)
            self.log.emit(f'Séquence enregistrée : {path}', 'info')

    def _load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Charger une séquence', DEFAULT_TEACH_FILE, 'JSON (*.json)')
        if path:
            with open(path, 'r', encoding='utf-8') as f:
                self.steps = json.load(f)
            self._refresh_table()
            self.log.emit(f'Séquence chargée : {path}', 'info')

    # ------------------------------------------------------ lecture de la séquence ---
    def _toggle_sequence(self):
        if self._play_i >= 0:
            self._stop_sequence('arrêtée')
            return
        if not self.steps:
            self.log.emit('Aucune pose mémorisée', 'warn')
            return
        if self.bridge.snapshot()['state'] != STATE_READY:
            self.log.emit('Robot non prêt : impossible de jouer la séquence', 'warn')
            return
        self._loop = self.cb_loop.isChecked()
        self._start_step(0)
        self.btn_seq.setText('■ Arrêter la séquence')

    def _stop_sequence(self, why: str):
        self._play_i, self._phase = -1, 'idle'
        self.btn_seq.setText('▶ Jouer la séquence')
        self.lbl_seq.setText(f'Séquence {why}.')
        self.bridge.soft_stop()

    def _start_step(self, i: int):
        s = self.steps[i]
        self._play_i, self._phase, self._t_phase = i, 'moving', time.time()
        if s['gripper'] is not None:
            self.bridge.send_gripper(s['gripper'])
        self.bridge.send_joint_goal([s['q']])
        self.lbl_seq.setText(f'Pose {i + 1}/{len(self.steps)} : {s["name"]}')
        self.table.selectRow(i)

    def _tick(self):
        if self._play_i < 0:
            return
        snap = self.bridge.snapshot()
        if snap['state'] != STATE_READY:
            self._stop_sequence('interrompue (robot non prêt)')
            return
        s = self.steps[self._play_i]
        if self._phase == 'moving':
            q = snap['q_meas']
            arrived = q is not None and np.max(np.abs(q - np.array(s['q']))) < np.deg2rad(0.8) \
                and not snap['traj_active'] and time.time() - self._t_phase > 0.3
            if arrived:
                self._phase, self._t_phase = 'dwell', time.time()
            elif time.time() - self._t_phase > 90.0:
                self._stop_sequence('interrompue (pose non atteinte)')
        elif self._phase == 'dwell' and time.time() - self._t_phase >= s['dwell']:
            nxt = self._play_i + 1
            if nxt >= len(self.steps):
                if self._loop:
                    nxt = 0
                else:
                    self._stop_sequence('terminée')
                    return
            self._start_step(nxt)

    def update_state(self, snap: dict):
        rec = self.bridge.bag_recording()
        self.btn_rec.setText('■ Arrêter l\'enregistrement' if rec else '● Enregistrer la session')
        self.lbl_rec.setText('Enregistrement en cours…' if rec else '')
        self.lbl_rec.setStyleSheet(f'color: {theme.ERR};')
        play = self.bridge.bag_playing()
        self.btn_play.setText('■ Arrêter le rejeu' if play else '▶ Rejouer dans l\'IHM')
        if self.bridge.list_bags() != self._bag_names:   # liste mise à jour par /bag/status
            self._fill_bags()
        if self.bridge.replay_finished():                # le rejeu s'est terminé tout seul
            self.bridge.bag_replay_stop()
