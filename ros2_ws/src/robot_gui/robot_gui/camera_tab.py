"""Vue caméra : flux annoté par vision_node (contour, ArUco, repère), coordonnées 3D de la cible
et bouton « Aller saisir l'objet détecté » (action /pick_object)."""
from __future__ import annotations

import time

import cv2
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from . import theme


class CameraTab(QtWidgets.QWidget):
    log = QtCore.Signal(str, str)

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self._seq = 0
        self._fps_t = time.time()
        self._fps_n = 0
        self._fps = 0.0
        self._picking = False

        lay = QtWidgets.QHBoxLayout(self)
        self.video = QtWidgets.QLabel('En attente de /camera/image_annotated…')
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setMinimumSize(480, 360)
        self.video.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.video.setStyleSheet(f'border: 1px solid {theme.BORDER}; background: #000;')
        lay.addWidget(self.video, 3)

        side = QtWidgets.QVBoxLayout()
        gb = QtWidgets.QGroupBox('Cible détectée (repère base_link)')
        f = QtWidgets.QFormLayout(gb)
        self.lbl_vis = QtWidgets.QLabel('—')
        self.lbl_xyz = QtWidgets.QLabel('—')
        self.lbl_xyz.setStyleSheet('font-family: monospace; font-size: 14px;')
        self.lbl_age = QtWidgets.QLabel('—')
        self.lbl_fps = QtWidgets.QLabel('—')
        f.addRow('Statut', self.lbl_vis)
        f.addRow('Position', self.lbl_xyz)
        f.addRow('Âge de la mesure', self.lbl_age)
        f.addRow('Flux vidéo', self.lbl_fps)
        side.addWidget(gb)

        self.btn_pick = QtWidgets.QPushButton('Aller saisir l\'objet détecté')
        self.btn_pick.setMinimumHeight(48)
        self.btn_pick.clicked.connect(self._pick)
        self.btn_cancel = QtWidgets.QPushButton('Annuler la saisie')
        self.btn_cancel.clicked.connect(self.bridge.cancel_pick)
        self.btn_cancel.setEnabled(False)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, 100)
        self.lbl_phase = QtWidgets.QLabel('')
        self.lbl_phase.setWordWrap(True)
        side.addWidget(self.btn_pick)
        side.addWidget(self.btn_cancel)
        side.addWidget(self.bar)
        side.addWidget(self.lbl_phase)
        side.addStretch(1)
        lay.addLayout(side, 1)

    # ------------------------------------------------------------------ actions ---------
    def _pick(self):
        snap = self.bridge.snapshot()
        if not snap['target_visible'] or snap['target_age'] > 2.0:
            self.lbl_phase.setText('Aucune cible fraîche : la caméra ne voit pas l\'objet.')
            self.lbl_phase.setStyleSheet(f'color: {theme.WARN};')
            return
        self._picking = True
        self.btn_pick.setEnabled(False)
        self.btn_cancel.setEnabled(True)
        self.bar.setValue(0)
        self.lbl_phase.setStyleSheet('')
        self.lbl_phase.setText('Démarrage de la saisie…')
        self.bridge.pick_detected()

    def on_event(self, ev: tuple):
        if ev[0] == 'pick_feedback':
            self.lbl_phase.setText(f'Phase : {ev[1]}')
            self.bar.setValue(int(ev[2] * 100))
        elif ev[0] == 'pick_done':
            self._picking = False
            self.btn_pick.setEnabled(True)
            self.btn_cancel.setEnabled(False)
            ok = bool(ev[1])
            self.bar.setValue(100 if ok else self.bar.value())
            self.lbl_phase.setStyleSheet(f'color: {theme.OK if ok else theme.ERR};')
            self.lbl_phase.setText(('✔ ' if ok else '✖ ') + str(ev[2]))
            self.log.emit(f'Saisie : {ev[2]}', 'info' if ok else 'warn')

    # ------------------------------------------------------------------ affichage -------
    def update_view(self, snap: dict):
        T, age, vis = snap['target_T'], snap['target_age'], snap['target_visible']
        if T is not None and age < 5.0 and vis:
            x, y, z = T[:3, 3] * 1000.0
            self.lbl_vis.setText('● détectée')
            self.lbl_vis.setStyleSheet(f'color: {theme.OK};')
            self.lbl_xyz.setText(f'X {x:7.1f}  Y {y:7.1f}  Z {z:7.1f} mm')
            self.lbl_age.setText(f'{age * 1000:.0f} ms')
        else:
            self.lbl_vis.setText('● aucune cible')
            self.lbl_vis.setStyleSheet(f'color: {theme.MUTED};')
            self.lbl_xyz.setText('—')
            self.lbl_age.setText('—')
        if not self._picking:
            self.btn_pick.setEnabled(vis and snap['source'] == 'live')

        if not self.isVisible():
            return                                      # onglet masqué : ni décodage JPEG ni redimensionnement
        seq, data = self.bridge.image_jpeg()
        if data is not None and seq != self._seq:
            self._seq = seq
            self._fps_n += 1
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                # redimensionnement à la taille d'affichage par OpenCV (bien plus rapide que Qt en mode « smooth »)
                h, w = img.shape[:2]
                scale = min(self.video.width() / w, self.video.height() / h)
                if scale > 0 and abs(scale - 1.0) > 0.02:
                    img = cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))),
                                     interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)
                rgb = np.ascontiguousarray(img[:, :, ::-1])
                h, w, _ = rgb.shape
                qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
                self.video.setPixmap(QtGui.QPixmap.fromImage(qimg))    # fromImage copie les pixels
        now = time.time()
        if now - self._fps_t >= 1.0:
            self._fps = self._fps_n / (now - self._fps_t)
            self._fps_n, self._fps_t = 0, now
        self.lbl_fps.setText(f'{self._fps:.1f} images/s')
