"""Vue 3D temps réel (pyqtgraph.opengl) : jumeau numérique du bras.

Le robot est dessiné avec la MÊME cinématique (ArmKinematics.link_frames) que le nœud fk_node
et avec la même géométrie que l'URDF (moyeu + segment d le long de -z + segment a le long de +x).
Aucun aller-retour TF/RViz : la latence d'affichage se réduit à celle de /joint_states + < 20 ms de rendu.

Éléments affichés : robot MESURÉ (encodeurs, opaque), robot FANTÔME de la CONSIGNE (translucide),
repères des articulations, trajectoire de l'effecteur, cible détectée par la caméra.
"""
from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from robot_kinematics.kinematics import ArmKinematics

from . import theme

_MESH_CACHE = {}


def _t3d(M: np.ndarray):
    """Matrice numpy 4x4 -> Transform3D de pyqtgraph."""
    return pg.Transform3D(*M.reshape(16).tolist())


def _translate(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def _rot_y(a: float) -> np.ndarray:
    M = np.eye(4)
    c, s = np.cos(a), np.sin(a)
    M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c
    return M


def _cylinder(radius: float, length: float) -> gl.MeshData:
    key = ('cyl', round(radius, 4), round(length, 4))
    if key not in _MESH_CACHE:
        _MESH_CACHE[key] = gl.MeshData.cylinder(rows=1, cols=24, radius=[radius, radius], length=length)
    return _MESH_CACHE[key]


def _sphere(radius: float) -> gl.MeshData:
    key = ('sph', round(radius, 4))
    if key not in _MESH_CACHE:
        _MESH_CACHE[key] = gl.MeshData.sphere(rows=8, cols=16, radius=radius)
    return _MESH_CACHE[key]


class RobotModel:
    """Un exemplaire du bras (mesuré ou fantôme) : liste de maillages attachés aux repères U_i."""

    def __init__(self, view: gl.GLViewWidget, arm: ArmKinematics, rgb: Tuple[float, float, float],
                 alpha: float, translucent: bool):
        self.arm = arm
        self.items: List[Tuple[gl.GLMeshItem, int, np.ndarray]] = []   # (item, index lien | -1 = base, local)
        cfg = arm.cfg
        opts = 'translucent' if translucent else 'opaque'

        def add(md, link: int, local: np.ndarray, shade: float = 1.0):
            colour = (rgb[0] * shade, rgb[1] * shade, rgb[2] * shade, alpha)
            item = gl.GLMeshItem(meshdata=md, smooth=True, shader='shaded', color=colour, glOptions=opts)
            view.addItem(item)
            self.items.append((item, link, local))

        add(_cylinder(0.09, 0.04), -1, np.eye(4), 0.6)                    # socle
        for i in range(6):
            shade = 1.0 if i % 2 == 0 else 0.75                            # alternance comme dans l'URDF
            add(_cylinder(0.045, 0.07), i, _translate(z=-0.035), 0.6)      # moyeu de l'articulation (axe z)
            d, a = float(cfg.d[i]), float(cfg.a[i])
            if d > 1e-6:                                                   # segment d : de l'origine vers -z
                add(_cylinder(0.03, d), i, _translate(z=-d), shade)
            if a > 1e-6:                                                   # segment a : de l'origine vers +x
                add(_cylinder(0.03, a), i, _rot_y(np.pi / 2), shade)
        add(_sphere(0.018), 5, np.eye(4), 1.0)                             # TCP (tool0)

    def update(self, q: np.ndarray, dh_frames=None):
        frames = self.arm.link_frames(q, dh_frames)    # U_1 .. U_6 (repères DH déjà connus : pas de recalcul)
        for item, link, local in self.items:
            M = np.eye(4) if link < 0 else frames[link]
            item.setTransform(_t3d(M @ local))

    def set_visible(self, visible: bool):
        for item, _, _ in self.items:
            item.setVisible(visible)


class RobotView3D(QtWidgets.QWidget):
    def __init__(self, arm: ArmKinematics, parent=None):
        super().__init__(parent)
        self.arm = arm
        self._last_q = None                           # dernières poses dessinées : on ne redessine que si ça change
        self._last_cmd = None
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(theme.BG)
        self.view.setMinimumSize(300, 220)
        self.view.setCameraPosition(distance=1.7, elevation=25, azimuth=45)
        self.view.opts['center'] = pg.Vector(0, 0, 0.35)

        grid = gl.GLGridItem()
        grid.setSize(1.2, 1.2)
        grid.setSpacing(0.1, 0.1)
        grid.setColor((80, 90, 100, 120))
        self.view.addItem(grid)
        world = gl.GLAxisItem(size=QtGui.QVector3D(0.2, 0.2, 0.2))
        self.view.addItem(world)

        self.real = RobotModel(self.view, arm, theme.REAL_RGB, 1.0, translucent=False)
        self.ghost = RobotModel(self.view, arm, theme.GHOST_RGB, 0.30, translucent=True)

        self.frame_axes = [gl.GLAxisItem(size=QtGui.QVector3D(0.06, 0.06, 0.06)) for _ in range(7)]
        for ax in self.frame_axes:
            self.view.addItem(ax)
        self.trail_pts = deque(maxlen=800)
        self.trail = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=(1.0, 0.9, 0.2, 1.0), width=2, antialias=True)
        self.view.addItem(self.trail)
        self.target = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=22, color=(0.2, 1.0, 0.3, 1.0), pxMode=True)
        self.target_axes = gl.GLAxisItem(size=QtGui.QVector3D(0.08, 0.08, 0.08))
        self.view.addItem(self.target)
        self.view.addItem(self.target_axes)
        self.target.setVisible(False)
        self.target_axes.setVisible(False)

        self._follow = False
        self._build_toolbar()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.toolbar)
        lay.addWidget(self.view, 1)
        self.apply_options()
        self.real.update(self.arm.cfg.rest)
        self.ghost.update(self.arm.cfg.rest)

    # ------------------------------------------------------------------ barre d'outils --
    def _build_toolbar(self):
        self.toolbar = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(self.toolbar)
        h.setContentsMargins(4, 2, 4, 2)
        for text, fn in (('Iso', lambda: self._preset(25, 45)), ('Face', lambda: self._preset(0, -90)),
                         ('Côté', lambda: self._preset(0, 0)), ('Dessus', lambda: self._preset(89, -90))):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            h.addWidget(b)
        self.cb_follow = QtWidgets.QCheckBox('Suivre l\'effecteur')
        self.cb_ghost = QtWidgets.QCheckBox('Fantôme')
        self.cb_tf = QtWidgets.QCheckBox('Repères')
        self.cb_trail = QtWidgets.QCheckBox('Trace')
        self.cb_target = QtWidgets.QCheckBox('Cible')
        for cb, on in ((self.cb_follow, False), (self.cb_ghost, True), (self.cb_tf, True),
                       (self.cb_trail, True), (self.cb_target, True)):
            cb.setChecked(on)
            cb.toggled.connect(self.apply_options)
            h.addWidget(cb)
        clear = QtWidgets.QPushButton('Effacer')
        clear.clicked.connect(self.trail_pts.clear)
        h.addWidget(clear)
        h.addStretch(1)
        self.lbl_info = QtWidgets.QLabel('En attente de /joint_states…')
        self.lbl_info.setMinimumWidth(10)
        self.lbl_info.setStyleSheet(f'color: {theme.MUTED};')
        h.addWidget(self.lbl_info)

    def _preset(self, elevation: float, azimuth: float):
        self.view.setCameraPosition(elevation=elevation, azimuth=azimuth)

    def apply_options(self):
        self._last_q = self._last_cmd = None          # force le rafraîchissement après un changement d'option
        self._follow = self.cb_follow.isChecked()
        self.ghost.set_visible(self.cb_ghost.isChecked())
        for ax in self.frame_axes:
            ax.setVisible(self.cb_tf.isChecked())
        self.trail.setVisible(self.cb_trail.isChecked())

    # ------------------------------------------------------------------ mise à jour ----
    def update_view(self, snap: dict):
        q = snap['q_meas']
        if q is None:
            self.lbl_info.setText('En attente de /joint_states…')
            return
        frames = self.arm.forward_frames(q)            # calculé UNE seule fois par image
        moved = self._last_q is None or not np.array_equal(q, self._last_q)
        if moved:
            self.real.update(q, frames)
            self._last_q = q.copy()
            if self.cb_tf.isChecked():
                for ax, T in zip(self.frame_axes, frames):
                    ax.setTransform(_t3d(T))
        cmd = snap['q_cmd']
        if cmd is not None and self.cb_ghost.isChecked() and (
                self._last_cmd is None or not np.array_equal(cmd, self._last_cmd)):
            self.ghost.update(cmd)
            self._last_cmd = cmd.copy()

        T_tool = frames[6]
        tip = T_tool[:3, 3]
        if not self.trail_pts or np.linalg.norm(tip - self.trail_pts[-1]) > 0.003:
            self.trail_pts.append(tip.copy())
            if self.cb_trail.isChecked() and len(self.trail_pts) > 1:
                self.trail.setData(pos=np.array(self.trail_pts))
        if self._follow:
            self.view.opts['center'] = pg.Vector(*tip)

        show_target = snap['target_T'] is not None and snap['target_age'] < 5.0 and self.cb_target.isChecked()
        self.target.setVisible(show_target)
        self.target_axes.setVisible(show_target)
        if show_target:
            T = snap['target_T']
            self.target.setData(pos=T[:3, 3].reshape(1, 3))
            self.target_axes.setTransform(_t3d(T))

        err = 0.0 if cmd is None else float(np.rad2deg(np.max(np.abs(cmd - q))))
        self.lbl_info.setText(f'écart consigne/mesure max : {err:.2f}°')
