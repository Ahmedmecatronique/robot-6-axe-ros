"""Mode DÉMO : un robot simulé DANS le processus de l'IHM — aucun Raspberry Pi, aucun ROS, aucun réseau.

    python windows\\run_gui.py --demo

Il réutilise le vrai code du projet (cinématique inverse, planificateur de trajectoire quintique, limites du
robot.yaml) et reproduit la machine d'états du firmware (DÉSACTIVÉ -> NON RÉFÉRENCÉ -> HOMING -> PRÊT,
arrêt d'urgence verrouillé). Sert à découvrir l'IHM et à la tester ; ce N'EST PAS le robot réel.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import cv2
import numpy as np

from robot_kinematics.status_codes import (CMD_ENABLE, FAULT_ESTOP, STATE_DISABLED, STATE_ESTOP, STATE_HOMING,
                                           STATE_IDLE, STATE_READY)
from robot_kinematics.trajectory import plan_segment
from robot_kinematics.transforms import make_transform, rot_x

from .bridge_core import BridgeCore, pose_dict_to_matrix

ALL_HOMED = 0b111111


class DemoBridge(BridgeCore):
    def __init__(self, start_ready: bool = True):
        super().__init__()
        cfg = self.cfg
        hw = cfg.raw['hardware']
        self.home_pos = np.where(np.array(hw['home_dir']) < 0, cfg.q_min, cfg.q_max)
        self.home_order = list(hw['home_order'])
        self.q = cfg.rest.copy()
        self.v = np.zeros(6)
        self.state = STATE_READY if start_ready else STATE_DISABLED
        self.homed = ALL_HOMED if start_ready else 0
        self.fault = 0
        self.q_cmd, self.v_cmd = self.q.copy(), np.zeros(6)
        self.segment, self.seg_t0, self.queue = None, 0.0, []
        self.speed_scale = 0.5
        self._home_t0 = 0.0
        self._home_from = self.q.copy()
        self._busy_pick = False
        self._cancel_pick = threading.Event()
        # cible factice, posée sur la table, vue « par la caméra »
        self._target = make_transform(rot_x(np.pi), [0.30, 0.12, 0.03])
        self._running = True
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._running = False

    # ================================================================== simulation ======
    def _loop(self):
        dt, last, n = 0.01, time.time(), 0
        cfg = self.cfg
        while self._running:
            time.sleep(dt)
            now = time.time()
            h = min(now - last, 0.05)
            last = now
            n += 1
            # ---- trajectoire (équivalent de trajectory_node)
            if self.state != STATE_READY:
                self.q_cmd, self.v_cmd, self.segment, self.queue = self.q.copy(), np.zeros(6), None, []
            elif self.segment is not None:
                t = now - self.seg_t0
                self.q_cmd, self.v_cmd, _ = self.segment.sample(t)
                if t >= self.segment.duration:
                    self.v_cmd, self.segment = np.zeros(6), None
                    self._start_next()
            # ---- « moteurs » (équivalent du STM32 simulé)
            if self.state == STATE_READY:
                v_des = np.clip(self.v_cmd + 8.0 * (self.q_cmd - self.q), -cfg.v_max, cfg.v_max)
                self.v += np.clip(v_des - self.v, -cfg.a_max * h, cfg.a_max * h)
                self.q = np.clip(self.q + self.v * h, cfg.q_min, cfg.q_max)
            elif self.state == STATE_HOMING:
                self._homing(now)
            else:
                self.v[:] = 0.0
            # ---- publication vers le noyau (comme un vrai transport)
            self.feed_joint_states('live', self.q.copy(), self.v.copy(), self.v / cfg.v_max, 3.0)
            self.feed_command('live', self.q_cmd.copy(), self.v_cmd.copy())
            if n % 2 == 0:
                self.feed_ee('live', self.arm.forward(self.q))
            if n % 10 == 0:
                self.feed_status([self.state, self.homed, 0, self.fault,
                                  int(np.max(np.abs(self.q_cmd - self.q)) * 1000), 1000, 8, int(now - self._t0)])
                self.feed_mode('simulation')
                self.feed_visible(True)
                self.feed_active(self.segment is not None or bool(self.queue))
                self.feed_target('live', self._target)
                self.feed_bag_status({'recording': False, 'playing': False, 'current': '', 'names': []})
            if n % 6 == 0:
                self.feed_image('live', self._frame(now))

    def _start_next(self):
        while self.queue:
            target = self.queue.pop(0)
            k = self.speed_scale
            seg = plan_segment(self.q_cmd, target, self.cfg.v_max * k, self.cfg.a_max * k, v0=self.v_cmd)
            if seg is None:
                continue
            self.segment, self.seg_t0 = seg, time.time()
            return
        self.segment = None

    def _homing(self, now: float):
        per = 0.6
        k = int((now - self._home_t0) / per)
        for n, axis in enumerate(self.home_order):
            if n < k:
                self.q[axis] = self.home_pos[axis]
                self.homed |= 1 << axis
            elif n == k:
                frac = ((now - self._home_t0) - n * per) / per
                self.q[axis] = self._home_from[axis] + frac * (self.home_pos[axis] - self._home_from[axis])
        if k >= len(self.home_order):
            self.state = STATE_READY

    def _frame(self, now: float) -> bytes:
        """Image synthétique « caméra » avec la cible détectée (pour l'onglet Caméra)."""
        img = np.full((360, 480, 3), 46, np.uint8)
        cv2.rectangle(img, (0, 300), (479, 359), (70, 62, 55), -1)
        cx, cy = 240 + int(60 * np.sin(now)), 200
        cv2.rectangle(img, (cx - 22, cy - 22), (cx + 22, cy + 22), (0, 0, 220), -1)
        cv2.rectangle(img, (cx - 22, cy - 22), (cx + 22, cy + 22), (0, 220, 0), 2)
        x, y, z = self._target[:3, 3] * 100
        cv2.putText(img, f'DEMO | objet X={x:.1f} Y={y:.1f} Z={z:.1f} cm', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes()

    # ================================================================== « réseau » ======
    def _publish(self, topic: str, msg_type: str, payload: dict) -> None:
        if topic == '/joint_goal':
            if self.state != STATE_READY:
                self.push_event('log', 'But refusé : robot non prêt (référencer / activer d\'abord)', 'warn')
                return
            self.queue = [np.array(p['positions'], dtype=float) for p in payload['points']]
            self.segment = None
            self._start_next()
        elif topic == '/target_pose':
            res = self._solve(pose_dict_to_matrix(payload['pose']))
            self.feed_ik_result(res)
            if res['success'] and self.state == STATE_READY:
                self._publish('/joint_goal', '', {'points': [{'positions': res['q']}]})
        elif topic == '/emergency_stop':
            if payload.get('data') and self.state not in (STATE_DISABLED, STATE_ESTOP):
                self.state, self.fault = STATE_ESTOP, FAULT_ESTOP
                self.v[:] = 0.0
                self.segment, self.queue = None, []
        elif topic == '/gui/ping':
            self.feed_pong(payload['data'])
        elif topic == '/gripper_command':
            self.push_event('log', 'Pince : ' + ('ouverte' if payload['data'] > 0.5 else 'fermée'), 'info')

    def _solve(self, T: np.ndarray) -> dict:
        if T[2, 3] < self.cfg.z_min:
            return {'success': False, 'message': f'Cible sous le plan de sécurité (z = {T[2, 3]:.3f} m)',
                    'q': [], 'near_singularity': False}
        r = self.arm.solve(T, self.q)
        msg = r.message + (' — ATTENTION proche d\'une singularité : ' + '; '.join(r.singularity_kinds)
                           if r.success and r.near_singularity else '')
        return {'success': bool(r.success), 'message': msg, 'method': r.method,
                'near_singularity': bool(r.near_singularity),
                'q': [] if r.q is None else [float(x) for x in r.q]}

    def _call(self, service: str, srv_type: str, request: dict, on_result: Callable[[Optional[dict]], None]) -> None:
        ok = lambda msg: on_result({'success': True, 'message': msg})   # noqa: E731
        if service == '/robot/enable':
            if request['enable']:
                self.fault = 0
                self.state = STATE_READY if self.homed == ALL_HOMED else STATE_IDLE
                ok('Arrêt d\'urgence effacé, moteurs activés')
            else:
                self.state, self.homed = STATE_DISABLED, 0
                ok('Moteurs désactivés')
        elif service == '/robot/home':
            if self.state in (STATE_DISABLED, STATE_ESTOP):
                on_result({'success': False, 'message': 'Activer les moteurs d\'abord'})
            else:
                self.state, self.homed = STATE_HOMING, 0
                self._home_t0, self._home_from = time.time(), self.q.copy()
                ok('Homing simulé lancé')
        elif service == '/solve_ik':
            res = self._solve(pose_dict_to_matrix(request['target']['pose']))
            on_result({'reachable': res['success'], 'near_singularity': res['near_singularity'],
                       'message': res['message'], 'joint_positions': res['q'], 'method': res.get('method', 'none')})
        elif service == '/trajectory_node/set_parameters':
            self.speed_scale = float(request['parameters'][0]['value']['double_value'])
            on_result({'results': [{'successful': True}]})
        elif service == '/pick/start':
            if self._busy_pick or self.state != STATE_READY:
                on_result({'success': False, 'message': 'saisie impossible (robot non prêt ou déjà en cours)'})
            else:
                self._busy_pick = True
                self._cancel_pick.clear()
                threading.Thread(target=self._pick, daemon=True).start()
                ok('saisie démarrée')
        elif service == '/pick/cancel':
            self._cancel_pick.set()
            ok('annulation demandée')
        elif service == '/bag/command':
            on_result({'success': False, 'message': 'rosbag indisponible en mode démo'})
        else:
            on_result(None)

    def _goto(self, q: np.ndarray) -> bool:
        self._publish('/joint_goal', '', {'points': [{'positions': list(q)}]})
        t_end = time.time() + 60
        time.sleep(0.2)
        while time.time() < t_end and not self._cancel_pick.is_set():
            if np.max(np.abs(self.q - q)) < 0.02:
                return True
            time.sleep(0.02)
        return False

    def _pick(self):
        try:
            steps = []
            grasp = self._target.copy()
            approach = self._target.copy()
            approach[2, 3] += 0.10
            qa, qg = self.arm.solve(approach, self.q), None
            if qa.success:
                qg = self.arm.solve(grasp, qa.q)
            if not (qa.success and qg is not None and qg.success):
                self.feed_pick_status({'done': True, 'success': False, 'message': 'cible inatteignable'})
                return
            steps = [('ouverture de la pince', 0.1, None), ('approche', 0.3, qa.q), ('descente', 0.55, qg.q),
                     ('fermeture de la pince', 0.7, None), ('remontée', 0.85, qa.q),
                     ('retour au repos', 0.95, self.cfg.rest)]
            for name, prog, q in steps:
                if self._cancel_pick.is_set():
                    self.feed_pick_status({'done': True, 'success': False, 'message': 'annulée'})
                    return
                self.feed_pick_status({'phase': name, 'progress': prog, 'done': False})
                if q is None:
                    time.sleep(0.8)
                elif not self._goto(q):
                    self.feed_pick_status({'done': True, 'success': False, 'message': f'{name} : annulée ou timeout'})
                    return
            self.feed_pick_status({'done': True, 'success': True, 'message': 'objet saisi (démo)'})
        finally:
            self._busy_pick = False
