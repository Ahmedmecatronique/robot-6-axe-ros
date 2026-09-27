"""Cinématique directe et inverse du bras 6 axes (anthropomorphe + poignet sphérique).

Convention Denavit-Hartenberg STANDARD :

    A_i = Rz(theta_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)

    i |  theta_i        | d_i | a_i | alpha_i
    --+-----------------+-----+-----+--------
    1 |  q1             | d1  | 0   | +90°
    2 |  q2 + 90°       | 0   | a2  |   0
    3 |  q3             | 0   | a3  |   0
    4 |  q4             | 0   | 0   | +90°
    5 |  q5             | 0   | 0   | -90°
    6 |  q6             | d6  | 0   |   0

(les décalages theta_offset sont lus dans robot.yaml ; à q = 0 le bras est vertical).

Les axes 4, 5, 6 se coupent au centre du poignet, ce qui permet de découpler
position (q1..q3) et orientation (q4..q6) : solution analytique, jusqu'à 8 configurations.

Toutes les fonctions travaillent en radians / mètres et sur des matrices homogènes 4x4.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .config import RobotConfig
from .transforms import angle_diff, rotation_to_rotvec, wrap_angle


@dataclass
class SingularityInfo:
    """Résultat de l'analyse de singularité pour une configuration."""

    near: bool
    kinds: List[str]
    sigma_min: float  # plus petite valeur singulière de la Jacobienne normalisée


@dataclass
class IKResult:
    """Résultat d'un calcul de cinématique inverse."""

    success: bool
    q: Optional[np.ndarray]
    method: str                      # "analytical" | "numerical" | "none"
    message: str
    pos_err: float = float('nan')    # m
    ori_err: float = float('nan')    # rad
    near_singularity: bool = False
    singularity_kinds: List[str] = field(default_factory=list)
    n_solutions: int = 0             # nb de solutions analytiques valides trouvées


class ArmKinematics:
    """FK, Jacobienne, IK analytique + numérique, singularités et limites."""

    # Seuils de singularité
    WRIST_EPS_DEG = 5.0       # |q5| < 5°  -> axes 4 et 6 alignés
    ELBOW_EPS_DEG = 5.0       # |sin q3| < sin 5° -> bras tendu ou replié
    SHOULDER_EPS_M = 0.02     # centre du poignet à < 2 cm de l'axe de la base
    SIGMA_MIN = 0.05          # seuil sur la Jacobienne normalisée

    def __init__(self, cfg: RobotConfig):
        self.cfg = cfg
        self.d, self.a, self.alpha = cfg.d, cfg.a, cfg.alpha
        self.off = cfg.theta_offset
        self.q_min, self.q_max = cfg.q_min, cfg.q_max
        self._wrist_eps = np.sin(np.deg2rad(self.WRIST_EPS_DEG))
        self._elbow_eps = np.sin(np.deg2rad(self.ELBOW_EPS_DEG))

    # ------------------------------------------------------------------ FK ----
    @staticmethod
    def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Matrice homogène 4x4 d'un maillon en convention DH standard."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,      d],
            [0.0,     0.0,      0.0,    1.0],
        ])

    def forward_frames(self, q) -> List[np.ndarray]:
        """Repères DH T_0^i pour i = 0..6 (7 matrices, la première est l'identité)."""
        theta = np.asarray(q, dtype=float) + self.off
        frames = [np.eye(4)]
        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh_matrix(theta[i], self.d[i], self.a[i], self.alpha[i])
            frames.append(T)
        return frames

    def forward(self, q) -> np.ndarray:
        """Pose de l'outil (bride) dans la base : matrice homogène 4x4."""
        return self.forward_frames(q)[-1]

    def link_frames(self, q, frames: Optional[List[np.ndarray]] = None) -> List[np.ndarray]:
        """Repères 'URDF' U_i = T_0^(i-1) · Tz(d_i) · Rz(theta_i), i = 1..6.

        Ce sont les repères des liens dans l'URDF : l'IHM les utilise pour dessiner le robot
        avec exactement le même modèle que RViz.
        """
        theta = np.asarray(q, dtype=float) + self.off
        if frames is None:                 # l'appelant peut fournir les repères DH déjà calculés
            frames = self.forward_frames(q)
        out = []
        for i in range(6):
            Tz = np.eye(4)
            Tz[2, 3] = self.d[i]
            c, s = np.cos(theta[i]), np.sin(theta[i])
            Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
            out.append(frames[i] @ Tz @ Rz)
        return out

    def jacobian(self, q) -> np.ndarray:
        """Jacobienne géométrique 6x6 dans le repère base (v ; w) au point d'origine de l'outil.

        Pour une articulation rotoïde i (axe z_(i-1)) :  J_v = z × (o_n - o_(i-1)),  J_w = z.
        """
        frames = self.forward_frames(q)
        o_n = frames[6][:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            z = frames[i][:3, 2]
            o = frames[i][:3, 3]
            J[:3, i] = np.cross(z, o_n - o)
            J[3:, i] = z
        return J

    # -------------------------------------------------- limites / singularités --
    def within_limits(self, q, margin: float = 0.0) -> Tuple[bool, List[int]]:
        """Vérifie les limites articulaires ; retourne (ok, indices violés)."""
        q = np.asarray(q, dtype=float)
        bad = np.where((q < self.q_min - margin) | (q > self.q_max + margin))[0]
        return len(bad) == 0, [int(i) for i in bad]

    def clip_to_limits(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.q_min, self.q_max)

    def singularity_info(self, q) -> SingularityInfo:
        """Détecte les trois singularités classiques + le conditionnement de la Jacobienne."""
        q = np.asarray(q, dtype=float)
        theta = q + self.off
        frames = self.forward_frames(q)
        kinds = []
        if abs(np.sin(theta[4])) < self._wrist_eps:
            kinds.append('poignet (q5 ≈ 0 : axes 4 et 6 alignés)')
        if abs(np.sin(theta[2])) < self._elbow_eps:
            kinds.append('coude (bras tendu ou replié)')
        pc = frames[3][:3, 3]  # centre du poignet
        if np.hypot(pc[0], pc[1]) < self.SHOULDER_EPS_M:
            kinds.append('épaule (centre du poignet sur l\'axe de la base)')
        J = self.jacobian(q)
        L = self.cfg.reach
        Js = np.vstack([J[:3] / L, J[3:]])       # même ordre de grandeur pour v et w
        sigma_min = float(np.linalg.svd(Js, compute_uv=False)[-1])
        if sigma_min < self.SIGMA_MIN and not kinds:
            kinds.append('conditionnement de la Jacobienne faible')
        return SingularityInfo(near=bool(kinds), kinds=kinds, sigma_min=sigma_min)

    # ------------------------------------------------------- erreur de pose ----
    @staticmethod
    def pose_error(T_target: np.ndarray, T_current: np.ndarray) -> np.ndarray:
        """Erreur 6D (position ; vecteur rotation) exprimée dans la base."""
        e = np.zeros(6)
        e[:3] = T_target[:3, 3] - T_current[:3, 3]
        e[3:] = rotation_to_rotvec(T_target[:3, :3] @ T_current[:3, :3].T)
        return e

    def fk_error(self, T_target: np.ndarray, q) -> Tuple[float, float]:
        e = self.pose_error(T_target, self.forward(q))
        return float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))

    # ------------------------------------------------------- IK analytique -----
    def _rot03(self, th1: float, th2: float, th3: float) -> np.ndarray:
        T = np.eye(4)
        for i, th in enumerate((th1, th2, th3)):
            T = T @ self.dh_matrix(th, self.d[i], self.a[i], self.alpha[i])
        return T[:3, :3]

    def ik_analytic_all(self, T: np.ndarray, q_ref=None) -> List[Tuple[np.ndarray, str]]:
        """Toutes les solutions analytiques (jusqu'à 8), SANS filtrage par les limites.

        Découplage :
          1. centre du poignet  p_c = p - d6 · z6
          2. q1 : atan2(y, x) (épaule "face") ou +180° (épaule "dos")
          3. q2, q3 : problème plan à 2 segments (a2, a3), coude haut/bas
          4. q4, q5, q6 : à partir de R36 = R03ᵀ · R  (poignet normal / retourné)
        Retourne une liste de (q, étiquette) ; q en coordonnées articulaires (offset retiré).
        """
        d1, a2, a3, d6 = self.d[0], self.a[1], self.a[2], self.d[5]
        R, p = T[:3, :3], T[:3, 3]
        pc = p - d6 * R[:, 2]                      # centre du poignet
        x, y, h = pc[0], pc[1], pc[2] - d1
        r = float(np.hypot(x, y))
        th_ref = None if q_ref is None else np.asarray(q_ref, dtype=float) + self.off

        sols: List[Tuple[np.ndarray, str]] = []
        for flip in (False, True):
            if r < 1e-6:                            # singularité d'épaule : q1 indéterminé
                th1 = th_ref[0] if th_ref is not None else 0.0
                rho = 0.0
            else:
                th1 = float(np.arctan2(y, x))
                rho = r
                if flip:
                    th1, rho = wrap_angle(th1 + np.pi), -r
            c3 = (rho * rho + h * h - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if abs(c3) > 1.0 + 1e-9:
                continue                            # centre du poignet hors de portée
            c3 = float(np.clip(c3, -1.0, 1.0))
            for elbow in (1, -1):
                s3 = elbow * np.sqrt(1.0 - c3 * c3)
                th3 = float(np.arctan2(s3, c3))
                th2 = float(np.arctan2(h, rho) - np.arctan2(a3 * s3, a2 + a3 * c3))
                R36 = self._rot03(th1, th2, th3).T @ R
                c5 = float(np.clip(R36[2, 2], -1.0, 1.0))
                s5_abs = float(np.sqrt(1.0 - c5 * c5))
                base_label = f"{'dos' if flip else 'face'}/coude{'+' if elbow > 0 else '-'}"
                if s5_abs < 1e-6:                   # singularité de poignet : q4 fixé à la référence
                    th4 = th_ref[3] if th_ref is not None else 0.0
                    if c5 > 0:
                        th5 = 0.0
                        th6 = wrap_angle(np.arctan2(R36[1, 0], R36[0, 0]) - th4)
                    else:
                        th5 = np.pi
                        th6 = wrap_angle(th4 - np.arctan2(-R36[1, 0], -R36[0, 0]))
                    th = np.array([th1, th2, th3, th4, th5, th6])
                    sols.append((wrap_angle(th - self.off), base_label + '/poignet singulier'))
                    continue
                for wflip in (1, -1):
                    s5 = wflip * s5_abs
                    th5 = float(np.arctan2(s5, c5))
                    th4 = float(np.arctan2(-R36[1, 2] / s5, -R36[0, 2] / s5))
                    th6 = float(np.arctan2(-R36[2, 1] / s5, R36[2, 0] / s5))
                    th = np.array([th1, th2, th3, th4, th5, th6])
                    sols.append((wrap_angle(th - self.off),
                                 base_label + ('/poignet+' if wflip > 0 else '/poignet-')))
        return sols

    # ------------------------------------------------------- IK numérique ------
    def _lm(self, T: np.ndarray, q_start, max_iter: int, tol_pos: float, tol_ori: float,
            w_ori: float) -> Tuple[np.ndarray, bool, float]:
        """Levenberg-Marquardt (moindres carrés amortis) avec projection sur les limites."""
        W = np.array([1.0, 1.0, 1.0, w_ori, w_ori, w_ori])
        q = self.clip_to_limits(q_start)
        e = self.pose_error(T, self.forward(q))
        cost = float((W * e) @ (W * e))
        lam = 1e-2
        for _ in range(max_iter):
            if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_ori:
                return q, True, cost
            J = W[:, None] * self.jacobian(q)
            g = J.T @ (W * e)
            dq = np.linalg.solve(J.T @ J + lam * np.eye(6), g)
            q_new = self.clip_to_limits(q + dq)
            e_new = self.pose_error(T, self.forward(q_new))
            cost_new = float((W * e_new) @ (W * e_new))
            if cost_new < cost:
                q, e, cost = q_new, e_new, cost_new
                lam = max(lam / 3.0, 1e-9)
            else:
                lam *= 4.0
                if lam > 1e8:
                    break
        ok = np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_ori
        return q, bool(ok), cost

    def ik_numeric(self, T: np.ndarray, q0, max_iter: int = 200, tol_pos: float = 1e-5,
                   tol_ori: float = 1e-4, restarts: int = 6, seed: int = 0,
                   w_ori: float = 0.5) -> Tuple[Optional[np.ndarray], bool]:
        """IK numérique (Levenberg-Marquardt) depuis q0, puis départs aléatoires si échec."""
        rng = np.random.default_rng(seed)
        starts = [np.asarray(q0, dtype=float)]
        starts += [rng.uniform(self.q_min, self.q_max) for _ in range(restarts)]
        best_q, best_cost = None, np.inf
        for qs in starts:
            q, ok, cost = self._lm(T, qs, max_iter, tol_pos, tol_ori, w_ori)
            if ok:
                return q, True
            if cost < best_cost:
                best_q, best_cost = q, cost
        return best_q, False

    # ------------------------------------------------------- IK "haut niveau" --
    def solve(self, T: np.ndarray, q_ref=None, allow_numeric: bool = True,
              tol_pos: float = 1e-4, tol_ori: float = 1e-3) -> IKResult:
        """Cinématique inverse complète.

        1. solutions analytiques -> on garde celles qui respectent les limites et
           reproduisent la pose (garde-fou contre un modèle non conforme) ;
        2. on choisit la plus proche de q_ref (somme des écarts angulaires) ;
        3. sinon, repli numérique (Levenberg-Marquardt) ;
        4. sinon, message d'échec explicite (hors de portée / limites articulaires).
        """
        q_ref = np.asarray(self.cfg.rest if q_ref is None else q_ref, dtype=float)
        analytic = self.ik_analytic_all(T, q_ref)
        valid, out_of_limits = [], []
        for q, label in analytic:
            pe, oe = self.fk_error(T, q)
            if pe > tol_pos or oe > tol_ori:
                continue
            ok, bad = self.within_limits(q)
            (valid if ok else out_of_limits).append((q, label, bad))

        if valid:
            q, label, _ = min(valid, key=lambda s: float(np.sum(np.abs(angle_diff(s[0], q_ref)))))
            pe, oe = self.fk_error(T, q)
            sing = self.singularity_info(q)
            return IKResult(True, q, 'analytical', f'Solution analytique ({label})', pe, oe,
                            sing.near, sing.kinds, len(valid))

        if allow_numeric:
            q, ok = self.ik_numeric(T, q_ref)
            if ok and q is not None:
                pe, oe = self.fk_error(T, q)
                sing = self.singularity_info(q)
                return IKResult(True, q, 'numerical', 'Solution numérique (repli LM)', pe, oe,
                                sing.near, sing.kinds, 0)

        # ---- échec : on explique pourquoi
        pc = T[:3, 3] - self.d[5] * T[:3, :3][:, 2]
        dist = float(np.linalg.norm(pc - np.array([0.0, 0.0, self.d[0]])))
        a2, a3 = self.a[1], self.a[2]
        if dist > a2 + a3 + 1e-9:
            msg = (f'Pose inatteignable : centre du poignet à {dist:.3f} m de l\'épaule, '
                   f'portée max {a2 + a3:.3f} m')
        elif dist < abs(a2 - a3) - 1e-9:
            msg = 'Pose inatteignable : cible trop proche de l\'épaule'
        elif out_of_limits:
            names = sorted({self.cfg.joint_names[i] for _, _, bad in out_of_limits for i in bad})
            msg = 'Pose atteignable géométriquement mais hors limites articulaires : ' + ', '.join(names)
        else:
            msg = 'Aucune solution trouvée (pose inatteignable)'
        return IKResult(False, None, 'none', msg, n_solutions=0)
