"""Génération de trajectoires articulaires synchronisées (sans dépendance ROS).

Deux profils :
  * 'quintic'   : polynôme de degré 5 (position/vitesse/accélération continues, jerk borné).
                  Accepte une vitesse initiale non nulle (enchaînement sans à-coup).
  * 'trapezoid' : profil trapézoïdal (ou triangulaire) synchronisé sur les 6 axes ;
                  départ à l'arrêt uniquement (sinon repli sur 'quintic').

Tous les axes partagent la même durée : le plus "difficile" impose le temps, les autres
sont ralentis (les articulations arrivent ensemble).
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

EPS = 1e-9


class QuinticSegment:
    """Polynôme de degré 5 de (q0, v0, a0 = 0) vers (q1, 0, 0) en une durée T."""

    def __init__(self, q0, v0, q1, T: float):
        q0, v0, q1 = (np.asarray(x, dtype=float) for x in (q0, v0, q1))
        self.T = float(T)
        self.q0, self.q1 = q0, q1
        h = q1 - q0
        # conditions : q(0)=q0, q'(0)=v0, q''(0)=0 ; q(T)=q1, q'(T)=0, q''(T)=0
        self.c0 = q0
        self.c1 = v0
        self.c2 = np.zeros_like(q0)
        self.c3 = (20.0 * h - 12.0 * v0 * T) / (2.0 * T ** 3)
        self.c4 = (-30.0 * h + 16.0 * v0 * T) / (2.0 * T ** 4)
        self.c5 = (12.0 * h - 6.0 * v0 * T) / (2.0 * T ** 5)

    @property
    def duration(self) -> float:
        return self.T

    def sample(self, t: float):
        """Retourne (q, v, a) à l'instant t (saturé à [0, T])."""
        t = min(max(t, 0.0), self.T)
        q = self.c0 + self.c1 * t + self.c2 * t ** 2 + self.c3 * t ** 3 + self.c4 * t ** 4 + self.c5 * t ** 5
        v = self.c1 + 2 * self.c2 * t + 3 * self.c3 * t ** 2 + 4 * self.c4 * t ** 3 + 5 * self.c5 * t ** 4
        a = 2 * self.c2 + 6 * self.c3 * t + 12 * self.c4 * t ** 2 + 20 * self.c5 * t ** 3
        if t >= self.T:
            q, v, a = self.q1.copy(), np.zeros_like(v), np.zeros_like(a)
        return q, v, a


class TrapezoidSegment:
    """Profil trapézoïdal synchronisé, du repos au repos."""

    def __init__(self, q0, q1, v_max, a_max):
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        v_max, a_max = np.asarray(v_max, dtype=float), np.asarray(a_max, dtype=float)
        self.q0, self.q1 = q0, q1
        delta = q1 - q0
        moving = np.abs(delta) > EPS
        if not np.any(moving):
            self.T = 0.0
            self.vp = self.an = self.ta = 0.0
            return
        # Profil normalisé s(t) de 0 à 1 : vitesse et accélération max imposées par l'axe limitant
        vn = float(np.min(v_max[moving] / np.abs(delta[moving])))
        an = float(np.min(a_max[moving] / np.abs(delta[moving])))
        self.vp = min(vn, np.sqrt(an))   # vitesse de croisière (triangle si sqrt(an) < vn)
        self.an = an
        self.ta = self.vp / an
        self.T = 1.0 / self.vp + self.ta

    @property
    def duration(self) -> float:
        return self.T

    def sample(self, t: float):
        delta = self.q1 - self.q0
        if self.T <= 0.0 or t >= self.T:
            return self.q1.copy(), np.zeros_like(delta), np.zeros_like(delta)
        t = max(t, 0.0)
        if t < self.ta:                       # accélération
            s, ds, dds = 0.5 * self.an * t * t, self.an * t, self.an
        elif t < self.T - self.ta:            # croisière
            s = 0.5 * self.an * self.ta ** 2 + self.vp * (t - self.ta)
            ds, dds = self.vp, 0.0
        else:                                 # décélération
            tr = self.T - t
            s, ds, dds = 1.0 - 0.5 * self.an * tr * tr, self.an * tr, -self.an
        return self.q0 + s * delta, ds * delta, dds * delta


def _quintic_fits(seg: QuinticSegment, v_max, a_max, samples: int = 120) -> Tuple[bool, float]:
    """Vérifie par échantillonnage que le segment respecte v_max/a_max ; retourne (ok, ratio)."""
    ts = np.linspace(0.0, seg.T, samples)
    ratio = 0.0
    for t in ts:
        _, v, a = seg.sample(t)
        ratio = max(ratio, float(np.max(np.abs(v) / v_max)), float(np.sqrt(np.max(np.abs(a) / a_max))))
    return ratio <= 1.0 + 1e-6, ratio


def plan_segment(q0, q1, v_max, a_max, v0: Optional[np.ndarray] = None,
                 profile: str = 'quintic', min_duration: float = 0.0):
    """Planifie un segment synchronisé de q0 vers q1 (arrêt en q1).

    v_max, a_max : limites par axe (déjà multipliées par le facteur de vitesse voulu).
    Retourne un segment (avec .duration et .sample(t)) ou None si aucun mouvement n'est nécessaire.
    """
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    v_max, a_max = np.asarray(v_max, dtype=float), np.asarray(a_max, dtype=float)
    v0 = np.zeros_like(q0) if v0 is None else np.asarray(v0, dtype=float)
    delta = np.abs(q1 - q0)
    at_rest = bool(np.all(np.abs(v0) < 1e-6))

    if float(np.max(delta)) < 1e-6 and at_rest:
        return None

    if profile == 'trapezoid' and at_rest:
        seg = TrapezoidSegment(q0, q1, v_max, a_max)
        if seg.duration < min_duration and seg.duration > 0.0:
            # on étire uniquement en réduisant les limites (le profil reste valide)
            k = seg.duration / min_duration
            seg = TrapezoidSegment(q0, q1, v_max * k, a_max * k * k)
        return seg

    # Quintique : durée initiale estimée du repos au repos, puis élargie jusqu'à respecter les limites
    T = float(np.max(np.maximum(1.875 * delta / v_max, np.sqrt(5.7735 * delta / a_max))))
    T = max(T, min_duration, 0.02)
    for _ in range(60):
        seg = QuinticSegment(q0, v0, q1, T)
        ok, ratio = _quintic_fits(seg, v_max, a_max)
        if ok:
            return seg
        T *= max(1.05, min(ratio, 2.0))
    return seg
