"""Chargement de la configuration du robot depuis robot_description/config/robot.yaml.

Ce module ne dépend pas de ROS : il est utilisable dans les tests et les scripts.
Toutes les grandeurs angulaires sont converties en RADIANS ; les longueurs en mètres.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
import yaml

# Angles alpha_i de la structure anthropomorphe + poignet sphérique (DH standard).
# Ils font partie de la topologie du robot, pas des paramètres à ajuster.
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])


@dataclass
class RobotConfig:
    """Paramètres géométriques et limites du bras."""

    joint_names: list
    d: np.ndarray             # (6,) décalages le long de z_(i-1)  [m]
    a: np.ndarray             # (6,) longueurs le long de x_i      [m]
    alpha: np.ndarray         # (6,) torsions                      [rad]
    theta_offset: np.ndarray  # (6,) theta_i = q_i + offset_i      [rad]
    q_min: np.ndarray         # (6,) limites basses                [rad]
    q_max: np.ndarray         # (6,) limites hautes                [rad]
    v_max: np.ndarray         # (6,) vitesses max                  [rad/s]
    a_max: np.ndarray         # (6,) accélérations max             [rad/s²]
    rest: np.ndarray          # (6,) pose de repos                 [rad]
    z_min: float              # plan de sécurité (table)           [m]
    base_frame: str
    tool_frame: str
    raw: dict                 # dictionnaire YAML brut (section hardware, etc.)

    @property
    def reach(self) -> float:
        """Portée maximale approximative (épaule -> centre du poignet)."""
        return float(self.a[1] + self.a[2])


def find_config_path(path: Optional[str] = None) -> str:
    """Retourne le chemin de robot.yaml.

    Ordre de recherche : argument explicite, variable ROBOT_CONFIG, paquet installé
    (share de robot_description), puis arborescence des sources (tests hors ROS).
    """
    candidates = []
    if path:
        candidates.append(path)
    if os.environ.get('ROBOT_CONFIG'):
        candidates.append(os.environ['ROBOT_CONFIG'])
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('robot_description'), 'config', 'robot.yaml'))
    except Exception:  # ROS absent ou paquet non installé
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.normpath(os.path.join(
        here, '..', '..', 'robot_description', 'config', 'robot.yaml')))
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    raise FileNotFoundError('robot.yaml introuvable. Essayé : ' + ', '.join(map(str, candidates)))


def load_robot_config(path: Optional[str] = None) -> RobotConfig:
    """Lit robot.yaml et retourne un RobotConfig (unités SI)."""
    with open(find_config_path(path), 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    rb = raw['robot']
    dh = rb['dh']
    lim = rb['limits']
    deg = np.deg2rad
    return RobotConfig(
        joint_names=list(rb['joint_names']),
        d=np.array([dh['d1'], 0.0, 0.0, 0.0, 0.0, dh['d6']], dtype=float),
        a=np.array([0.0, dh['a2'], dh['a3'], 0.0, 0.0, 0.0], dtype=float),
        alpha=DH_ALPHA.copy(),
        theta_offset=deg(np.array(rb['theta_offset_deg'], dtype=float)),
        q_min=deg(np.array(lim['position_min_deg'], dtype=float)),
        q_max=deg(np.array(lim['position_max_deg'], dtype=float)),
        v_max=deg(np.array(lim['velocity_max_deg_s'], dtype=float)),
        a_max=deg(np.array(lim['acceleration_max_deg_s2'], dtype=float)),
        rest=deg(np.array(rb['poses']['rest_deg'], dtype=float)),
        z_min=float(rb.get('workspace', {}).get('z_min', 0.0)),
        base_frame=rb.get('base_frame', 'base_link'),
        tool_frame=rb.get('tool_frame', 'tool0'),
        raw=raw,
    )
