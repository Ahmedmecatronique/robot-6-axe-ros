#!/usr/bin/env python3
"""Vérifie que l'URDF généré par xacro décrit EXACTEMENT le même robot que la cinématique DH du code
(repères link_1..link_6 et tool0, sur 200 configurations aléatoires).

À relancer après toute modification de robot.yaml ou de arm6.urdf.xacro :

    pip install xacro pyyaml numpy
    python tools/check_urdf_vs_dh.py
"""
import os
import sys
import tempfile
from xml.etree import ElementTree as ET

import numpy as np
import xacro

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.join(ROOT, 'ros2_ws', 'src')
sys.path.insert(0, os.path.join(SRC, 'robot_kinematics'))

from robot_kinematics.config import load_robot_config  # noqa: E402
from robot_kinematics.kinematics import ArmKinematics  # noqa: E402
from robot_kinematics.transforms import make_transform, rot_z, rpy_to_matrix  # noqa: E402

YAML_PATH = os.path.join(SRC, 'robot_description', 'config', 'robot.yaml')
XACRO_PATH = os.path.join(SRC, 'robot_description', 'urdf', 'arm6.urdf.xacro')


def main() -> int:
    text = open(XACRO_PATH, encoding='utf-8').read()
    text = text.replace('$(find robot_description)/config/robot.yaml', YAML_PATH.replace('\\', '/'))
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False, encoding='utf-8') as f:
        f.write(text)
        tmp = f.name
    try:
        root = ET.fromstring(xacro.process_file(tmp).toxml())
    finally:
        os.remove(tmp)

    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()] if o is not None else [0, 0, 0]
        joints[j.get('name')] = dict(type=j.get('type'), parent=j.find('parent').get('link'),
                                     child=j.find('child').get('link'),
                                     T=make_transform(rpy_to_matrix(*rpy), xyz))
    by_child = {j['child']: (n, j) for n, j in joints.items()}

    def link_pose(link, q):
        chain = []
        while link != 'base_link':
            name, j = by_child[link]
            chain.append((name, j))
            link = j['parent']
        T = np.eye(4)
        for name, j in reversed(chain):
            T = T @ j['T']
            if j['type'] == 'revolute':
                T = T @ make_transform(rot_z(q[int(name.split('_')[1]) - 1]), [0, 0, 0])
        return T

    arm = ArmKinematics(load_robot_config(YAML_PATH))
    rng = np.random.default_rng(11)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(arm.q_min, arm.q_max)
        frames = arm.link_frames(q)
        for i in range(6):
            worst = max(worst, float(np.abs(link_pose(f'link_{i + 1}', q) - frames[i]).max()))
        worst = max(worst, float(np.abs(link_pose('tool0', q) - arm.forward(q)).max()))
    print(f'écart max URDF vs DH : {worst:.2e}')
    if worst > 1e-9:
        print('ÉCHEC : l\'URDF et le modèle DH divergent')
        return 1
    print('OK : URDF == DH')
    return 0


if __name__ == '__main__':
    sys.exit(main())
