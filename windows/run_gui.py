"""Lanceur de l'IHM sous Windows (sans ROS 2) : prépare le chemin Python puis démarre robot_gui.main_web.

    python windows\\run_gui.py --host 192.168.1.50
    python windows\\run_gui.py                 # demande l'adresse au démarrage

Le fichier de configuration du robot (robot.yaml) est lu directement dans le dossier du projet :
modifier ce fichier suffit, il n'y a pas de « colcon build » côté PC.
"""
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.join(ROOT, 'ros2_ws', 'src')

for pkg in ('robot_kinematics', 'robot_gui'):
    sys.path.insert(0, os.path.join(SRC, pkg))
os.environ.setdefault('ROBOT_CONFIG', os.path.join(SRC, 'robot_description', 'config', 'robot.yaml'))

from robot_gui.main_web import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
