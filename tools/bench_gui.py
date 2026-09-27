#!/usr/bin/env python3
"""Banc de mesure de l'IHM (côté PC) : coût CPU de la boucle d'affichage, sur le robot simulé du mode démo.

    python tools/bench_gui.py [--seconds 10] [--json résultat.json]

Mesure, à la cadence réelle (une itération toutes les 16 ms, le robot simulé publiant à 100 Hz) :
  * le temps de chaque itération de la boucle d'affichage (MainWindow._tick), moyenne / p95 / max ;
  * le temps de chaque composant (3D, courbes, tableau, caméra, instantané, historique) ;
  * la part de CPU consommée par le processus.

LIMITE : la mesure est faite « hors écran » : le dessin OpenGL par le GPU n'est pas exécuté. On mesure donc le
travail Python/Qt (celui qui bloque le fil d'interface), pas le temps de rendu GPU.
"""
import argparse
import json
import os
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.join(ROOT, 'ros2_ws', 'src')
sys.path[:0] = [os.path.join(SRC, 'robot_kinematics'), os.path.join(SRC, 'robot_gui')]

import numpy as np  # noqa: E402
from pyqtgraph.Qt import QtWidgets  # noqa: E402

from robot_gui import theme  # noqa: E402
from robot_gui.demo_bridge import DemoBridge  # noqa: E402
from robot_gui.main_window import MainWindow  # noqa: E402


def timed(store: dict, name: str, fn, *args):
    t = time.perf_counter()
    r = fn(*args)
    store.setdefault(name, []).append((time.perf_counter() - t) * 1e3)
    return r


def summarize(values):
    a = np.array(values)
    return {'moy_ms': float(a.mean()), 'p95_ms': float(np.percentile(a, 95)), 'max_ms': float(a.max())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--json', default='')
    args = ap.parse_args()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyleSheet(theme.STYLESHEET)
    bridge = DemoBridge()
    win = MainWindow(bridge)
    win.resize(1500, 1000)
    win.show()
    win.timer.stop()                        # on pilote la boucle nous-mêmes, à la cadence réelle

    # le robot bouge en continu pour que les tampons, la 3D et les courbes travaillent vraiment
    bridge.set_speed_scale(1.0)
    rng = np.random.default_rng(0)

    def new_goal():
        bridge.send_joint_goal([rng.uniform(bridge.cfg.q_min * 0.5, bridge.cfg.q_max * 0.5)])

    new_goal()
    time.sleep(1.0)                         # remplissage des tampons
    # tous les onglets sont mesurés « visibles » : pire cas (on peut n'en voir qu'un à la fois)
    parts, ticks = {}, []
    t_end = time.perf_counter() + args.seconds
    cpu0, wall0, n = time.process_time(), time.perf_counter(), 0
    while time.perf_counter() < t_end:
        t0 = time.perf_counter()
        snap = timed(parts, 'snapshot', bridge.snapshot)
        timed(parts, '3D (update_view)', win.view3d.update_view, snap)
        n += 1
        if n % 3 == 0:
            timed(parts, 'historique', bridge.history, 10.0)
            timed(parts, 'courbes (update_plots)', win.dashboard.update_plots)
        if n % 4 == 0:
            timed(parts, 'caméra (update_view)', win.camera.update_view, snap)
        if n % 6 == 0:
            timed(parts, 'tableau (update_table)', win.dashboard.update_table, snap)
            timed(parts, 'panneau (update_state)', win.panel.update_state, snap)
        timed(parts, 'événements Qt (processEvents)', app.processEvents)
        ticks.append((time.perf_counter() - t0) * 1e3)
        if n % 400 == 0:
            new_goal()
        time.sleep(max(0.0, 0.016 - (time.perf_counter() - t0)))
    wall, cpu = time.perf_counter() - wall0, time.process_time() - cpu0
    bridge.shutdown()

    result = {'iteration': summarize(ticks), 'composants': {k: summarize(v) for k, v in parts.items()},
              'cpu_pct_processus': 100.0 * cpu / wall, 'iterations': n, 'secondes': wall}
    it = result['iteration']
    print(f"Itérations : {n} en {wall:.1f} s  |  CPU processus : {result['cpu_pct_processus']:.1f} % d'un cœur")
    print(f"Boucle d'affichage : moy {it['moy_ms']:.2f} ms · p95 {it['p95_ms']:.2f} ms · max {it['max_ms']:.2f} ms "
          f"(budget 16 ms pour 60 images/s)")
    print(f"{'composant':34s} {'moy':>8s} {'p95':>8s} {'max':>8s}   (ms par appel)")
    for k, v in sorted(result['composants'].items(), key=lambda kv: -kv[1]['moy_ms']):
        print(f"{k:34s} {v['moy_ms']:8.3f} {v['p95_ms']:8.3f} {v['max_ms']:8.3f}")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
