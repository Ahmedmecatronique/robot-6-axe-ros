"""Point d'entrée de l'IHM pour WINDOWS (ou toute machine SANS ROS 2) : connexion par rosbridge.

    python -m robot_gui.main_web --host 192.168.1.50          # IP du Raspberry Pi (ou du PC de simulation)
    python -m robot_gui.main_web                              # demande l'adresse au démarrage

Voir windows/README_WINDOWS.md.
"""
import argparse
import os
import sys

from pyqtgraph.Qt import QtCore, QtWidgets

from . import theme
from .main_window import MainWindow
from .demo_bridge import DemoBridge
from .web_bridge import WebBridge


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=os.environ.get('ROBOT_HOST', ''), help='adresse IP du robot (rosbridge)')
    ap.add_argument('--port', type=int, default=int(os.environ.get('ROBOT_PORT', '9090')))
    ap.add_argument('--rate', type=float, default=50.0,
                    help='débit des flux rapides reçus (Hz, 0 = tous) ; 50 par défaut, plus léger pour le PC')
    ap.add_argument('--demo', action='store_true', help='robot simulé dans le processus (aucun réseau, aucun robot)')
    ap.add_argument('--demo-unhomed', action='store_true', help='démo : démarre désactivé (tester Activer + Homing)')
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)
    settings = QtCore.QSettings('robot6axes', 'gui')

    if args.demo or args.demo_unhomed:
        bridge = DemoBridge(start_ready=not args.demo_unhomed)
        win = MainWindow(bridge)
        win.setWindowTitle('Bras 6 axes — MODE DÉMO (robot simulé localement)')
        win.show()
        code = app.exec() if hasattr(app, 'exec') else app.exec_()
        bridge.shutdown()
        return code

    host = args.host
    if not host:
        host, ok = QtWidgets.QInputDialog.getText(
            None, 'Connexion au robot', 'Adresse IP du robot (Raspberry Pi ou simulation) :',
            text=str(settings.value('last_host', '192.168.1.50')))
        if not ok or not host.strip():
            return 1
        host = host.strip()
    settings.setValue('last_host', host)

    bridge = WebBridge(host, args.port, args.rate)
    win = MainWindow(bridge)
    win.setWindowTitle(f'Bras 6 axes — {host}:{args.port}')
    win.show()
    code = app.exec() if hasattr(app, 'exec') else app.exec_()
    bridge.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
