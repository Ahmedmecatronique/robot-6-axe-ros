"""Point d'entrée de l'IHM :  ros2 run robot_gui gui   (à lancer sur le PC portable, pas sur le Pi)."""
import sys

import rclpy
from pyqtgraph.Qt import QtWidgets

from . import theme
from .main_window import MainWindow
from .ros_bridge import RosBridge


def main(args=None):
    rclpy.init(args=args)
    bridge = RosBridge()
    bridge.start()                                # rclpy tourne dans son propre thread

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)
    win = MainWindow(bridge)
    win.resize(1500, 950)
    win.show()
    code = app.exec() if hasattr(app, 'exec') else app.exec_()

    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
