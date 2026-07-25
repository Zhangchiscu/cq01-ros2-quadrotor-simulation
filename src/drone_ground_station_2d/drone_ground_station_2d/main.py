from __future__ import annotations

import signal
import sys
import threading

import rclpy
from PyQt5.QtWidgets import QApplication

from .main_window import MainWindow
from .ros_bridge import GroundStationBridge


def main(args=None) -> None:
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rclpy.init(args=args)
    node = GroundStationBridge()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    app = QApplication(sys.argv)
    app.setApplicationName('Drone Ground Station 2D')
    window = MainWindow(node)
    window.show()
    exit_code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    thread.join(timeout=2.0)
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
