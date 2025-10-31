#!/usr/bin/env python3
import rclpy
import os

from rclpy.node import Node

from .gui.main_window import WorkspaceManagerGUI
from ament_index_python import get_package_share_directory
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
import sys

class WorkspaceManagerNode(Node):
    def __init__(self):
        super().__init__('workspace_manager_node')
        self.get_logger().info('Workspace Manager Node started')

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    icon_path = os.path.join(get_package_share_directory("workspace_manager"), "icon", "icon.jpg")
    node = WorkspaceManagerNode()

    # Create and show GUI
    window = WorkspaceManagerGUI(node)
    window.setWindowIcon(QIcon(icon_path))
    window.show()


    # Run Qt event loop
    ret = app.exec_()

    # Cleanup
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
