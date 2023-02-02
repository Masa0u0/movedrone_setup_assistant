#!/usr/bin/env python3

import os.path as osp
import sys
import rospy
from PyQt5.QtWidgets import QApplication

from movedrone_setup_assistant import SetupAssistant


if __name__ == '__main__':
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    setup_assistant = SetupAssistant()
    setup_assistant.show()

    sys.exit(app.exec())
