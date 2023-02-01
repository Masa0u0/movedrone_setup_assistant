#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication

from movedrone_setup_assistant import SetupAssistant


if __name__ == '__main__':
    app = QApplication(sys.argv)

    setup_assistant = SetupAssistant()
    setup_assistant.show()

    sys.exit(app.exec())
