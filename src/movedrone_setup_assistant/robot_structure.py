from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtWidgets import QWidget, QHBoxLayout

from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget


class RobotStructureWidget(QWidget):
    
    HEIGHT = 350

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self.main = main

        self.setFixedHeight(self.HEIGHT)

        self.frame_tree = FrameTreeWidget(main)
        self.rviz = RvizWidget(main)

        self.cols = QHBoxLayout()
        self.cols.addWidget(self.frame_tree)
        self.cols.addWidget(self.rviz)
        self.setLayout(self.cols)

        # self.setVisible(False)
