from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtWidgets import QTreeWidget, QTreeWidgetItem


class FrameTreeWidget(QTreeWidget):

    WIDTH = 200

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self.main = main

        self.setFixedWidth(self.WIDTH)
        self.setColumnCount(1)
        self.setHeaderLabels(["Frames"])

    def define_connections(self) -> None:
        self.main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    def _on_robot_model_updated(self) -> None:
        pass  # TODO: cf. https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
