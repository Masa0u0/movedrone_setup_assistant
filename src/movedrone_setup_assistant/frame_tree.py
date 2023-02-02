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

        # self.main.hoge.connect(self.on_urdf_loaded)  # TODO: URDFが正しく読み込まれた時の処理

    def on_urdf_loaded() -> None:
        pass  # TODO: cf. https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
