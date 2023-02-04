from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os.path as osp
from rviz import bindings as rviz
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .utils import get_pkg_path


class RvizWidget(QWidget):

    MIN_WIDTH = 300

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self.main = main

        self.highlighted_link = None

        # Setup frame
        # cf. RViz Python Tutorial: https://docs.ros.org/en/indigo/api/rviz_python_tutorial/html/
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rviz_config_path = osp.join(get_pkg_path(), "config/setup_assistant.rviz")
        reader.readFile(config, rviz_config_path)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        self.frame.load(config)
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        # Setup robot_model_display
        manager = self.frame.getManager()
        self.robot_model_display = manager.getRootDisplayGroup().getDisplayAt(2)
        self.robot_model_display.setBool(False)

        # robot_model_displaのサブプロパティを取得
        self.link_highlighter = self.robot_model_display.subProp("Highlight Link")
        self.link_unhighlighter = self.robot_model_display.subProp("Unhighlight Link")

        # Layout
        self.rows = QVBoxLayout()
        self.rows.addWidget(self.frame)
        self.setLayout(self.rows)

        self.setMinimumWidth(self.MIN_WIDTH)

    def define_connections(self) -> None:
        self.main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    def highlight_link(self, link_name: str) -> None:
        if link_name == self.highlighted_link:
            return

        if self.highlighted_link != None:
            self.unhighlight_link(self.highlighted_link)

        self.link_highlighter.setValue(link_name)
        self.highlighted_link = link_name

    def unhighlight_link(self, link_name: str) -> None:
        self.link_unhighlighter.setValue(link_name)

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self.robot_model_display.setBool(True)
        self.frame.getManager().setFixedFrame(self.urdf_parser.base)
