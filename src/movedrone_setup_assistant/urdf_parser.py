from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from urdf_parser_py.urdf import Robot, Link, Joint  # https://github.com/ros/urdf_parser_py


class URDFParser(QWidget):

    robot_model_updated = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self.main = main

        self.robot = Robot()

    def define_connections(self) -> None:
        self.main.settings.start.robot_model_loader.urdf_loaded.connect(self._on_urdf_loaded)

    @pyqtSlot()
    def _on_urdf_loaded(self) -> None:
        self.robot = Robot.from_parameter_server("/robot_description")
        self.robot_model_updated.emit()

    def get_links(self) -> List[Link]:
        return self.robot.links

    def get_joints(self) -> List[Joint]:
        return self.robot.joints

    def get_root(self) -> Link:
        root_name = self.robot.get_root()
        return self.robot.link_map[root_name]

    def get_joint(self, link_name: str) -> Joint:
        joint_name, _ = self.robot.parent_map[link_name]
        return self.robot.joint_map[joint_name]

    def get_parent(self, link_name: str) -> Link:
        _, parent_name = self.robot.parent_map[link_name]
        return self.robot.link_map[parent_name]
