from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import rospy
from typing import List, Tuple
from urdf_parser_py.urdf import Robot, Link, Joint  # https://github.com/ros/urdf_parser_py
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class URDFParser(QWidget):

    robot_model_updated = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._robot = Robot()

    def define_connections(self) -> None:
        self._main.settings.start.robot_model_loader.urdf_loaded.connect(self._on_urdf_loaded)

    @pyqtSlot()
    def _on_urdf_loaded(self) -> None:
        self._robot = Robot.from_parameter_server("/robot_description")
        rospy.loginfo("Robot model is loaded successfully.")
        self.robot_model_updated.emit()

    def get_links(self) -> List[Link]:
        return self._robot.links

    def get_joints(self) -> List[Joint]:
        return self._robot.joints

    def get_root(self) -> Link:
        root_name = self._robot.get_root()
        return self._robot.link_map[root_name]

    def get_link(self, link_name: str) -> Link:
        return self._robot.link_map[link_name]

    def get_joint(self, link_name: str) -> Joint:
        joint_name, _ = self._robot.parent_map[link_name]
        return self._robot.joint_map[joint_name]

    def get_parent(self, link_name: str) -> Link:
        _, parent_name = self._robot.parent_map[link_name]
        return self._robot.link_map[parent_name]

    def get_children(self, link_name: str) -> List[Tuple[str, str]]:
        return self._robot.child_map[link_name]

    def is_end_link(self, link_name: str) -> bool:
        assert link_name in self._robot.link_map.keys()
        return link_name not in self._robot.child_map.keys()

    def link_exists(self, link_name: str) -> bool:
        for link in self._robot.links:
            if link.name == link_name:
                return True
        return False

    def joint_exists(self, joint_name: str) -> bool:
        for joint in self._robot.joints:
            if joint.name == joint_name:
                return True
        return False

    def get_fixed_link_names(self) -> List[str]:
        """ ルートリンクに固定されているリンクの名前の配列を返す． """
        root = self.get_root()
        return self._get_fixed_link_names_rec(root.name)

    def _get_fixed_link_names_rec(self, parent_name: str) -> List[str]:
        """ parent以下の固定リンクの名前の配列を返す． """
        res = [parent_name]

        if self.is_end_link(parent_name):
            return res

        for _, child_name in self.get_children(parent_name):
            joint = self.get_joint(child_name)
            if joint.type != "fixed":
                continue
            res += self._get_fixed_link_names_rec(child_name)
        return res
