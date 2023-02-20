import rospy
import rospkg
from xml.etree import ElementTree as ET
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def get_proj_path() -> str:
    """ スクリプトのパッケージまでのフルパスを返す． """
    pkg_name = rospkg.get_package_name(__file__)
    return rospkg.RosPack().get_path(pkg_name)


def get_drone_name() -> str:
    """ URDFからドローンの名前を取得する． """
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"
    return root.get("name")


class SpinBox(QSpinBox):
    """ QSpineBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class DoubleSpinBox(QDoubleSpinBox):
    """ QDoubleSpineBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class ComboBox(QComboBox):
    """ QComboBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()
