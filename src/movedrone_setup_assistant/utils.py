import rospkg

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def get_pkg_path() -> str:
    """ スクリプトのパッケージまでのフルパスを返す． """
    pkg_name = rospkg.get_package_name(__file__)
    return rospkg.RosPack().get_path(pkg_name)


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
