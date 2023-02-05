import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .urdf_parser import URDFParser
from .robot_visualizer import RobotVisualizerWidget
from .settings import SettingsWidget
from .utils import get_pkg_path
from .const import *


class SetupAssistant(QWidget):

    def __init__(self) -> None:
        super().__init__()

        pkg_path = get_pkg_path()
        icon_path = osp.join(pkg_path, 'resources/movedrone_icon.png')  # TODO
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        self.urdf_parser = URDFParser(self)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        # 高さを指定するために，単なる横並びのレイアウトもウィジェットとして定義している
        self.robot_visualizer = RobotVisualizerWidget(self)
        self._rows.addWidget(self.robot_visualizer)

        self.settings = SettingsWidget(self)
        self._rows.addWidget(self.settings)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        self.urdf_parser.define_connections()
        self.robot_visualizer.define_connections()
        self.settings.define_connections()
