import os.path as osp
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QIcon


from .urdf_parser import URDFParser
from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget
from .settings import SettingsWidget
from .utils import get_pkg_path


class SetupAssistant(QWidget):

    TITLE = "MoveDrone Setup Assistant"

    def __init__(self) -> None:
        super().__init__()

        pkg_path = get_pkg_path()
        icon_path = osp.join(pkg_path, 'resources/movedrone_icon.png')  # TODO
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(self.TITLE)

        self.robot = URDFParser(self)
        self.frame_tree = FrameTreeWidget(self)
        self.rviz = RvizWidget(self)
        self.settings = SettingsWidget(self)

        # レイアウト
        self.rows = QVBoxLayout()
        self.cols = QHBoxLayout()
        self.cols.addWidget(self.frame_tree)
        self.cols.addWidget(self.rviz)
        self.rows.addLayout(self.cols)
        self.rows.addWidget(self.settings)
        self.setLayout(self.rows)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()
    
    def define_connections(self) -> None:
        self.robot.define_connections()
        self.frame_tree.define_connections()
        self.rviz.define_connections()
        self.settings.define_connections()
