import os.path as osp
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QIcon

from .robot_structure import RobotStructureWidget
from .settings import SettingsWidget

from .utils import get_pkg_path


class SetupAssistant(QWidget):

    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("MoveDrone Setup Assistant")

        pkg_path = get_pkg_path()
        icon_path = osp.join(pkg_path, 'resources/movedrone_icon.png')
        self.setWindowIcon(QIcon(icon_path))

        self.robot_structure = RobotStructureWidget(self)
        self.robot_structure.setVisible(False)

        self.settings = SettingsWidget(self)

        self.rows = QVBoxLayout()
        self.rows.addWidget(self.robot_structure)
        self.rows.addWidget(self.settings)
        self.setLayout(self.rows)
