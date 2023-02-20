from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os.path as osp
import re
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..const import *


class RosPackageWidget(BaseSettingWidget):

    TEXT_HEIGHT = 50
    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Generate ROS Package'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        pardir_description = "TODO: description"
        self.pardir = ParamGetterWidget_DirDialog("Parent Directory", pardir_description)
        self._rows.addWidget(self.pardir)

        pkg_name_description = "TODO: description"
        self.pkg_name = ParamGetterWidget_LineEdit("Package Name", pkg_name_description)
        self._rows.addWidget(self.pkg_name)

        text = QLabel("The package will be generated as")
        text.setFont(QFont("Default", pointSize=BODY_PSIZE))
        text.setFixedHeight(self.TEXT_HEIGHT)
        self.setAlignment(Qt.AlignTop)
        self._rows.addWidget(text)

        self.pkg_path = PackagePath(main)
        self._rows.addWidget(self.pkg_path)

        # ボタンを中央に配置するためにLayoutとWidgetを噛ませる必要がある
        self.generate_button = QPushButton("Generate")
        self.generate_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self.generate_button.setEnabled(False)
        button_widget = QWidget()
        button_layout = QVBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)  # この操作のためにLayoutが必要
        button_widget.setLayout(button_layout)
        button_layout.addWidget(self.generate_button)
        self._rows.addWidget(button_widget)  # この操作のためにWidgetが必要

        self._add_dummy_widget()

    def define_connections(self) -> None:
        super().define_connections()
        self.pkg_path.define_connections()
        self.pkg_path.path_changed.connect(self._on_path_changed)

    @pyqtSlot(str, str)
    def _on_path_changed(self, pardir: str, pkg_name: str) -> None:
        if not self._pardir_is_valid(pardir):
            self.generate_button.setEnabled(False)
            return

        if not self._pkg_name_is_valid(pkg_name):
            self.generate_button.setEnabled(False)
            return

        self.generate_button.setEnabled(True)

    def _pardir_is_valid(self, pardir: str) -> bool:
        # 存在していないとダメ
        if not osp.isdir(pardir):
            return False

        # ルートはダメ
        if pardir == "/":
            return False

        return True

    def _pkg_name_is_valid(self, pkg_name: str) -> bool:
        # 既に存在していたらダメ
        if osp.exists(self.pkg_path.text()):
            return False

        # 空欄はダメ
        if len(pkg_name) == 0:
            return False

        # 複数階層はダメ
        if pkg_name.count("/") > 0:
            return False

        # スペースはダメ
        if pkg_name.count(" ") > 0:
            return False

        return True


class PackagePath(QLabel):

    HEIGHT = 50

    path_changed = pyqtSignal(str, str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._pardir = ""
        self._pkg_name = ""

        self.setFont(QFont("Default", pointSize=BODY_PSIZE, weight=QFont.Bold))
        self.setFixedHeight(self.HEIGHT)
        self.setAlignment(Qt.AlignTop)

        self._update()

    def define_connections(self) -> None:
        self._main.settings.ros_package.pardir.path_changed.connect(self._on_pardir_changed)
        self._main.settings.ros_package.pkg_name.text_changed.connect(self._on_pkg_name_changed)

    def _update(self) -> None:
        path = self._pardir + "/" + self._pkg_name
        path = re.sub("/*/", "/", path)  # スラッシュの重複を削除
        self.setText(path)

        self.path_changed.emit(self._pardir, self._pkg_name)

    @pyqtSlot(str)
    def _on_pardir_changed(self, pardir: str) -> None:
        self._pardir = pardir
        self._update()

    @pyqtSlot(str)
    def _on_pkg_name_changed(self, pkg_name: str) -> None:
        self._pkg_name = pkg_name
        self._update()
