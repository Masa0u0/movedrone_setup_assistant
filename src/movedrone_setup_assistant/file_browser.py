from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import rospkg
import os.path as osp
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QFileDialog, QMessageBox


class FileBrowserWidget(QWidget):

    description_load_requested = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super(QWidget, self).__init__()

        self.main = main
        self.description_path = None

        self.columns = QHBoxLayout()
        self.setLayout(self.columns)

        self.file_text = QLineEdit("")
        self.file_text.textChanged.connect(self.on_file_path_changed)
        self.columns.addWidget(self.file_text)

        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.on_browse_button_clicked)
        self.columns.addWidget(self.browse_button)

        self.load_button = QPushButton("Load")
        self.load_button.clicked.connect(self.on_load_button_clicked)
        self.load_button.setEnabled(False)
        self.columns.addWidget(self.load_button)

    def on_file_path_changed(self) -> None:
        file_path = self.file_text.text().strip()

        if not self.is_valid_extension(file_path):
            self.load_button.setEnabled(False)
            return

        self.load_button.setEnabled(True)
        self.file_text.setText(file_path)

        pkg_name = rospkg.get_package_name(file_path)

        if pkg_name == None:
            self.description_path = file_path
            return

        try:
            start = file_path.index(pkg_name) + len(pkg_name)
            file_name = osp.basename(file_path)
            end = file_path.index(file_name)
            file_sub_path = file_path[start:end]
            self.description_path = f'$(find {pkg_name}){file_sub_path}{file_name}'
        except:
            pass

    def on_browse_button_clicked(self) -> None:
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self, self.main.TITLE, "", "URDF(*.urdf);;XACRO(*.xacro)", options=options
        )

        if self.is_valid_extension(file_path):
            self.file_text.setText(file_path)
            self.load_button.setEnabled(True)
        elif file_path == "":
            pass
        else:
            QMessageBox.information(self, "ERROR", "Invalid file path: " + "\r\n" + file_path)

    def on_load_button_clicked(self) -> None:
        self.file_text.setEnabled(False)
        self.browse_button.setEnabled(False)
        self.load_button.setEnabled(False)
        self.description_load_requested.emit()

    def is_valid_extension(self, file_path: str) -> bool:
        _, extension = osp.splitext(file_path)

        if extension in {'.urdf', '.xacro'}:
            return osp.isfile(file_path)
        else:
            return False
