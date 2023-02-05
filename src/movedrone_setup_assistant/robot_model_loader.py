from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os
import os.path as osp
import rospy
import rospkg
import roslaunch
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .utils import get_pkg_path
from .const import *


class RobotModelLoaderWidget(QWidget):

    WAIT_UNTIL_URDF_LOADED = 5.

    urdf_loaded = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self.main = main
        self.description_path = None

        description_loader_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        description_launch_path = osp.join(get_pkg_path(), "launch/description.launch")
        roslaunch.configure_logging(description_loader_uuid)
        self.description_launcher = roslaunch.parent.ROSLaunchParent(
            description_loader_uuid, [description_launch_path]
        )

        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        label = QLabel("Description path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        self.rows.addWidget(label)

        instruction_text = "TODO: instruction"
        instruction = QLabel(instruction_text)
        instruction.setFont(QFont("Default", pointSize=BODY_PSIZE))
        instruction.setAlignment(Qt.AlignTop)
        self.rows.addWidget(instruction)

        self.columns = QHBoxLayout()
        self.rows.addLayout(self.columns)

        self.file_text = QLineEdit("")
        self.columns.addWidget(self.file_text)

        self.browse_button = QPushButton("Browse")
        self.columns.addWidget(self.browse_button)

        self.load_button = QPushButton("Load")
        self.load_button.setEnabled(False)
        self.columns.addWidget(self.load_button)

    def define_connections(self) -> None:
        self.file_text.textChanged.connect(self._on_file_path_changed)
        self.browse_button.clicked.connect(self._on_browse_button_clicked)
        self.load_button.clicked.connect(self._on_load_button_clicked)

    @pyqtSlot()
    def _on_file_path_changed(self) -> None:
        file_path = self.file_text.text().strip()

        if not self._is_valid_extension(file_path):
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

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self, self.main.TITLE, "", "Robot Description (*.urdf *xacro);;All (*)", options=options
        )

        if self._is_valid_extension(file_path):
            self.file_text.setText(file_path)
            self.load_button.setEnabled(True)
        elif file_path == "":
            pass
        else:
            QMessageBox.information(self, "ERROR", "Invalid file path: " + "\r\n" + file_path)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        self._launch_file()

        self.file_text.setEnabled(False)
        self.browse_button.setEnabled(False)
        self.load_button.setEnabled(False)

        ok = self._wait_until_urdf_ready()
        if ok:
            self.urdf_loaded.emit()
        else:
            rospy.logerr('Failed to load robot description.')
            self.file_text.clear()
            self.file_text.setEnabled(True)
            self.browse_button.setEnabled(True)
            self.load_button.setEnabled(True)

    def _launch_file(self) -> None:
        # this is a hack to pass urdf file
        # description.launchで使われる環境変数を設定
        print(self.description_path)
        os.environ["MOVEDRONE_SETUP_ASSISTANT_DESCRIPTION_PATH"] = self.description_path

        # robot_descriptionをrosparamに登録
        self.description_launcher.shutdown()
        self.description_launcher.start()

    def _is_valid_extension(self, file_path: str) -> bool:
        _, extension = osp.splitext(file_path)

        if extension in {'.urdf', '.xacro'}:
            return osp.isfile(file_path)
        else:
            return False

    def _wait_until_urdf_ready(self) -> bool:
        """ 制限時間内にdescriptionがパラメータサーバに登録されたらTrue． """
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            try:
                rospy.get_param('/robot_description')
                return True
            except:
                pass

            now = rospy.Time.now()
            elapsed_time = (now - start).to_sec()
            if elapsed_time > self.WAIT_UNTIL_URDF_LOADED:
                return False

            rospy.sleep(0.1)
