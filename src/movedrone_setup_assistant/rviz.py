from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os
import os.path as osp
import roslaunch
from rviz import bindings as rviz
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from .utils import get_pkg_path


class RvizWidget(QWidget):

    urdf_loaded = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()

        self.main = main

        self.setFixedHeight(350)  # ウィンドウサイズを変えてもRvizの縦幅は不変

        pkg_path = get_pkg_path()
        rviz_config_path = osp.join(pkg_path, "config/setup_assistant.rviz")
        description_launch_path = osp.join(pkg_path, "/launch/description.launch")

        # TODO: Loadボタンが押された時のコールバック
        # self.main.hogehoge.connect(self.on_urdf_path_load)

        # cf. RViz Python Tutorial: https://docs.ros.org/en/indigo/api/rviz_python_tutorial/html/
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, rviz_config_path)

        self.frame.load(config)
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()

        self.robot_model_display = self.manager.getRootDisplayGroup().getDisplayAt(2)
        self.robot_model_display.setBool(False)

        # sub-property(サブクラス)を取得
        self.link_highlighter = self.robot_model_display.subProp("Highlight Link")
        self.link_unhighlighter = self.robot_model_display.subProp("Unhighlight Link")

        self.rows = QVBoxLayout()
        self.rows.addWidget(self.frame)
        self.setLayout(self.rows)

        self.highlighted_link = None

        self.description_launch_args = [description_launch_path, "description_file:foo"]

        description_loader_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(description_loader_uuid)
        self.description_launcher = roslaunch.parent.ROSLaunchParent(
            description_loader_uuid,
            [self.description_launch_args[0]],
        )

    def highlight_link(self, link_name: str) -> None:
        if link_name == self.highlighted_link:
            return

        if self.highlighted_link != None:
            self.unhighlight_link(self.highlighted_link)

        self.link_highlighter.setValue(link_name)
        self.highlighted_link = link_name

    def unhighlight_link(self, link_name: str) -> None:
        self.link_unhighlighter.setValue(link_name)

    def update_urdf_file(self, description_path: str) -> None:
        path_arg = "description_file:" + description_path
        self.description_launch_args[1] = path_arg

    def launch_file(self, description_path):
        # this is a hack to pass urdf file
        # description.launchで使われる環境変数を設定
        os.environ["CHAMP_SETUP_ASSISTANT_DESCRIPTION_PATH"] = description_path

        # robot_descriptionをrosparamに登録
        self.description_launcher.shutdown()
        self.description_launcher.start()

    def load_robot_description(self, fixed_frame: str, description_path: str) -> None:
        self.update_urdf_file(description_path)
        self.launch_file(description_path)
        self.robot_model_display.setBool(True)
        self.frame.getManager().setFixedFrame(fixed_frame)

    def on_urdf_path_load(self):
        description_path = self.main.file_browser.description_path  # URDFの絶対パス
        self.main.robot.load_urdf(description_path)
        self.load_robot_description(self.main.robot.base, description_path)
        self.urdf_loaded.emit()
