from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from .robot_model_loader import RobotModelLoaderWidget


class StartWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'MoveDrone Setup Assistant'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.robot_model_loader = RobotModelLoaderWidget(main)
        self.rows.addWidget(self.robot_model_loader)

    def define_connections(self) -> None:
        self.robot_model_loader.define_connections()
