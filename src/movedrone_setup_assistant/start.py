from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from .base_setting import BaseSettingWidget
from .robot_model_loader import RobotModelLoaderWidget


class StartWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        tab_text = 'MoveDrone Setup Assistant'
        abst_text = 'TODO: abstruct'
        super().__init__(main, tab_text, abst_text)

        self.robot_model_loader = RobotModelLoaderWidget(main)
        self.rows.addWidget(self.robot_model_loader)

    def define_connections(self) -> None:
        self.robot_model_loader.define_connections()
