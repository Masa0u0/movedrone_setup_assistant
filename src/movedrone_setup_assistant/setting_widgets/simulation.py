from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..setting_widgets.base_setting import BaseSettingWidget


class SimulationWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Gazebo Simulation'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self._add_dummy_widget()
