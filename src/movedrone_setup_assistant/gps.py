from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from .parameter_getters import *


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Global Positioning System'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        frame_description = "TODO: instruction"
        frame_choices = self.main.urdf_parser.get_fixed_link_names()
        self.frame_getter = ParamGetterWidget_ComboBox("Frame", frame_description, frame_choices)
        self.rows.addWidget(self.frame_getter)

        topic_description = "TODO: instruction"
        self.topic_getter = ParamGetterWidget_LineEdit("GPS Topic", topic_description, "/gps/data")
        self.rows.addWidget(self.topic_getter)

        update_rate_description = "TODO: instruction"
        self.update_rate_getter = ParamGetterWidget_DoubleSpinBox(
            "Update Rate",
            update_rate_description,
            min=1.,
            suffix=" Hz",
        )
        self.rows.addWidget(self.update_rate_getter)

        gaussian_noise_description = "TODO: instruction"
        self.gaussian_noise_getter = ParamGetterWidget_DoubleSpinBox(
            "Velocity Gaussian Noise",
            gaussian_noise_description,
            min=0.,
        )
        self.rows.addWidget(self.gaussian_noise_getter)

    def define_connections(self) -> None:
        pass
