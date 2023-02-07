from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Global Positioning System'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        frame_description = "TODO: instruction"
        frame_choices = self._main.urdf_parser.get_fixed_link_names()
        self.frame = ParamGetterWidget_ComboBox("Frame", frame_description, frame_choices)
        self._rows.addWidget(self.frame)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit("GPS Topic", topic_description, "/gps/data")
        self._rows.addWidget(self.topic)

        update_rate_description = "TODO: instruction"
        self.update_rate = ParamGetterWidget_DoubleSpinBox(
            "Update Rate",
            update_rate_description,
            min=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.update_rate)

        gaussian_noise_description = "TODO: instruction"
        self.gaussian_noise = ParamGetterWidget_DoubleSpinBox(
            "Velocity Gaussian Noise",
            gaussian_noise_description,
            min=0.,
        )
        self._rows.addWidget(self.gaussian_noise)

        self._add_dummy_widget()
