from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class ImuWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Inertial Measurement Unit'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit("Topic Name", topic_description, "/imu/data_raw")
        self._rows.addWidget(self.topic)

        body_description = "TODO: instruction"
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.body = ParamGetterWidget_ComboBox("Body Name", body_description, body_choices)
        self._rows.addWidget(self.body)

        gaussian_noise_description = "TODO: instruction"
        self.gaussian_noise = ParamGetterWidget_DoubleSpinBox(
            "Gaussian Noise", gaussian_noise_description, min=0., default=0.
        )
        self._rows.addWidget(self.gaussian_noise)

        trans_offset_description = "TODO: instruction"
        self.trans_offset = ParamGetterWidget_Vector3d("Translational Offset", trans_offset_description)
        self._rows.addWidget(self.trans_offset)

        rot_offset_description = "TODO: instruction"
        self.rot_offset = ParamGetterWidget_Vector3d("Rotational Offset", rot_offset_description)
        self._rows.addWidget(self.rot_offset)
        
        self._add_dummy_widget()
