from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..const import *
from ..parameter_getters import *


class MagnetometerWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Magnetometer'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        body_description = "TODO: instruction"
        self.body = ParamGetterWidget_ComboBox("Body Name", body_description, [])
        self._rows.addWidget(self.body)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit(
            "Magnetometer Topic", topic_description, "/magnetic_field")
        self._rows.addWidget(self.topic)

        self.use_custom_magnetometer = QCheckBox("Use custom magnetometer")
        self.use_custom_magnetometer.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.use_custom_magnetometer)

        ref_mag_north_description = "TODO: instruction"
        self.ref_mag_north = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude North", ref_mag_north_description, min=0., default=0.000021493
        )
        self._rows.addWidget(self.ref_mag_north)

        ref_mag_east_description = "TODO: instruction"
        self.ref_mag_east = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude East", ref_mag_east_description, min=0., default=0.000000815
        )
        self._rows.addWidget(self.ref_mag_east)

        ref_mag_down_description = "TODO: instruction"
        self.ref_mag_down = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude Down", ref_mag_down_description, min=0., default=0.000042795
        )
        self._rows.addWidget(self.ref_mag_down)

        noise_normal_description = "TODO: instruction"
        self.noise_normal = ParamGetterWidget_Vector3d(
            "Noise Normal",
            noise_normal_description,
            min=[0., 0., 0.],
            default=[0.00000008, 0.00000008, 0.00000008],
        )
        self._rows.addWidget(self.noise_normal)

        noise_uniform_description = "TODO: instruction"
        self.noise_uniform = ParamGetterWidget_Vector3d(
            "Noise Uniform Initial Bias",
            noise_uniform_description,
            default=[0.0000004, 0.0000004, 0.0000004],
        )
        self._rows.addWidget(self.noise_uniform)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.use_custom_magnetometer.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.body.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.use_custom_magnetometer.isChecked():
            self.ref_mag_north.setVisible(True)
            self.ref_mag_east.setVisible(True)
            self.ref_mag_down.setVisible(True)
            self.noise_normal.setVisible(True)
            self.noise_uniform.setVisible(True)
        else:
            self.ref_mag_north.setVisible(False)
            self.ref_mag_east.setVisible(False)
            self.ref_mag_down.setVisible(False)
            self.noise_normal.setVisible(False)
            self.noise_uniform.setVisible(False)
