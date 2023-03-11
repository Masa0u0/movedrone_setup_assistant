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

        link_description = "TODO: instruction"
        self.link = ParamGetterWidget_ComboBox("Link name", link_description, [])
        self._rows.addWidget(self.link)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit(
            "Magnetometer topic", topic_description, "/magnetic_field")
        self._rows.addWidget(self.topic)

        self.use_custom_magnetometer = QCheckBox("Use custom magnetometer")
        self.use_custom_magnetometer.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.use_custom_magnetometer)

        ref_mag_north_description = "TODO: instruction"
        self.ref_mag_north = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude North", ref_mag_north_description, minimum=0., default=2.1493e-5
        )
        self._rows.addWidget(self.ref_mag_north)

        ref_mag_east_description = "TODO: instruction"
        self.ref_mag_east = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude East", ref_mag_east_description, minimum=0., default=8.15e-7
        )
        self._rows.addWidget(self.ref_mag_east)

        ref_mag_down_description = "TODO: instruction"
        self.ref_mag_down = ParamGetterWidget_DoubleSpinBox(
            "Reference Magnitude Down", ref_mag_down_description, minimum=0., default=4.2795e-5
        )
        self._rows.addWidget(self.ref_mag_down)

        gauss_noise_description = "TODO: instruction"
        self.gauss_noise = ParamGetterWidget_DoubleSpinBox(
            "Standard deviation of additive white gaussian noise [Tesla]",
            gauss_noise_description,
            minimum=0.,
            default=8e-8,
        )
        self._rows.addWidget(self.gauss_noise)

        uniform_noise_description = "TODO: instruction"
        self.uniform_noise = ParamGetterWidget_DoubleSpinBox(
            "Symmetric bounds of uniform noise for initial sensor bias [Tesla]",
            uniform_noise_description,
            minimum=0.,
            default=4e-7,
        )
        self._rows.addWidget(self.uniform_noise)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.use_custom_magnetometer.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.link.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.use_custom_magnetometer.isChecked():
            self.ref_mag_north.setVisible(True)
            self.ref_mag_east.setVisible(True)
            self.ref_mag_down.setVisible(True)
            self.gauss_noise.setVisible(True)
            self.uniform_noise.setVisible(True)
        else:
            self.ref_mag_north.setVisible(False)
            self.ref_mag_east.setVisible(False)
            self.ref_mag_down.setVisible(False)
            self.gauss_noise.setVisible(False)
            self.uniform_noise.setVisible(False)
