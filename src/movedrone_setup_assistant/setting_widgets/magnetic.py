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


class MagneticSensorWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Magnetic Sensor'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with magnetic sensor.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit("Topic Name", topic_description, "/imu/mag")
        self._rows.addWidget(self.topic)

        body_description = "TODO: instruction"
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.body = ParamGetterWidget_ComboBox("Body Name", body_description, body_choices)
        self._rows.addWidget(self.body)

        self.edit_sim_params = QCheckBox("Edit parameters only for simulation.")
        self.edit_sim_params.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.edit_sim_params)

        magnitude_description = "TODO: instruction"
        self.magnitude = ParamGetterWidget_DoubleSpinBox(
            "Magnitude", magnitude_description, min=0., default=1.
        )
        self._rows.addWidget(self.magnitude)

        ref_heading_description = "TODO: instruction"
        self.ref_heading = ParamGetterWidget_DoubleSpinBox(
            "Reference Heading", ref_heading_description, min=-90., max=90., default=0.
        )
        self._rows.addWidget(self.ref_heading)

        declination_description = "TODO: instruction"
        self.declination = ParamGetterWidget_DoubleSpinBox(
            "Declination", declination_description, min=-90., max=90., default=0.
        )
        self._rows.addWidget(self.declination)

        inclunation_description = "TODO: instruction"
        self.inclunation = ParamGetterWidget_DoubleSpinBox(
            "Inclination", inclunation_description, min=-90., max=90., default=60.
        )
        self._rows.addWidget(self.inclunation)

        self._add_dummy_widget()

        self._update_visibility()  # slotは通常のメソッドとして呼び出すこともできる

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self.edit_sim_params.toggled.connect(self._update_visibility)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.edit_sim_params.setVisible(False)
            self.topic.setVisible(False)
            self.body.setVisible(False)
            self.magnitude.setVisible(False)
            self.ref_heading.setVisible(False)
            self.declination.setVisible(False)
            self.inclunation.setVisible(False)
        else:
            self.edit_sim_params.setVisible(True)
            self.topic.setVisible(True)
            self.body.setVisible(True)
            if self.edit_sim_params.isChecked():
                self.magnitude.setVisible(True)
                self.ref_heading.setVisible(True)
                self.declination.setVisible(True)
                self.inclunation.setVisible(True)
            else:
                self.magnitude.setVisible(False)
                self.ref_heading.setVisible(False)
                self.declination.setVisible(False)
                self.inclunation.setVisible(False)
