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


class GpsWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Global Positioning System'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with GPS.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit("Topic Name", topic_description, "/fix")
        self._rows.addWidget(self.topic)

        vel_topic_description = "TODO: instruction"
        self.vel_topic = ParamGetterWidget_LineEdit(
            "Velocity Topic Name", vel_topic_description, "/fix_velocity"
        )
        self._rows.addWidget(self.vel_topic)

        body_description = "TODO: instruction"
        self.body = ParamGetterWidget_ComboBox("Body Name", body_description, [])
        self._rows.addWidget(self.body)

        frame_id_description = "TODO: instruction"
        self.frame_id = ParamGetterWidget_LineEdit("Frame ID", frame_id_description, "world")
        self._rows.addWidget(self.frame_id)

        ref_latitude_description = "TODO: instruction"
        self.ref_latitude = ParamGetterWidget_DoubleSpinBox(
            "Reference Latitude", ref_latitude_description, default=49.9
        )
        self._rows.addWidget(self.ref_latitude)

        ref_longitude_description = "TODO: instruction"
        self.ref_longitude = ParamGetterWidget_DoubleSpinBox(
            "Reference Longitude", ref_longitude_description, default=8.9
        )
        self._rows.addWidget(self.ref_longitude)

        ref_heading_description = "TODO: instruction"
        self.ref_heading = ParamGetterWidget_DoubleSpinBox(
            "Reference Heading", ref_heading_description, default=0.
        )
        self._rows.addWidget(self.ref_heading)

        ref_altitude_description = "TODO: instruction"
        self.ref_altitude = ParamGetterWidget_DoubleSpinBox(
            "Reference Altitude", ref_altitude_description, default=0.
        )
        self._rows.addWidget(self.ref_altitude)

        status_description = "TODO: instruction"
        status_choices = ["NO FIX", "FIX", "SBAS FIX", "GBAS FIX"]
        self.status = ParamGetterWidget_ComboBox(
            "Status", status_description, status_choices, default="FIX",
        )
        self._rows.addWidget(self.status)

        service_description = "TODO: instruction"
        service_choices = ["None", "GPS", "GLONASS", "COMPASS", "GALILEO"]
        self.service = ParamGetterWidget_ComboBox(
            "Service", service_description, service_choices, default="None",
        )
        self._rows.addWidget(self.service)

        self._add_dummy_widget()

        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.body.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.no_sensor.isChecked():
            self.topic.setVisible(False)
            self.vel_topic.setVisible(False)
            self.body.setVisible(False)
            self.frame_id.setVisible(False)
            self.ref_latitude.setVisible(False)
            self.ref_longitude.setVisible(False)
            self.ref_heading.setVisible(False)
            self.ref_altitude.setVisible(False)
            self.status.setVisible(False)
            self.service.setVisible(False)
        else:
            self.topic.setVisible(True)
            self.vel_topic.setVisible(True)
            self.body.setVisible(True)
            self.frame_id.setVisible(True)
            self.ref_latitude.setVisible(True)
            self.ref_longitude.setVisible(True)
            self.ref_heading.setVisible(True)
            self.ref_altitude.setVisible(True)
            self.status.setVisible(True)
            self.service.setVisible(True)
