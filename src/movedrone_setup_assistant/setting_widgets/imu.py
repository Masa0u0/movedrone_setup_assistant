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


class ImuWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Inertial Measurement Unit'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        body_description = "TODO: instruction"
        self.body = ParamGetterWidget_ComboBox("Body Name", body_description, [])
        self._rows.addWidget(self.body)

        topic_description = "TODO: instruction"
        self.topic = ParamGetterWidget_LineEdit("IMU Topic", topic_description, "/imu")
        self._rows.addWidget(self.topic)

        self.use_custom_imu = QCheckBox("Use custom IMU")
        self.use_custom_imu.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.use_custom_imu)

        gyro_noise_density_description = "TODO: instruction"
        self.gyro_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope Noise Density", gyro_noise_density_description, min=0., default=0.0003394
        )
        self._rows.addWidget(self.gyro_noise_density)

        gyro_random_walk_description = "TODO: instruction"
        self.gyro_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope Random Walk", gyro_random_walk_description, min=0., default=0.000038785
        )
        self._rows.addWidget(self.gyro_random_walk)

        gyro_bias_corr_time_description = "TODO: instruction"
        self.gyro_bias_corr_time = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope Bias Correlation Time", gyro_bias_corr_time_description, min=0., default=1000.
        )
        self._rows.addWidget(self.gyro_bias_corr_time)

        gyro_turn_on_bias_sigma_description = "TODO: instruction"
        self.gyro_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Gyroscope Turn On Bias Sigma", gyro_turn_on_bias_sigma_description, min=0., default=0.0087
        )
        self._rows.addWidget(self.gyro_turn_on_bias_sigma)

        accel_noise_density_description = "TODO: instruction"
        self.accel_noise_density = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer Noise Density", accel_noise_density_description, min=0., default=0.004
        )
        self._rows.addWidget(self.accel_noise_density)

        accel_random_walk_description = "TODO: instruction"
        self.accel_random_walk = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer Random Walk", accel_random_walk_description, min=0., default=0.006
        )
        self._rows.addWidget(self.accel_random_walk)

        accel_bias_corr_time_description = "TODO: instruction"
        self.accel_bias_corr_time = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer Bias Correlation Time", accel_bias_corr_time_description, min=0., default=300.
        )
        self._rows.addWidget(self.accel_bias_corr_time)

        accel_turn_on_bias_sigma_description = "TODO: instruction"
        self.accel_turn_on_bias_sigma = ParamGetterWidget_DoubleSpinBox(
            "Accelerometer Turn On Bias Sigma", accel_turn_on_bias_sigma_description, min=0., default=0.196
        )
        self._rows.addWidget(self.accel_turn_on_bias_sigma)

        self._add_dummy_widget()
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.use_custom_imu.toggled.connect(self._update_visibility)
        self._main.urdf_parser.robot_model_updated.connect(self._add_fixed_links)

    @pyqtSlot()
    def _add_fixed_links(self) -> None:
        body_choices = self._main.urdf_parser.get_fixed_link_names()
        self.body.box.addItems(body_choices)

    @pyqtSlot()
    def _update_visibility(self) -> None:
        if self.use_custom_imu.isChecked():
            self.gyro_noise_density.setVisible(True)
            self.gyro_random_walk.setVisible(True)
            self.gyro_bias_corr_time.setVisible(True)
            self.gyro_turn_on_bias_sigma.setVisible(True)
            self.accel_noise_density.setVisible(True)
            self.accel_random_walk.setVisible(True)
            self.accel_bias_corr_time.setVisible(True)
            self.accel_turn_on_bias_sigma.setVisible(True)
        else:
            self.gyro_noise_density.setVisible(False)
            self.gyro_random_walk.setVisible(False)
            self.gyro_bias_corr_time.setVisible(False)
            self.gyro_turn_on_bias_sigma.setVisible(False)
            self.accel_noise_density.setVisible(False)
            self.accel_random_walk.setVisible(False)
            self.accel_bias_corr_time.setVisible(False)
            self.accel_turn_on_bias_sigma.setVisible(False)
