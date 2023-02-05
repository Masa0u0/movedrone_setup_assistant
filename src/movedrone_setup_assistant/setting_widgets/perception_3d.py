from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..const import *


class Perception3dWidget(BaseSettingWidget):

    POINT_CLOUD_LABEL = "Point Cloud"
    DEPTH_MAP_LABEL = "Depth Map"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Setup 3D Perception Sensor'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.no_sensor = QCheckBox("The drone is not equipped with 3D perception sensor.")
        self.no_sensor.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._rows.addWidget(self.no_sensor)

        type_description = "TODO: instruction"
        self.sensor_type_getter = ParamGetterWidget_ComboBox(
            "Type of 3D Perception Sensor",
            type_description,
            [self.POINT_CLOUD_LABEL, self.DEPTH_MAP_LABEL],
        )
        self._rows.addWidget(self.sensor_type_getter)

        self.point_cloud_settings = PointCloudSettingsWidget(main)
        self._rows.addWidget(self.point_cloud_settings)

        self.depth_map_settings = DepthMapSettingsWidget(main)
        self._rows.addWidget(self.depth_map_settings)

        self._add_dummy_widget()

        self._update_sensors_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.no_sensor.toggled.connect(self._on_no_sensor_toggled)
        self.sensor_type_getter.text_changed.connect(self._on_type_changed)

    def _update_sensors_visibility(self) -> None:
        sensor_type = self.sensor_type_getter.get()

        if sensor_type == self.POINT_CLOUD_LABEL:
            self.point_cloud_settings.setVisible(True)
            self.depth_map_settings.setVisible(False)
        elif sensor_type == self.DEPTH_MAP_LABEL:
            self.point_cloud_settings.setVisible(False)
            self.depth_map_settings.setVisible(True)
        else:
            raise RuntimeError(f'Invalid sensor type: {sensor_type}')

    @pyqtSlot()
    def _on_no_sensor_toggled(self) -> None:
        if self.no_sensor.isChecked():
            self.sensor_type_getter.setVisible(False)
            self.point_cloud_settings.setVisible(False)
            self.depth_map_settings.setVisible(False)
        else:
            self.sensor_type_getter.setVisible(True)
            self._update_sensors_visibility()

    @pyqtSlot(str)
    def _on_type_changed(self, sensor_type: str) -> None:
        self._update_sensors_visibility()


class PointCloudSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        frame_description = "TODO: instruction"
        frame_choices = self._main.urdf_parser.get_fixed_link_names()
        self.frame_getter = ParamGetterWidget_ComboBox("Frame", frame_description, frame_choices)
        self._rows.addWidget(self.frame_getter)

        raw_topic_description = "TODO: instruction"
        self.raw_topic_getter = ParamGetterWidget_LineEdit(
            "Point Cloud Topic",
            raw_topic_description,
            "/head_mount_kinect/depth_registered/points")
        self._rows.addWidget(self.raw_topic_getter)

        max_range_description = "TODO: instruction"
        self.max_range_getter = ParamGetterWidget_DoubleSpinBox(
            "Max Range",
            max_range_description,
            min=0.,
            default=5.,
        )
        self._rows.addWidget(self.max_range_getter)

        subsample_description = "TODO: instruction"
        self.subsample_getter = ParamGetterWidget_SpinBox(
            "Point Subsample",
            subsample_description,
            min=0,
            default=1,
        )
        self._rows.addWidget(self.subsample_getter)

        padding_offset_description = "TODO: instruction"
        self.padding_offset_getter = ParamGetterWidget_DoubleSpinBox(
            "Padding Offset",
            padding_offset_description,
            min=0.,
            default=0.1,
        )
        self._rows.addWidget(self.padding_offset_getter)

        padding_scale_description = "TODO: instruction"
        self.padding_scale_getter = ParamGetterWidget_DoubleSpinBox(
            "Padding Scale",
            padding_scale_description,
            min=0.,
            default=0.1,
        )
        self._rows.addWidget(self.padding_scale_getter)

        filtered_topic_description = "TODO: instruction"
        self.filtered_topic_getter = ParamGetterWidget_LineEdit(
            "Filtered Cloud Topic",
            filtered_topic_description,
            "/head_mount_kinect/depth_registered/points")
        self._rows.addWidget(self.filtered_topic_getter)

        max_update_rate_description = "TODO: instruction"
        self.max_update_rate_getter = ParamGetterWidget_DoubleSpinBox(
            "Max Update Rate",
            max_update_rate_description,
            min=1.,
            suffix=" Hz",
        )
        self._rows.addWidget(self.max_update_rate_getter)


class DepthMapSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        # TODO
