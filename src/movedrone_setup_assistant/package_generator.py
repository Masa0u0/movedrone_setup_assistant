from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os
import os.path as osp
import yaml
import rospy
from xml.etree import ElementTree as ET
from jinja2 import Environment, FileSystemLoader
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .utils import *
from .xml_nodes import *


class PackageGenerator(QWidget):

    generated = pyqtSignal()

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._proj_path = get_proj_path()
        self._template_env = Environment(
            loader=FileSystemLoader(osp.join(self._proj_path, "templates")),
            trim_blocks=True,
            lstrip_blocks=True,
        )

        self._drone_name = ""

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)
        self._main.settings.ros_package.generate_button.clicked.connect(
            self._on_generate_button_clicked
        )

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        self._drone_name = get_drone_name()

    @pyqtSlot()
    def _on_generate_button_clicked(self) -> None:
        if not self._is_valid_config():
            return
        self._generate_pkg()
        q_info(self._main, "Configuration package is generated.")
        self.generated.emit()

    def _is_valid_config(self) -> bool:
        propellers = self._main.settings.propellers.selected
        author_info = self._main.settings.author_information
        ros_pkg = self._main.settings.ros_package

        if propellers.num() < 3:
            # TODO: ヘリのような駆動関節の先にプロペラが付いたモデルなら2つでもいけるはず
            q_error(self._main, "[Propellers]: At least 3 propellers are needed.")
            return False

        author_name = author_info.name.get()
        if author_name == "":
            q_error(self._main, "[Author Info]: Author name is blank.")
            return False

        author_email = author_info.email.get()
        if not is_valid_email(author_email):
            q_error(self._main, "[Author Info]: Invalid email address.")
            return False

        pardir = ros_pkg.pkg_path.pardir
        if not osp.isdir(pardir):
            q_error(self._main, f'[ROS Package]: {pardir} does not exist.')
            return False

        pkg_name = ros_pkg.pkg_path.pkg_name
        if pkg_name.count("/") > 0 or pkg_name.count(" "):
            q_error(self._main, f'[ROS Package]: Invalid package name.')
            return False

        pkg_path = ros_pkg.pkg_path.text()
        if osp.exists(pkg_path):
            q_error(self._main, f'[ROS Package]: {pkg_path} already exists.')
            return False

        return True

    def _generate_pkg(self) -> None:
        # 各ディレクトリのパス
        pkg_path = self._main.settings.ros_package.pkg_path.text()
        config_dir = osp.join(pkg_path, "config")
        launch_dir = osp.join(pkg_path, "launch")
        urdf_dir = osp.join(pkg_path, "urdf")

        # ディレクトリを作る
        os.mkdir(pkg_path)
        os.mkdir(config_dir)
        os.mkdir(launch_dir)
        os.mkdir(urdf_dir)

        # ファイルを作る
        items = self._make_template_items()
        self._generate_from_template(items, "CMakeLists.txt", pkg_path)
        self._generate_from_template(items, "package.xml", pkg_path)
        self._generate_from_template(items, "node_params.yaml", config_dir)
        self._generate_from_template(items, "gazebo.launch", launch_dir)
        self._generate_from_template(items, "controller.launch", launch_dir)
        self._generate_drone_props(config_dir)
        self._generate_urdf(urdf_dir)

    def _make_template_items(self) -> None:
        template_items = {}

        template_items["drone_name"] = self._drone_name

        ros_pkg = self._main.settings.ros_package
        template_items["pkg_name"] = ros_pkg.pkg_name.get()

        author_info = self._main.settings.author_information
        template_items["author_name"] = author_info.name.get()
        template_items["author_email"] = author_info.email.get()

        lmpc = self._main.settings.controllers.lmpc_settings
        lmpc_items = {
            "natural_freq": lmpc.natural_freq.get(),
            "damp_ratio": lmpc.damp_ratio.get(),
            "pred_horizon": lmpc.pred_horizon.get(),
            "pred_steps": lmpc.pred_steps.get(),
            "input_steps": lmpc.input_steps.get(),
            "rot_decay": lmpc.rot_decay.get(),
            "angvel_decay": lmpc.angvel_decay.get(),
            "rot_weight": lmpc.rot_weight.get(),
            "angvel_weight": lmpc.angvel_weight.get(),
        }
        template_items["lmpc"] = lmpc_items

        nmpc = self._main.settings.controllers.nmpc_settings
        nmpc_items = {}  # TODO
        template_items["nmpc"] = nmpc_items

        smc = self._main.settings.controllers.smc_settings
        smc_items = {}  # TODO
        template_items["smc"] = smc_items

        return template_items

    def _generate_from_template(self, items: dict, template_file: str, dest: str) -> None:
        template = self._template_env.get_template(template_file)
        content = template.render(items)  # テンプレートにdict型で文字を埋め込む
        file_path = osp.join(dest, template_file)
        with open(file_path, "w") as f:
            f.write(content)

    def _generate_drone_props(self, config_dir: str) -> None:
        # yamlファイルに書き込むための辞書を作る
        propellers_widget = self._main.settings.propellers.selected
        num_rotors = propellers_widget.num()
        drone_props = {
            "drone_name": self._drone_name,
            "num_rotors": num_rotors,
        }
        for i in range(num_rotors):
            drone_props[f'rotor_{i}'] = {
                "link_name": propellers_widget.link_names[i].text(),
                "direction": propellers_widget.directions[i].currentText().lower(),
                "max_velocity": propellers_widget.max_vels[i].value(),
                "motor_constant": propellers_widget.motor_consts[i].value(),
                "moment_constant": propellers_widget.moment_consts[i].value(),
                "rotor_drag_coefficient": propellers_widget.drag_coefs[i].value(),
                "rolling_moment_coefficient": propellers_widget.rolling_coefs[i].value(),
                "time_constant_up": propellers_widget.time_consts_up[i].value(),
                "time_constant_down": propellers_widget.time_consts_down[i].value(),
            }

        # yamlファイルを作成
        drone_props_path = osp.join(config_dir, "drone_properties.yaml")
        with open(drone_props_path, "w") as f:
            yaml.dump(drone_props, f)

    def _generate_urdf(self, urdf_dir: str) -> None:
        tree = self._make_urdf_with_plugins()
        urdf_path = osp.join(urdf_dir, f'{self._drone_name}.urdf')
        tree.write(urdf_path)

    def _make_urdf_with_plugins(self) -> ET.ElementTree:
        description = rospy.get_param("/robot_description")
        root = ET.fromstring(description)
        assert root.tag == "robot"

        self._remove_gazebo_plugins(root)
        self._add_gazebo_plugins(root)

        return ET.ElementTree(root)

    def _remove_gazebo_plugins(self, root: ET.Element) -> None:
        for child in root:
            if child.tag != "gazebo":
                continue

            for gchild in child:
                if gchild.tag == "plugin":
                    msg_box = QMessageBox(self._main)  # 親を設定しておけば一緒に落とせる
                    msg_box.setText(
                        "Gazebo plugin is detected.\n\n"
                        f'    name: {gchild.attrib["name"]}\n'
                        f'    filename: {gchild.attrib["filename"]}\n\n'
                        "This may interfere with components automatically added by MoveDrone."
                    )
                    msg_box.setInformativeText("Is it OK if this plugin is ignored?")
                    msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.No)
                    msg_box.setDefaultButton(QMessageBox.Ok)
                    ret = msg_box.exec()
                    if ret == QMessageBox.Ok:
                        root.remove(child)

                elif gchild.tag == "sensor":
                    msg_box = QMessageBox(self._main)
                    msg_box.setText(
                        "Gazebo sensor is detected.\n\n"
                        f'    name: {gchild.attrib["name"]}\n'
                        f'    type: {gchild.attrib["type"]}\n\n'
                        "This may interfere with components automatically added by MoveDrone."
                    )
                    msg_box.setInformativeText("Is it OK if this sensor is ignored?")
                    msg_box.setStandardButtons(QMessageBox.Ok | QMessageBox.No)
                    msg_box.setDefaultButton(QMessageBox.Ok)
                    ret = msg_box.exec()
                    if ret == QMessageBox.Ok:
                        root.remove(child)

    def _add_gazebo_plugins(self, root: ET.Element) -> None:
        root_link = self._main.urdf_parser.get_root().name

        # Base
        base_model = BaseModel(self._drone_name, root_link)
        root.append(base_model)

        # Motor
        propellers_widget = self._main.settings.propellers.selected
        for i in range(propellers_widget.num()):
            motor_model = MotorModel(
                self._drone_name,
                i,
                propellers_widget.link_names[i].text(),
                propellers_widget.joint_names[i].text(),
                propellers_widget.directions[i].currentText().lower(),
                propellers_widget.max_vels[i].value(),
                propellers_widget.motor_consts[i].value(),
                propellers_widget.moment_consts[i].value(),
                propellers_widget.drag_coefs[i].value(),
                propellers_widget.rolling_coefs[i].value(),
                propellers_widget.time_consts_up[i].value(),
                propellers_widget.time_consts_down[i].value(),
            )
            root.append(motor_model)

        # Controller Interface
        controller_interface = ControllerInterface(self._drone_name)
        root.append(controller_interface)

        # IMU
        imu_widget = self._main.settings.imu
        imu_model = ImuModel(
            self._drone_name,
            imu_widget.link.get(),
            imu_widget.topic.get(),
            imu_widget.gyro_noise_density.get(),
            imu_widget.gyro_random_walk.get(),
            imu_widget.gyro_bias_corr_time.get(),
            imu_widget.gyro_turn_on_bias_sigma.get(),
            imu_widget.acc_noise_density.get(),
            imu_widget.acc_random_walk.get(),
            imu_widget.acc_bias_corr_time.get(),
            imu_widget.acc_turn_on_bias_sigma.get(),
        )
        root.append(imu_model)

        # Magnetometer
        mag_widget = self._main.settings.magnetometer
        mag_model = MagneometerModel(
            self._drone_name,
            mag_widget.link.get(),
            mag_widget.topic.get(),
            mag_widget.ref_mag_north.get(),
            mag_widget.ref_mag_east.get(),
            mag_widget.ref_mag_down.get(),
            mag_widget.gauss_noise.get(),
            mag_widget.uniform_noise.get(),
        )
        root.append(mag_model)

        # GPS
        gps_widget = self._main.settings.gps
        gps_model = GpsModel(
            self._drone_name,
            gps_widget.link.get(),
            gps_widget.pos_topic.get(),
            gps_widget.vel_topic.get(),
            gps_widget.horizontal_pos_std.get(),
            gps_widget.vertical_pos_std.get(),
            gps_widget.horizontal_vel_std.get(),
            gps_widget.vertical_vel_std.get(),
        )
        root.append(gps_model)

        # IMU (Ground Truth)
        imu_gt_model = ImuModelGT(self._drone_name, imu_widget.link.get())
        root.append(imu_gt_model)

        # Odometry (Ground Truth)
        odom_gt_model = OdometryModelGT(self._drone_name, root_link)
        root.append(odom_gt_model)

        return root
