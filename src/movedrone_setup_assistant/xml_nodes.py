from xml.etree import ElementTree as ET

SLOWDOWN_SIM = 10.


class BaseModel(ET.Element):

    def __init__(self, drone_name: str, root_link: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_multirotor_base_plugin.so"
        plugin.attrib["name"] = "multirotor_base_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = root_link
        ET.SubElement(plugin, "rotorVelocitySlowdownSim").text = f'{SLOWDOWN_SIM}'


class MotorModel(ET.Element):

    def __init__(
        self,
        drone_name: str,
        motor_number: int,
        link_name: str,
        joint_name: str,
        direction: str,
        max_rot_vel: float,
        motor_const: float,
        moment_const: float,
        drag_coef: float,
        roll_coef: float,
        time_const_up: float,
        time_const_down: float,
    ) -> None:
        assert direction in {"cw", "ccw"}
        assert max_rot_vel > 0.
        assert motor_const > 0.
        assert moment_const > 0.
        assert drag_coef > 0.
        assert roll_coef > 0.
        assert time_const_up > 0.
        assert time_const_down > 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_motor_model.so"
        plugin.attrib["name"] = f'{drone_name}_{motor_number}_motor_model'

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "motorNumber").text = f'{motor_number}'
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "jointName").text = joint_name
        ET.SubElement(plugin, "turningDirection").text = direction
        ET.SubElement(plugin, "maxRotVelocity").text = f'{max_rot_vel}'
        ET.SubElement(plugin, "motorConstant").text = f'{motor_const}'
        ET.SubElement(plugin, "momentConstant").text = f'{moment_const}'
        ET.SubElement(plugin, "rotorDragCoefficient").text = f'{drag_coef}'
        ET.SubElement(plugin, "rollingMomentCoefficient").text = f'{roll_coef}'
        ET.SubElement(plugin, "timeConstantUp").text = f'{time_const_up}'
        ET.SubElement(plugin, "timeConstantDown").text = f'{time_const_down}'
        ET.SubElement(plugin, "commandSubTopic").text = "gazebo/command/motor_speed"
        ET.SubElement(plugin, "motorSpeedPubTopic").text = f'motor_speed/{motor_number}'
        ET.SubElement(plugin, "rotorVelocitySlowdownSim").text = f'{SLOWDOWN_SIM}'


class ControllerInterface(ET.Element):

    def __init__(self, drone_name: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_controller_interface.so"
        plugin.attrib["name"] = "controller_interface"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "commandMotorSpeedSubTopic").text = "command/motor_speed"
        ET.SubElement(plugin, "motorSpeedCommandPubTopic").text = "gazebo/command/motor_speed"


class ImuModel(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        topic: str,
        gyro_noise_density: float,
        gyro_random_walk: float,
        gyro_bias_corr_time: float,
        gyro_turn_on_bias_sigma: float,
        acc_noise_density: float,
        acc_random_walk: float,
        acc_bias_corr_time: float,
        acc_turn_on_bias_sigma: float,
    ) -> None:
        assert gyro_noise_density >= 0.
        assert gyro_random_walk >= 0.
        assert gyro_bias_corr_time > 0.
        assert gyro_turn_on_bias_sigma >= 0.
        assert acc_noise_density >= 0.
        assert acc_random_walk >= 0.
        assert acc_bias_corr_time > 0.
        assert acc_turn_on_bias_sigma >= 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_imu_plugin.so"
        plugin.attrib["name"] = "rotors_gazebo_imu_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "imuTopic").text = topic
        ET.SubElement(plugin, "gyroscopeNoiseDensity").text = f'{gyro_noise_density}'
        ET.SubElement(plugin, "gyroscopeRandomWalk").text = f'{gyro_random_walk}'
        ET.SubElement(plugin, "gyroscopeBiasCorrelationTime").text = f'{gyro_bias_corr_time}'
        ET.SubElement(plugin, "gyroscopeTurnOnBiasSigma").text = f'{gyro_turn_on_bias_sigma}'
        ET.SubElement(plugin, "accelerometerNoiseDensity").text = f'{acc_noise_density}'
        ET.SubElement(plugin, "accelerometerRandomWalk").text = f'{acc_random_walk}'
        ET.SubElement(plugin, "accelerometerBiasCorrelationTime").text = f'{acc_bias_corr_time}'
        ET.SubElement(plugin, "accelerometerTurnOnBiasSigma").text = f'{acc_turn_on_bias_sigma}'


class MagnetometerModel(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        topic: str,
        ref_mag_north: float,
        ref_mag_east: float,
        ref_mag_down: float,
        gauss_noise: float,
        uniform_noise: float,
    ) -> None:
        assert ref_mag_north > 0.
        assert ref_mag_east > 0.
        assert ref_mag_down > 0.
        assert gauss_noise >= 0.
        assert uniform_noise >= 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_magnetometer_plugin.so"
        plugin.attrib["name"] = "rotors_gazebo_magnetometer_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "magnetometerTopic").text = topic
        ET.SubElement(plugin, "refMagNorth").text = f'{ref_mag_north}'
        ET.SubElement(plugin, "refMagEast").text = f'{ref_mag_east}'
        ET.SubElement(plugin, "refMagDown").text = f'{ref_mag_down}'
        ET.SubElement(plugin, "noiseNormal").text = f'{gauss_noise} {gauss_noise} {gauss_noise}'
        ET.SubElement(plugin, "noiseUniformInitialBias").text \
            = f'{uniform_noise} {uniform_noise} {uniform_noise}'


class BarometerModel(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        topic: str,
        ref_altitude: float,
        pressure_var: float,
    ) -> None:
        assert ref_altitude >= 0.
        assert pressure_var >= 0.

        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_pressure_plugin.so"
        plugin.attrib["name"] = "rotors_gazebo_pressure_sensor_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "pressureTopic").text = topic
        ET.SubElement(plugin, "referenceAltitude").text = f'{ref_altitude}'
        ET.SubElement(plugin, "pressureVariance").text = f'{pressure_var}'


class GpsModel(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        pos_topic: str,
        vel_topic: str,
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
    ) -> None:
        assert hor_pos_std >= 0.
        assert ver_pos_std >= 0.
        assert hor_vel_std >= 0.
        assert ver_vel_std >= 0.

        super().__init__("gazebo", reference=link_name)

        sensor = _GpsSensor(
            drone_name,
            link_name,
            pos_topic,
            vel_topic,
            hor_pos_std,
            ver_pos_std,
            hor_vel_std,
            ver_vel_std,
        )
        self.append(sensor)


class _GpsSensor(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        pos_topic: str,
        vel_topic: str,
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
    ) -> None:
        super().__init__("sensor", name=f'{drone_name}_gps', type="gps")

        ET.SubElement(self, "pose").text = "0 0 0 0 0 0"
        ET.SubElement(self, "visualize").text = "0"
        ET.SubElement(self, "always_on").text = "1"
        ET.SubElement(self, "update_rate").text = "5"

        gps = _GpsSensorGps(hor_pos_std, ver_pos_std, hor_vel_std, ver_vel_std)
        self.append(gps)

        plugin = _GpsSensorPlugin(
            drone_name,
            link_name,
            pos_topic,
            vel_topic,
            hor_pos_std,
            ver_pos_std,
            hor_vel_std,
            ver_vel_std,
        )
        self.append(plugin)


class _GpsSensorGps(ET.Element):

    def __init__(
        self,
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
    ) -> None:
        super().__init__("gps")

        pos_sensing = _GpsSensorGpsSensing("position_sensing", hor_pos_std, ver_pos_std)
        self.append(pos_sensing)

        vel_sensing = _GpsSensorGpsSensing("velocity_sensing", hor_vel_std, ver_vel_std)
        self.append(vel_sensing)


class _GpsSensorGpsSensing(ET.Element):

    def __init__(self, tag: str, hor_std: float, ver_std: float) -> None:
        super().__init__(tag)

        horizontal = _GpsSensorGpsSensingElement("horizontal", hor_std)
        self.append(horizontal)

        vertical = _GpsSensorGpsSensingElement("vertical", ver_std)
        self.append(vertical)


class _GpsSensorGpsSensingElement(ET.Element):

    def __init__(self, tag: str, std: float) -> None:
        super().__init__(tag)

        gauss_noise = ET.SubElement(self, "gauss_noise")
        gauss_noise.attrib["type"] = "gaussian"

        ET.SubElement(gauss_noise, "mean").text = "0."
        ET.SubElement(gauss_noise, "stddev").text = f'{std}'
        ET.SubElement(gauss_noise, "bias_mean").text = "0."
        ET.SubElement(gauss_noise, "bias_stddev").text = "0."


class _GpsSensorPlugin(ET.Element):

    def __init__(
        self,
        drone_name: str,
        link_name: str,
        pos_topic: str,
        vel_topic: str,
        hor_pos_std: float,
        ver_pos_std: float,
        hor_vel_std: float,
        ver_vel_std: float,
    ) -> None:
        plugin = ET.SubElement(self, "plugin")
        # plugin.attrib["filename"] = "librotors_gazebo_gps_plugin.so"
        # plugin.attrib["name"] = "rotors_gazebo_gps_plugin"
        plugin.attrib["filename"] = "libdh_gazebo_gps_plugin.so"
        plugin.attrib["name"] = "dh_gazebo_gps_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "gpsTopic").text = pos_topic
        ET.SubElement(plugin, "groundSpeedTopic").text = vel_topic
        ET.SubElement(plugin, "horPosStdDev").text = f'{hor_pos_std}'
        ET.SubElement(plugin, "verPosStdDev").text = f'{ver_pos_std}'
        ET.SubElement(plugin, "horVelStdDev").text = f'{hor_vel_std}'
        ET.SubElement(plugin, "verVelStdDev").text = f'{ver_vel_std}'


class ImuModelGT(ET.Element):

    def __init__(self, drone_name: str, link_name: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_imu_plugin.so"
        plugin.attrib["name"] = "rotors_gazebo_imu_gt_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "imuTopic").text = "ground_truth/imu"
        ET.SubElement(plugin, "gyroscopeNoiseDensity").text = "0."
        ET.SubElement(plugin, "gyroscopeRandomWalk").text = "0."
        ET.SubElement(plugin, "gyroscopeBiasCorrelationTime").text = "1000."
        ET.SubElement(plugin, "gyroscopeTurnOnBiasSigma").text = "0."
        ET.SubElement(plugin, "accelerometerNoiseDensity").text = "0."
        ET.SubElement(plugin, "accelerometerRandomWalk").text = "0."
        ET.SubElement(plugin, "accelerometerBiasCorrelationTime").text = "300."
        ET.SubElement(plugin, "accelerometerTurnOnBiasSigma").text = "0."


class OdometryModelGT(ET.Element):

    def __init__(self, drone_name: str, root_link: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "librotors_gazebo_odometry_plugin.so"
        plugin.attrib["name"] = "rotors_gazebo_odometry_gt_plugin"

        ET.SubElement(plugin, "robotNamespace").text = drone_name
        ET.SubElement(plugin, "linkName").text = root_link
        ET.SubElement(plugin, "poseTopic").text = "ground_truth/pose"
        ET.SubElement(plugin, "poseWithCovarianceTopic").text = "ground_truth/pose_with_covariance"
        ET.SubElement(plugin, "positionTopic").text = "ground_truth/position"
        ET.SubElement(plugin, "transformTopic").text = "ground_truth/transform"
        ET.SubElement(plugin, "odometryTopic").text = "ground_truth/odometry"
        ET.SubElement(plugin, "parentFrameId").text = "world"
        ET.SubElement(plugin, "childFrameId").text = root_link
        ET.SubElement(plugin, "measurementDivisor").text = "1"
        ET.SubElement(plugin, "measurementDelay").text = "0"
        ET.SubElement(plugin, "unknownDelay").text = "0."
        ET.SubElement(plugin, "noiseNormalPosition").text = "0 0 0"
        ET.SubElement(plugin, "noiseNormalQuaternion").text = "0 0 0"
        ET.SubElement(plugin, "noiseNormalLinearVelocity").text = "0 0 0"
        ET.SubElement(plugin, "noiseNormalAngularVelocity").text = "0 0 0"
        ET.SubElement(plugin, "noiseUniformPosition").text = "0 0 0"
        ET.SubElement(plugin, "noiseUniformQuaternion").text = "0 0 0"
        ET.SubElement(plugin, "noiseUniformLinearVelocity").text = "0 0 0"
        ET.SubElement(plugin, "noiseUniformAngularVelocity").text = "0 0 0"


class GazeboRosControlModel(ET.Element):

    def __init__(self, drone_name: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libgazebo_ros_control.so"
        plugin.attrib["name"] = "gazebo_ros_control"

        ET.SubElement(plugin, "robotNamespace").text = drone_name  # コントローラはこのNSに属する
        ET.SubElement(plugin, "robotSimType").text = "gazebo_ros_control/DefaultRobotHWSim"
        ET.SubElement(plugin, "legacyModeNS").text = "true"


class TransmissionModel(ET.Element):

    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"

    def __init__(self, joint_name: str, interface: str = POSITION, reduction: float = 1.):
        assert interface in {self.POSITION, self.VELOCITY, self.EFFORT}
        assert reduction >= 1.

        super().__init__("transmission", name=f'{joint_name}_trans')

        ET.SubElement(self, "type").text = "transmission_interface/SimpleTransmission"

        joint = ET.SubElement(self, "joint", {"name": joint_name})
        ET.SubElement(joint, "hardwareInterface").text = interface

        actuator = ET.SubElement(self, "actuator", {"name": f'{joint_name}_actuator'})
        ET.SubElement(actuator, "hardwareInterface").text = interface
        ET.SubElement(actuator, "mechanicalReduction").text = f'{reduction}'
