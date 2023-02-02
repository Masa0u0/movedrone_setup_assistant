from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from pyqt_vertical_tab_widget.verticalTabWidget import VerticalTabWidget

from .start import StartWidget
from .propellers import PropellersWidget
from .imu import ImuWidget
from .gps import GpsWidget
from .perception_3d import Perception3dWidget
from .controllers import ControllersWidget
from .simulation import SimulationWidget
from .author_information import AuthorInformationWidget
from .ros_package import RosPackageWidget


class SettingsWidget(VerticalTabWidget):
    
    TAB_HEIGHT = 30  # 30以上無いと何故かTabBarの文字が横に見切れてしまう
    TAB_WIDTH = 70
    MIN_HEIGHT = 600

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self.main = main

        self.start = StartWidget(main)
        self.propellers = PropellersWidget(main)
        self.imu = ImuWidget(main)
        self.gps = GpsWidget(main)
        self.perception_3d = Perception3dWidget(main)
        self.controllers = ControllersWidget(main)
        self.simulation = SimulationWidget(main)
        self.author_information = AuthorInformationWidget(main)
        self.ros_package = RosPackageWidget(main)

        self.addTab(self.start, 'Start')
        self.addTab(self.propellers, 'Propellers')
        self.addTab(self.imu, 'IMU')
        self.addTab(self.gps, 'GPS')
        self.addTab(self.perception_3d, '3D Perception')
        self.addTab(self.controllers, 'Controllers')
        self.addTab(self.simulation, 'Simulation')
        self.addTab(self.author_information, 'Author Info')
        self.addTab(self.ros_package, 'ROS Package')

        self.setMinimumHeight(self.MIN_HEIGHT)
        self.setStyleSheet(f'QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}')

    def define_connections(self) -> None:
        self.start.define_connections()
        self.propellers.define_connections()
        self.imu.define_connections()
        self.gps.define_connections()
        self.perception_3d.define_connections()
        self.controllers.define_connections()
        self.simulation.define_connections()
        self.author_information.define_connections()
        self.ros_package.define_connections()
