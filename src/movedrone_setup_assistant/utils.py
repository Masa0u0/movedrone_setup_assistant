from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import re
import rospy
import rospkg
from xml.etree import ElementTree as ET
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def get_proj_path() -> str:
    """ スクリプトのパッケージまでのフルパスを返す． """
    pkg_name = rospkg.get_package_name(__file__)
    return rospkg.RosPack().get_path(pkg_name)


def get_drone_name() -> str:
    """ URDFからドローンの名前を取得する． """
    description = rospy.get_param("/robot_description")
    root = ET.fromstring(description)
    assert root.tag == "robot"
    return root.get("name")


def is_valid_email(email: str) -> bool:
    """
    Emailアドレスが有効かどうかを判定する．
    cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
    """
    regex = r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b'
    return re.fullmatch(regex, email)


def q_info(main: SetupAssistant, msg: str) -> None:
    rospy.loginfo(msg)
    QMessageBox.information(main, "INFO", msg)


def q_warn(main: SetupAssistant, msg: str) -> None:
    rospy.logwarn(msg)
    QMessageBox.warning(main, "WARN", msg)


def q_error(main: SetupAssistant, msg: str) -> None:
    rospy.logerr(msg)
    QMessageBox.critical(main, "ERROR", msg)
