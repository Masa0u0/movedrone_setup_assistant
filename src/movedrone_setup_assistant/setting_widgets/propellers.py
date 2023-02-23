from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from typing import Union, List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..basic_widgets import ComboBox, DoubleSpinBox
from ..const import *
from ..utils import q_error


class PropellersWidget(BaseSettingWidget):

    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Propellers'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        propellers_label = QLabel("Propellers")
        propellers_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        propellers_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(propellers_label)

        self.selected = SelectedPropellersWidget(main)
        self._rows.addWidget(self.selected)

        self.add_delete = AddDeleteButtonsWidget(main)
        self._rows.addWidget(self.add_delete)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self.available_links = AvailableLinksWidget(main)
        self._rows.addWidget(self.available_links)

    def define_connections(self) -> None:
        super().define_connections()
        self.selected.define_connections()
        self.add_delete.define_connections()
        self.available_links.define_connections()


class SelectedPropellersWidget(QTableWidget):

    NUM_ENTRIES = 10
    COL_WIDTH = 180

    link_added = pyqtSignal(str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, self.NUM_ENTRIES)
        self._main = main

        self.link_names: List[QLabel] = []
        self.joint_names: List[QLabel] = []
        self.directions: List[ComboBox] = []
        self.max_vels: List[DoubleSpinBox] = []
        self.motor_consts: List[DoubleSpinBox] = []
        self.moment_consts: List[DoubleSpinBox] = []
        self.drag_coefs: List[DoubleSpinBox] = []
        self.rolling_coefs: List[DoubleSpinBox] = []
        self.time_consts_up: List[DoubleSpinBox] = []
        self.time_consts_down: List[DoubleSpinBox] = []

        self.setHorizontalHeaderLabels([
            "Link Name",
            "Joint Name",
            "Direction",
            "Max Velocity",
            "Motor Constant",
            "Moment Constant",
            "Rotor Drag Coefficient",
            "Rolling Moment Coefficient",
            "Time Constant Up",
            "Time Constant Down",
        ])

        for c in range(self.columnCount()):
            self.setColumnWidth(c, self.COL_WIDTH)

    def define_connections(self) -> None:
        # 必ずAdd -> Deleteの順に実行する
        self._main.settings.propellers.add_delete.add.connect(self._add_selected_link)
        self._main.settings.propellers.available_links.link_added.connect(self._delete_cur_row)

    def selected_link(self) -> Union[str, None]:
        row = self.currentRow()
        if row < 0:
            return None
        # return self.cellWidget(row, 0).property("text")
        return self.link_names[row].text()

    def num(self) -> int:
        """ 選択テーブル内のプロペラの個数を返す． """
        return len(self.link_names)

    def get_link_names(self) -> List[str]:
        """ 選択テーブル内のリンクの名前のリストを返す． """
        return [link_name.text() for link_name in self.link_names]

    def get_joint_names(self) -> List[str]:
        """ 選択テーブル内のジョイントの名前のリストを返す． """
        return [joint_name.text() for joint_name in self.joint_names]

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.propellers.available_links.selected_link()
        if selected_link is None:
            q_error(self._main, "No link is selected.")
            return

        row = self.rowCount()
        self.insertRow(row)

        link_name = QLabel(selected_link)
        link_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        link_name.setAlignment(Qt.AlignCenter)
        self.link_names.append(link_name)
        self.setCellWidget(row, 0, link_name)

        joint_name = QLabel(self._main.urdf_parser.get_joint(selected_link).name)
        joint_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        joint_name.setAlignment(Qt.AlignCenter)
        self.joint_names.append(joint_name)
        self.setCellWidget(row, 1, joint_name)

        direction = ComboBox()
        direction.addItems(["CW", "CCW"])
        self.directions.append(direction)
        self.setCellWidget(row, 2, direction)

        max_vel = DoubleSpinBox()
        max_vel.setMinimum(0.)
        max_vel.setMaximum(1e+4)
        max_vel.setDecimals(1)
        max_vel.setSuffix(" rad/s")
        self.max_vels.append(max_vel)
        self.setCellWidget(row, 3, max_vel)

        motor_const = DoubleSpinBox()
        motor_const.setMinimum(0.)
        motor_const.setDecimals(12)
        motor_const.setSuffix(" kg*m/s^2")
        self.motor_consts.append(motor_const)
        self.setCellWidget(row, 4, motor_const)

        moment_const = DoubleSpinBox()
        moment_const.setMinimum(0.)
        moment_const.setDecimals(6)
        moment_const.setSuffix(" m")
        self.moment_consts.append(moment_const)
        self.setCellWidget(row, 5, moment_const)

        drag_coef = DoubleSpinBox()
        drag_coef.setMinimum(0.)
        drag_coef.setDecimals(9)
        self.drag_coefs.append(drag_coef)
        self.setCellWidget(row, 6, drag_coef)

        rolling_coef = DoubleSpinBox()
        rolling_coef.setMinimum(0.)
        rolling_coef.setDecimals(9)
        self.rolling_coefs.append(rolling_coef)
        self.setCellWidget(row, 7, rolling_coef)

        time_const_up = DoubleSpinBox()
        time_const_up.setMinimum(0.)
        time_const_up.setDecimals(6)
        time_const_up.setSuffix(" s")
        self.time_consts_up.append(time_const_up)
        self.setCellWidget(row, 8, time_const_up)

        time_const_down = DoubleSpinBox()
        time_const_down.setMinimum(0.)
        time_const_down.setDecimals(6)
        time_const_down.setSuffix(" s")
        self.time_consts_down.append(time_const_down)
        self.setCellWidget(row, 9, time_const_down)

        if row == 0:
            # 1段目はデフォルト値
            max_vel.setValue(838.)
            motor_const.setValue(8.54858e-6)
            moment_const.setValue(0.016)
            drag_coef.setValue(8.06428e-5)
            rolling_coef.setValue(1e-6)
            time_const_up.setValue(0.0125)
            time_const_down.setValue(0.025)
        else:
            # 2段目以降は直前の設定を初期値とする
            direction.setCurrentText(self.directions[row - 1].currentText())
            max_vel.setValue(self.max_vels[row - 1].value())
            motor_const.setValue(self.motor_consts[row - 1].value())
            moment_const.setValue(self.moment_consts[row - 1].value())
            drag_coef.setValue(self.drag_coefs[row - 1].value())
            rolling_coef.setValue(self.rolling_coefs[row - 1].value())
            time_const_up.setValue(self.time_consts_up[row - 1].value())
            time_const_down.setValue(self.time_consts_down[row - 1].value())

        self.link_added.emit(selected_link)

    @pyqtSlot()
    def _delete_cur_row(self) -> None:
        row = self.currentRow()
        if row < 0:
            return

        self.removeRow(row)

        self.link_names.pop(row)
        self.joint_names.pop(row)
        self.directions.pop(row)
        self.max_vels.pop(row)
        self.motor_consts.pop(row)
        self.moment_consts.pop(row)
        self.drag_coefs.pop(row)
        self.rolling_coefs.pop(row)
        self.time_consts_up.pop(row)
        self.time_consts_down.pop(row)


class AddDeleteButtonsWidget(QWidget):

    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    add = pyqtSignal()
    delete = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._add_button = QPushButton("⬆")
        self._add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._add_button)

        self._delete_button = QPushButton("⬇")
        self._delete_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._delete_button)

    def define_connections(self) -> None:
        # 必ずAdd -> Deleteの順に実行する
        self._add_button.clicked.connect(self._add_button_clicked)
        self._delete_button.clicked.connect(self._delete_button_clicked)

    @pyqtSlot()
    def _add_button_clicked(self) -> None:
        self.add.emit()

    @pyqtSlot()
    def _delete_button_clicked(self) -> None:
        self.delete.emit()


class AvailableLinksWidget(QListWidget):

    link_added = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)
        self._main.settings.propellers.add_delete.delete.connect(self._add_selected_link)
        self._main.settings.propellers.selected.link_added.connect(self._delete_selected_link)

    def add_link(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f'Unknown link: {link_name}'
        assert not self._link_exists_in_list(link_name), f'Duplicated: {link_name}'
        self.addItem(link_name)

    def delete_link(self, link_name: str) -> None:
        links = self.findItems(link_name, Qt.MatchExactly)
        assert len(links) > 0
        for link in links:  # link: PyQt5.QtWidgets.QListWidgetItem
            mathced_link = self.row(link)  # linkの行番号をint型で取得
            self.takeItem(mathced_link)

    def selected_link(self) -> Union[str, None]:
        try:
            return self.currentItem().text()
        except:
            return None

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """ rootから複数のfixedと1つのcontinuousで繋がったリンクのみプロペラ候補とする． """
        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()
        fixed_link_names = self._main.urdf_parser.get_fixed_link_names()

        for link in links:
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            parent = self._main.urdf_parser.get_parent(link.name)
            if joint.type == "continuous" and parent.name in fixed_link_names:
                self.add_link(link.name)

        self.sortItems()

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.propellers.selected.selected_link()
        if selected_link is None:
            q_error(self._main, "No link is selected.")
            return

        self.add_link(selected_link)
        self.sortItems()

        self.link_added.emit()

    @pyqtSlot(str)
    def _delete_selected_link(self, link_name: str) -> None:
        self.delete_link(link_name)

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0
