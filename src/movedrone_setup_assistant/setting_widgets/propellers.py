from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from typing import Union
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from ..const import *


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

    link_added = pyqtSignal(str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, 4)
        self._main = main

        self.setHorizontalHeaderLabels([
            "Link Name",
            "Direction",
            "Motor Constant",
            "Moment Constant",
        ])

    def define_connections(self) -> None:
        # 必ずAdd -> Deleteの順に実行する
        self._main.settings.propellers.add_delete.add.connect(self._add_selected_link)
        self._main.settings.propellers.available_links.link_added.connect(self._delete_cur_row)

    def selected_link(self) -> Union[str, None]:
        row = self.currentRow()
        if row < 0:
            return None
        return self.cellWidget(row, 0).property("text")

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.propellers.available_links.selected_link()
        if selected_link is None:
            QMessageBox.information(self, "ERROR", "No link is selected.")
            return

        row = self.rowCount()
        self.insertRow(row)

        link_name = QLabel(selected_link)
        link_name.setFont(QFont("Default", pointSize=BODY_PSIZE))
        link_name.setAlignment(Qt.AlignCenter)
        self.setCellWidget(row, 0, link_name)

        self.link_added.emit(selected_link)

    @pyqtSlot()
    def _delete_cur_row(self) -> None:
        row = self.currentRow()
        if row < 0:
            return
        self.removeRow(row)


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
        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()
        for link in links:
            if link.name == root_link.name:
                continue
            joint = self._main.urdf_parser.get_joint(link.name)
            if joint.type == "continuous":
                self.add_link(link.name)

    @pyqtSlot()
    def _add_selected_link(self) -> None:
        selected_link = self._main.settings.propellers.selected.selected_link()
        if selected_link is None:
            QMessageBox.information(self, "ERROR", "No link is selected.")
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
