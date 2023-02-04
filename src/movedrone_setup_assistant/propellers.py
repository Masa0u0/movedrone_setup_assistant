from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget


class PropellersWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        tab_text = 'Define Propellers'
        abst_text = 'TODO: abstruct'
        super().__init__(main, tab_text, abst_text)

        self.selected = SelectedPropellersWidget(main)
        self.rows.addWidget(self.selected)

        self.add_delete = AddDeleteButtonsWidget(main)
        self.rows.addWidget(self.add_delete)

        self.available_links = AvailableLinksWidget(main)
        self.rows.addWidget(self.available_links)

    def define_connections(self) -> None:
        self.selected.define_connections()
        self.add_delete.define_connections()
        self.available_links.define_connections()


class SelectedPropellersWidget(QTableWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(0, 4)
        self.main = main

        self.setHorizontalHeaderLabels([
            "Link Name",
            "Direction",
            "Motor Constant",
            "Moment Constant",
        ])

    def define_connections(self) -> None:
        self.main.settings.propellers.add_delete.add_button.clicked.connect(self._add_new_link)

    @pyqtSlot()
    def _add_new_link(self) -> None:
        # TODO
        n_rows = self.rowCount()
        self.insertRow(n_rows)


class AddDeleteButtonsWidget(QWidget):

    BUTTON_HEIGHT = 20
    BUTTON_WIDTH = 50

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        self.cols = QHBoxLayout()
        self.setLayout(self.cols)

        self.add_button = QPushButton("⬆")
        self.add_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.add_button.setFixedHeight(self.BUTTON_HEIGHT)
        self.add_button.setFixedWidth(self.BUTTON_WIDTH)
        self.cols.addWidget(self.add_button)

        self.delete_button = QPushButton("⬇")
        self.delete_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.delete_button.setFixedHeight(self.BUTTON_HEIGHT)
        self.delete_button.setFixedWidth(self.BUTTON_WIDTH)
        self.cols.addWidget(self.delete_button)

    def define_connections(self) -> None:
        pass


class AvailableLinksWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        # TODO

    def define_connections(self) -> None:
        pass
