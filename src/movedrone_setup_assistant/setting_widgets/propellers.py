from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..setting_widgets.base_setting import BaseSettingWidget


class PropellersWidget(BaseSettingWidget):

    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Define Propellers'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        propellers_label = QLabel("Propellers")
        propellers_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        propellers_label.setAlignment(Qt.AlignLeft)
        self.rows.addWidget(propellers_label)

        self.selected = SelectedPropellersWidget(main)
        self.rows.addWidget(self.selected)

        self.add_delete = AddDeleteButtonsWidget(main)
        self.rows.addWidget(self.add_delete)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self.rows.addWidget(links_label)

        self.available_links = AvailableLinksWidget(main)
        self.rows.addWidget(self.available_links)

    def define_connections(self) -> None:
        super().define_connections()
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

    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        self.cols = QHBoxLayout()
        self.setLayout(self.cols)

        self.add_button = QPushButton("⬆")
        self.add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self.cols.addWidget(self.add_button)

        self.delete_button = QPushButton("⬇")
        self.delete_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self.cols.addWidget(self.delete_button)

    def define_connections(self) -> None:
        pass


class AvailableLinksWidget(QListWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

    def define_connections(self) -> None:
        self.main.urdf_parser.robot_model_updated.connect(self._add_available_links)

    @pyqtSlot()
    def _add_available_links(self) -> None:
        root_link = self.main.urdf_parser.get_root()

        for link in self.main.urdf_parser.get_links():
            if link.name == root_link.name:
                continue

            joint = self.main.urdf_parser.get_joint(link.name)
            if joint.type == "continuous":
                self.addItem(QListWidgetItem(link.name))
