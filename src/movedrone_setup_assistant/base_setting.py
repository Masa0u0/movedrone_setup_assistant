from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class BaseSettingWidget(QWidget):
    
    TITLE_PSIZE = 18
    ABST_PSIZE = 9

    def __init__(self, main: SetupAssistant, title_text: str, abst_text: str) -> None:
        super().__init__()
        self.main = main
        
        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        title = QLabel(title_text)
        title.setFont(QFont('Default', pointSize=self.TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self.rows.addWidget(title)

        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=self.ABST_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self.rows.addWidget(abst)

    @abstractmethod
    def define_connections(self) -> None:
        raise NotImplementedError()
