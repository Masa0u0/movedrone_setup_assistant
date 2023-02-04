from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class ParamGetterWidget(QWidget):

    LABEL_PSIZE = 12
    LABEL_HEIGHT = 20
    DESCRIPTION_PSIZE = 9

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
    ) -> None:
        super().__init__()

        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        label = QLabel(param_name)
        label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        label.setFixedHeight(self.LABEL_HEIGHT)
        label.setAlignment(Qt.AlignTop)
        self.rows.addWidget(label)

        if description_text is not None:
            description = QLabel(description_text)
            description.setFont(QFont("Default", pointSize=self.DESCRIPTION_PSIZE))
            description.setAlignment(Qt.AlignTop)
            self.rows.addWidget(description)

    @abstractmethod
    def get(self):
        raise NotImplementedError()
