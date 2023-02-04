from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget


class ComboBox(QComboBox):
    """ QComboBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class ParamGetterWidget_ComboBox(ParamGetterWidget):

    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str,
        choices: List[str],
    ) -> None:
        super().__init__(param_name, description_text)

        self.box = ComboBox()
        for choice in choices:
            self.box.addItem(choice)
        self.rows.addWidget(self.box)

        self.box.currentTextChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self.box.currentText()

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
