from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget


class LineEdit(QLineEdit):
    """ QLineEditのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class ParamGetterWidget_LineEdit(ParamGetterWidget):

    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        default: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        self.line = LineEdit()
        self.rows.addWidget(self.line)

        self.line.setText(default)

        self.line.textChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self.line.text()

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
