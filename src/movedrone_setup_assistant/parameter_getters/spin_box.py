from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget


class SpinBox(QSpinBox):
    """ QSpineBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class ParamGetterWidget_SpinBox(ParamGetterWidget):

    value_changed = pyqtSignal(int)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        min: int = 0,
        max: int = +10**9,
        single_step: int = 1,
        default: int = None,
        suffix: str = "",
    ) -> None:
        assert min < max
        assert single_step > 0

        super().__init__(param_name, description_text)

        self.spin_box = SpinBox()
        self._rows.addWidget(self.spin_box)

        self.spin_box.setMinimum(min)
        self.spin_box.setMaximum(max)
        self.spin_box.setSingleStep(single_step)
        if default is not None:
            assert min <= default <= max
            self.spin_box.setValue(default)
        self.spin_box.setSuffix(suffix)

        self.spin_box.setFocusPolicy(Qt.StrongFocus)

        self.spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> int:
        return self.spin_box.value()

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(value)
