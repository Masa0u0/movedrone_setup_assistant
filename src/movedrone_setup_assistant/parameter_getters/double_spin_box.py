from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base import ParamGetterWidget


class DoubleSpinBox(QDoubleSpinBox):
    """ QDoubleSpineBoxのスクロールイベントを無効化したもの． """

    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class ParamGetterWidget_DoubleSpinBox(ParamGetterWidget):

    value_changed = pyqtSignal(float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        min: float = -1e-9,
        max: float = +1e+9,
        single_step: float = 1.,
        default: float = None,
        suffix: str = "",
    ) -> None:
        assert min < max
        assert single_step > 0.

        super().__init__(param_name, description_text)

        self.spin_box = DoubleSpinBox()
        self._rows.addWidget(self.spin_box)

        self.spin_box.setMinimum(min)
        self.spin_box.setMaximum(max)
        self.spin_box.setSingleStep(single_step)
        if default is not None:
            assert min <= default <= max
            self.spin_box.setValue(default)
        self.spin_box.setSuffix(suffix)

        self.spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> float:
        return self.spin_box.value()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(value)
