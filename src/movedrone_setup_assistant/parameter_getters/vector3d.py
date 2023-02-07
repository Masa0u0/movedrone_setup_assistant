from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import List, Tuple

from .base import ParamGetterWidget
from ..utils import DoubleSpinBox
from ..const import *


class ParamGetterWidget_Vector3d(ParamGetterWidget):

    value_changed = pyqtSignal(float, float, float)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        min: List[float] = [-1e-9] * 3,
        max: List[float] = [+1e+9] * 3,
        single_step: List[float] = [1.] * 3,
        default: List[float] = [None] * 3,
        suffix: str = "",
    ) -> None:
        assert len(min) == 3
        assert len(max) == 3
        assert len(single_step) == 3
        assert len(default) == 3

        super().__init__(param_name, description_text)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._x = DoubleGetter("x", min[0], max[0], single_step[0], default[0], suffix)
        self._cols.addWidget(self._x)

        self._y = DoubleGetter("y", min[1], max[1], single_step[1], default[1], suffix)
        self._cols.addWidget(self._y)

        self._z = DoubleGetter("z", min[2], max[2], single_step[2], default[2], suffix)
        self._cols.addWidget(self._z)

        self._x.data.valueChanged.connect(self._on_value_changed)
        self._y.data.valueChanged.connect(self._on_value_changed)
        self._z.data.valueChanged.connect(self._on_value_changed)

    def get(self) -> Tuple[float, float, float]:
        return self._x.get(), self._y.get(), self._z.get()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        self.value_changed.emit(self._x.get(), self._y.get(), self._z.get())


class DoubleGetter(QWidget):

    def __init__(
        self,
        name: str,
        min: float,
        max: float,
        single_step: float,
        default: float,
        suffix: str,
    ) -> None:
        assert min < max
        assert single_step > 0.
        
        super().__init__()

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        label = QLabel(name + ":")
        label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        label.setAlignment(Qt.AlignRight)
        self._cols.addWidget(label)

        self.data = DoubleSpinBox()
        self.data.setMinimum(min)
        self.data.setMaximum(max)
        self.data.setSingleStep(single_step)
        if default is not None:
            assert min <= default <= max
            self.data.setValue(default)
        self.data.setSuffix(suffix)
        self._cols.addWidget(self.data)

    def get(self) -> float:
        return self.data.value()
