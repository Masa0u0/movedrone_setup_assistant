from typing import List
from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class ParamGetterWidget(QWidget):

    LABEL_PSIZE = 12
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

        self.spin_box = QSpinBox()
        self.rows.addWidget(self.spin_box)

        self.spin_box.setMinimum(min)
        self.spin_box.setMaximum(max)
        self.spin_box.setSingleStep(single_step)
        if default is not None:
            assert min <= default <= max
            self.spin_box.setValue(default)
        self.spin_box.setSuffix(suffix)

        self.spin_box.valueChanged.connect(self._on_value_changed)

    def get(self) -> int:
        return self.spin_box.value()

    @pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        self.value_changed.emit(value)
        
class ParamGetterWidget_DoubleSpinBox(ParamGetterWidget):

    value_changed = pyqtSignal(float)

    def __init__(
        self,
        param_name: str,
        description_text: str,
        min: float = -1e-9,
        max: float = +1e+9,
        single_step: float = 1.,
        default: float = None,
        suffix: str = "",
    ) -> None:
        assert min < max
        assert single_step > 0.

        super().__init__(param_name, description_text)

        self.spin_box = QDoubleSpinBox()
        self.rows.addWidget(self.spin_box)

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


class ParamGetterWidget_LineEdit(ParamGetterWidget):

    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str,
        default: str = "",
    ) -> None:
        super().__init__(param_name, description_text)

        self.line = QLineEdit()
        self.rows.addWidget(self.line)
        
        self.line.setText(default)

        self.line.textChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self.line.text()

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)


class ParamGetterWidget_ComboBox(ParamGetterWidget):

    text_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str,
        choices: List[str],
    ) -> None:
        super().__init__(param_name, description_text)

        self.box = QComboBox()
        for choice in choices:
            self.box.addItem(choice)
        self.rows.addWidget(self.box)

        self.box.currentTextChanged.connect(self._on_text_changed)

    def get(self) -> str:
        return self.box.currentText()

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.text_changed.emit(text)
