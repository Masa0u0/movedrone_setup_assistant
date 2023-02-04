from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .base_setting import BaseSettingWidget
from .parameter_getters import *


class ControllersWidget(BaseSettingWidget):

    LMPC_LABEL = "Linear Model Predictive Control"
    NMPC_LABEL = "Nonlinear Model Predictive Control"
    SMC_LABEL = "Model Following Sliding Mode Control"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Setup Controllers'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        type_description = "TODO: instruction"
        self.controller_type_getter = ParamGetterWidget_ComboBox(
            "Type of Controller",
            type_description,
            [self.LMPC_LABEL, self.NMPC_LABEL, self.SMC_LABEL],
        )
        self.rows.addWidget(self.controller_type_getter)

        self.lmpc_settings = LMPCSettingsWidget(main)
        self.rows.addWidget(self.lmpc_settings)

        self.nmpc_settings = NMPCSettingsWidget(main)
        self.rows.addWidget(self.nmpc_settings)

        self.smc_settings = SMCSettingsWidget(main)
        self.rows.addWidget(self.smc_settings)

        self._add_dummy_widget()

        self._update_controllers_visibility()

    def define_connections(self) -> None:
        self.controller_type_getter.text_changed.connect(self._on_type_changed)

    def _update_controllers_visibility(self) -> None:
        controller_type = self.controller_type_getter.get()

        if controller_type == self.LMPC_LABEL:
            self.lmpc_settings.setVisible(True)
            self.nmpc_settings.setVisible(False)
            self.smc_settings.setVisible(False)
        elif controller_type == self.NMPC_LABEL:
            self.lmpc_settings.setVisible(False)
            self.nmpc_settings.setVisible(True)
            self.smc_settings.setVisible(False)
        elif controller_type == self.SMC_LABEL:
            self.lmpc_settings.setVisible(False)
            self.nmpc_settings.setVisible(False)
            self.smc_settings.setVisible(True)
        else:
            raise RuntimeError(f'Invalid controller type: {controller_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, controller_type: str) -> None:
        self._update_controllers_visibility()


class LMPCSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        abst_text = 'TODO: abstruct of LMPC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BaseSettingWidget.BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self.rows.addWidget(abst)

        # TODO


class NMPCSettingsWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        abst_text = 'TODO: abstruct of NMPC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BaseSettingWidget.BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self.rows.addWidget(abst)

        # TODO


class SMCSettingsWidget(QWidget):
    """ モデルフォロイング型スライディングモード制御の設定(cf. 「ドローン工学入門」,p.189) """

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self.main = main

        self.rows = QVBoxLayout()
        self.setLayout(self.rows)

        abst_text = 'TODO: abstruct of SMC'
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BaseSettingWidget.BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        self.rows.addWidget(abst)

        # TODO
