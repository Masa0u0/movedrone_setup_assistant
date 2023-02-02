from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from .base_setting import BaseSettingWidget


class ImuWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        tab_text = 'Define Inertial Measurement Unit'
        abst_text = 'TODO'
        super().__init__(main, tab_text, abst_text)

    def define_connections(self) -> None:
        pass
