from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from .base_setting import BaseSettingWidget


class AuthorInformationWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        tab_text = 'Specify Author Information'
        abst_text = 'TODO: abstruct'
        super().__init__(main, tab_text, abst_text)

    def define_connections(self) -> None:
        pass
