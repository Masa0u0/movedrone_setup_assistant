from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from .base_setting import BaseSettingWidget
from .file_browser import FileBrowserWidget


class StartWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        tab_text = 'MoveDrone Setup Assistant'
        abst_text = 'TODO'
        super().__init__(main, tab_text, abst_text)

        self.file_browser = FileBrowserWidget(main)
        self.rows.addWidget(self.file_browser)
