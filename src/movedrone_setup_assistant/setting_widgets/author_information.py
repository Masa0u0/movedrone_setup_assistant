from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ..setting_widgets.base_setting import BaseSettingWidget
from ..parameter_getters import *


class AuthorInformationWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = 'Specify Author Information'
        abst_text = 'TODO: abstruct'
        super().__init__(main, title_text, abst_text)

        self.name_getter = ParamGetterWidget_LineEdit("Name of the maintainer")
        self.rows.addWidget(self.name_getter)

        self.email_getter = ParamGetterWidget_LineEdit("Email of the maintainer")
        self.rows.addWidget(self.email_getter)

        self._add_dummy_widget()
