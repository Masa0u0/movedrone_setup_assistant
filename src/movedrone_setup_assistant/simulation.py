from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtWidgets import QWidget


class SimulationWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self.main = main

        # TODO
