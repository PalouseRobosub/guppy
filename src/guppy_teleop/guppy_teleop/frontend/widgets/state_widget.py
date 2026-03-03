import os

from PySide6.QtCore import Property, Signal, Slot

from guppy_teleop.frontend.widgets.widget import Widget

SOCKET_PATH = os.path.join("/tmp/terminal-ipc")

class StateWidget(Widget):
    stateChanged = Signal()

    @property
    def name(self) -> str:
        return "state"

    @property
    def qml_name(self) -> str:
        return "stateWidget"    

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = "not recieved"
    
    def handle_update(self, payload: dict):
        self._state = payload.get("state", "unknown")
        self.stateChanged.emit()

    @Property(str, notify=stateChanged)
    def state(self) -> str:
        return self._state

    @Slot(str)
    def pushState(self, new_state: str):
        arguments = {"new_state" : new_state}
        self._send("change_state", arguments)