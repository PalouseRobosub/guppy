from abc import abstractmethod

from PySide6.QtCore import QObject

class Widget(QObject):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._send_callback = None
    
    @property
    @abstractmethod
    def name(self) -> str: ...

    @property
    @abstractmethod
    def qml_name(self) -> str: ...

    @abstractmethod
    def handle_update(self, payload): ...

    def _send(self, action: str, arguments=None):
        if self._send_callback is None:
            return
        
        self._send_callback({
            "type": "command",
            "name": self.name,
            "payload": {"action": action, "arguments": arguments}
        })