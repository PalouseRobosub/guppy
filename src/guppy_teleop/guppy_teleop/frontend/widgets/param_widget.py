import os, json

from PySide6.QtCore import Property, Signal, Slot

from guppy_teleop.frontend.widgets.widget import Widget

from rclpy.logging import get_logger

SOCKET_PATH = os.path.join("/tmp/terminal-ipc")

class ParameterWidget(Widget):
    parametersChanged = Signal()

    @property
    def name(self) -> str:
        return "parameters"

    @property
    def qml_name(self) -> str:
        return "parameterWidget"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._parameters = {}
        get_logger("rclpy").info("widget created!")
    
    def handle_update(self, payload: dict):
        get_logger("rclpy").info("update!")
        self._parameters = payload.get("parameters", {})
        self.parametersChanged.emit()

    @Property("QVariantMap", notify=parametersChanged)
    def parameters(self):
        return self._parameters

    @Slot("QVariantMap")
    def pushParameters(self, params: dict):
        for key, value in params.items():
            dirty = False
            
            if isinstance(value, list):
                convert = [float(i) for i in self._parameters[key]]
                dirty = not value.__eq__(convert)
            else:
                dirty = not value.__eq__((type(value))(self._parameters[key]))

            if dirty:
                self._push_parameter(key, str(value), type(value).__name__)

    def _push_parameter(self, param_name: str, value: str, type: str):
        arguments = {
            "parameter": param_name,
            "value": value,
            "type": type
        }

        self._send("change_param", arguments)