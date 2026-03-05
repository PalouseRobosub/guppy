import json, os, socket, threading, time

from threading import Condition

from guppy_teleop.frontend.widgets.widget import Widget

from rclpy.logging import get_logger

from PySide6.QtQml import QQmlApplicationEngine
from PySide6 import QtCore
from PySide6.QtCore import QObject

SOCKET_PATH = os.path.join("/tmp/terminal-ipc")

logger = get_logger(__name__)

class WidgetRegistry:
    def __init__(self, engine: QQmlApplicationEngine):
        self._engine = engine
        self.toast_manager: QObject = None

        self._widgets: dict[str, Widget] = {}
        self._condition: Condition = threading.Condition()

        self._client = None
        self._connect_to_backend()  

    def register(self, widget):
        with self._condition:
            self._widgets[widget.name] = widget
            self._condition.notify_all()
        
        widget._send_callback = self._send
    
        self._engine.rootContext().setContextProperty(widget.qml_name, widget)
    
    def _connect_to_backend(self):
        thread = threading.Thread(target=self._client_process, daemon=True)
        thread.start()
    
    def _client_process(self):
        if not os.path.exists(SOCKET_PATH):
            return
        
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(SOCKET_PATH)
            self._client = client

            buffer = ""
            while True:
                data = client.recv(1024).decode()
                if not data:
                    break
                
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)

                    try:
                        self._route(json.loads(line))
                    except json.JSONDecodeError:
                        logger.error("bad message sent")
    
    def _route(self, message: dict):
        type = message.get("type")
        name = message.get("name")
        payload = message.get("payload")

        logger.info(payload.__str__())

        # root commands, not for widgets
        if name == "root":
            if (type == "command"):
                self._handle_command(payload)
            return

        widget = self.wait_for_widget(name, timeout=1.0)
        if widget is None:
            logger.error(f"could not find widget {name}!")
            return
        
        if type in ("sync", "update"):
            widget.handle_update(payload)
            return
        
        if type == "response":
            widget.handle_response(payload)
            return

    def wait_for_widget(self, name, timeout=1.0):
        end_time = time.time() + timeout

        with self._condition:
            while name not in self._widgets:
                remaining = end_time - time.time()
                if remaining <= 0:
                    return None

                self._condition.wait(timeout=remaining)
            
            return self._widgets[name]
    
    def _send(self, message: dict):
        self._client.sendall((json.dumps(message)).encode())
    
    def _handle_command(self, payload: dict):
        action = payload.get("action")
        arguments = payload.get("arguments", {})

        if action == "send_toast":
            message = arguments.get("message")
            if not message:
                return
            
            del arguments["message"]

            with self.toast_manager as manager:
                manager.createMessage(message, arguments)