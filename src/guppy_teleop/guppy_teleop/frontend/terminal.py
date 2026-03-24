import rclpy, threading, sys
from rclpy.executors import MultiThreadedExecutor

from pathlib import Path

from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6 import QtCore
from PySide6.QtCore import QObject

from guppy_teleop.frontend import build
from guppy_teleop.frontend.workspace_manager import WorkspaceManager
from guppy_teleop.frontend.widgets.state_widget import StateWidget
from guppy_teleop.frontend.widgets.param_widget import ParameterWidget
from guppy_teleop.frontend.widgets.toast_widget import ToastWidget
import guppy_teleop.frontend.rc_assets

def main(args = None):
    print(QtCore.__version__)
    
    build.run()

    rclpy.init(args = args)

    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    manager = WorkspaceManager()
    engine.rootContext().setContextProperty("workspaceManager", manager)

    widgets = []

    widgets.append(StateWidget())
    widgets.append(ParameterWidget())
    toast_widget = ToastWidget()

    executor = MultiThreadedExecutor()
    for widget in widgets:
        engine.rootContext().setContextProperty(widget.qml_name, widget)
        executor.add_node(widget)

    engine.addImportPath(str(Path(__file__).parent))
    engine.loadFromModule("ui", "Main")

    if not engine.rootObjects():
        print("something went pretty wrong")
        sys.exit(-1)
    
    root = None
    for object in engine.rootObjects():
        if object.objectName() == "window":
            root = object
            break

    toast_widget.toastManager = root.findChild(QObject, "toastManager")
    executor.add_node(toast_widget)
    
    ros_thread = threading.Thread(target = executor.spin, daemon = True)
    ros_thread.start()

    try:
        app.exec()
    except:
        print("app exited")

    del engine

if __name__ == "__main__":
    main()