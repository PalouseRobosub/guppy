import sys
from pathlib import Path

from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6 import QtCore

from guppy_teleop.frontend.widget_registry import WidgetRegistry
from guppy_teleop.frontend.workspace_manager import WorkspaceManager
from guppy_teleop.frontend.widgets.state_widget import StateWidget
from guppy_teleop.frontend.widgets.param_widget import ParameterWidget
import guppy_teleop.frontend.rc_assets

def main(args = None):
    print(QtCore.__version__)

    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    widget_registry = WidgetRegistry(engine)

    # nodes here

    widget_registry.register(StateWidget())
    widget_registry.register(ParameterWidget())

    # nodes end

    manager = WorkspaceManager()
    engine.rootContext().setContextProperty("workspaceManager", manager)

    engine.addImportPath(str(Path(__file__).parent))
    engine.loadFromModule("ui", "Main")

    if not engine.rootObjects():
        print("something went pretty wrong")
        sys.exit(-1)

    try:
        app.exec()
    except:
        print("app exited")

    del engine

if __name__ == "__main__":
    main()