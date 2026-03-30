from pathlib import Path
import json
from importlib.resources import files

WORKSPACES_PATH = files("guppy_teleop.frontend.workspaces")

from PySide6.QtCore import QObject, Signal, Slot, Property

class WorkspaceManager(QObject):
    workspaceLoaded = Signal("QVariantMap")
    workspacesChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._workspaces = self._scan_workspaces()

    @Property("QVariantList", notify=workspacesChanged)
    def workspaces(self):
        return self._workspaces

    @Slot(str)
    def loadWorkspace(self, workspace_id: str):
        self.loadWorkspaceFromUrl(WORKSPACES_PATH.joinpath(f"{workspace_id}.json"))
    
    @Slot(str)
    def loadWorkspaceFromUrl(self, filepath):
        path = Path(filepath)

        try:
            with open(path) as file:
                data = json.load(file)
            
            self.workspaceLoaded.emit(data)
        except FileNotFoundError:
            print("workspace not found!")
        except json.JSONDecodeError as err:
            print(f"bad json in {path}: {err}")

    @Slot()
    def refreshWorkspaces(self):
        self._workspaces = self._scan_workspaces()

        self.onWorkspacesChanged.emit()

    def _scan_workspaces(self) -> list[dict]:
        entries = []

        for filepath in WORKSPACES_PATH.glob("*.json"):
            stem = Path(filepath).stem

            try:
                with open(filepath) as f:
                    meta = json.load(f)
            except Exception:
                meta = {}

            entries.append({
                "id": stem,
                "name": meta.get("name", stem.replace("_", " ").title()),
                "icon": meta.get("icon", " ")
            })

        return entries

    @Slot(str, "QVariantMap")
    def saveWorkspace(self, filepath, children: dict):
        print(f"path: {filepath}")
        print(f"dict: {children}")