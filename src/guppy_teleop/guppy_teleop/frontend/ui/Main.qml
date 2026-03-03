import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: root

    width: 1400
    height: 800

    visible: true

    title: "Guppy Teleop"
    color: "#181818"

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Sidebar {
            id: sidebar

            Layout.fillHeight: true
        }

        WorkspaceCanvas {
            id: canvas

            manager: workspaceManager

            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    Connections {
        target: workspaceManager

        function onWorkspaceLoaded(data) {
            canvas.loadWorkspace(data)
        }
    }

    Component.onCompleted: {
        workspaceManager.loadWorkspace("default")

        console.log("loaded!")
    }
}
