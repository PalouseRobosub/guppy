import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

Rectangle {
    id: sidebar

    property int targetWidth: sidebar.expanded ? SidebarProperties.expandedWidth : SidebarProperties.collapsedWidth
    property bool expanded: true

    color: Theme.sidebarBackground

    Layout.preferredWidth: targetWidth

    Behavior on targetWidth {
        NumberAnimation {
            duration: SidebarProperties.animationDuration
            easing.type: Easing.OutCubic
        }
    }

    Column {
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }

        spacing: 0

        SidebarButton {
            iconSource: "qrc:/assets/icons/menu.svg"
            labelText: "Menu"
            onClicked: sidebar.expanded = !sidebar.expanded
        }

        Rectangle {
            width: parent.width
            height: 1
            color: Theme.sidebarDivider
        }

        Repeater {
            model: workspaceManager.workspaces

            delegate: SidebarButton {
                iconSource: modelData["icon"]
                labelText: modelData["name"] || modelData["id"]
                onClicked: workspaceManager.loadWorkspace(modelData["id"])
            }
        }

        Rectangle {
            width: parent.width
            height: 1
            color: Theme.sidebarDivider
        }
    }

    SidebarButton {
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }

        iconSource: "qrc:/assets/icons/cog.svg"
        labelText: "Settings"
        onClicked: console.log("TODO")
    }
}
