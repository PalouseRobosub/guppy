import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import QtQuick.Layouts
import QtQuick.Effects
import ui

Rectangle {
    id: sidebar

    property int targetWidth: sidebar.expanded ? SidebarProperties.expandedWidth : SidebarProperties.collapsedWidth
    property bool expanded: true

    color: "#1e1e1e"

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

        Rectangle {
            width: parent.width
            height: SidebarProperties.itemHeight

            color: closeArea.containsMouse ? "#2a2a2a" : "transparent"

            clip: true

            MouseArea {
                id: closeArea

                anchors.fill: parent

                hoverEnabled: true
                onClicked: sidebar.expanded = !sidebar.expanded
            }

            Row {
                anchors {
                    left: parent.left
                    leftMargin: SidebarProperties.leftMargin
                    verticalCenter: parent.verticalCenter
                }

                spacing: 20

                Item {
                    width: SidebarProperties.iconSize
                    height: SidebarProperties.iconSize

                    VectorImage {
                        id: menuIcon

                        source: "qrc:/assets/icons/menu.svg"

                        anchors.fill: parent
                        layer.enabled: true
                        visible: false

                        fillMode: VectorImage.PreserveAspectFit
                        smooth: true
                    }

                    MultiEffect {
                        source: menuIcon
                        anchors.fill: parent
                        colorization: 1.0
                        colorizationColor: SidebarProperties.iconColor
                    }
                }

                Text {
                    text: "Menu"
                    color: "#cccccc"
                    font.pixelSize: 13

                    anchors.verticalCenter: parent.verticalCenter
                }
            }
        }

        Rectangle {
            width: parent.width
            height: 1

            color: "#2e2e2e"
        }

        Repeater {
            model: workspaceManager.workspaces

            delegate: Rectangle {
                width: sidebar.width
                height: SidebarProperties.itemHeight

                color: workspaceArea.containsMouse ? "#252525" : "transparent"

                clip: true

                MouseArea {
                    id: workspaceArea

                    anchors.fill: parent

                    hoverEnabled: true
                    onClicked: workspaceManager.loadWorkspace(modelData["id"])
                }

                Row {
                    anchors {
                        left: parent.left
                        leftMargin: SidebarProperties.leftMargin
                        verticalCenter: parent.verticalCenter
                    }

                    spacing: 20

                    Item {
                        width: SidebarProperties.iconSize
                        height: SidebarProperties.iconSize

                        VectorImage {
                            id: workspaceIcon

                            source: modelData["icon"]

                            anchors.fill: parent
                            layer.enabled: true

                            fillMode: VectorImage.PreserveAspectFit
                            smooth: true
                        }

                        MultiEffect {
                            source: workspaceIcon
                            anchors.fill: parent
                            colorization: 1.0
                            colorizationColor: SidebarProperties.iconColor
                        }
                    }

                    Text {
                        text: modelData["name"] || modelData["id"]
                        color: "#cccccc"
                        font.pixelSize: 13

                        anchors.verticalCenter: parent.verticalCenter

                        elide: Text.ElideRight
                    }
                }
            }
        }

        Rectangle {
            width: parent.width
            height: 1

            color: "#2e2e2e"
        }
    }

    Rectangle {
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }

        height: SidebarProperties.itemHeight

        color: settingsArea.containsMouse ? "#2a2a2a" : "transparent"

        clip: true

        MouseArea {
            id: settingsArea

            anchors.fill: parent

            hoverEnabled: true
            onClicked: console.log("TODO")
        }

        Row {
            anchors {
                left: parent.left
                leftMargin: SidebarProperties.leftMargin
                verticalCenter: parent.verticalCenter
            }

            spacing: 20

            Item {
                width: SidebarProperties.iconSize
                height: SidebarProperties.iconSize

                VectorImage {
                    id: settingsIcon

                    source: "qrc:/assets/icons/cog.svg"

                    anchors.fill: parent
                    layer.enabled: true

                    fillMode: VectorImage.PreserveAspectFit
                    smooth: true
                }

                MultiEffect {
                    source: settingsIcon
                    anchors.fill: parent
                    colorization: 1.0
                    colorizationColor: SidebarProperties.iconColor
                }
            }

            Text {
                text: "Settings"
                color: "#cccccc"
                font.pixelSize: 13

                anchors.verticalCenter: parent.verticalCenter
            }
        }
    }
}
