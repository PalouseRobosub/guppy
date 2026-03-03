import QtQuick
import QtQuick.VectorImage

Item {
    id: root

    property string title: "Widget"
    property bool selected: false
    default property alias content: contentArea.data

    property bool snapToGrid: false
    property int gridStep: 40

    signal closeRequested()

    width: 300
    height: 200

    property real minWidth: 160
    property real minHeight: 100

    readonly property int titleHeight: 28
    readonly property int cornerRadius: 8
    readonly property color accentColor: "#3daee9"

    readonly property color frameColor: "#2b2b2b"
    readonly property color titleColor: selected ? "#184a6b" : "#313131"
    readonly property color borderColor: selected ? accentColor : "#404040"

    Rectangle {
        id: frame

        anchors.fill: parent

        radius: cornerRadius

        color: frameColor
        border.color: borderColor
        border.width: selected ? 2 : 1

        clip: true

        Behavior on border.color {
            ColorAnimation {
                duration: 120
            }
        }

        Behavior on border.width {
            NumberAnimation {
                duration: 120
            }
        }

        Rectangle {
            id: titleBar

            width: parent.width
            height: titleHeight

            topLeftRadius: 4
            topRightRadius: 4

            color: titleColor

            Behavior on color {
                ColorAnimation {
                    duration: 120
                } 
            }

            DragHandler {
                target: null

                property real lastX: 0
                property real lastY: 0

                onActiveChanged: {
                    if (active) {
                        lastX = 0
                        lastY = 0
                    } else if (root.snapToGrid) {
                        root.x = Math.round(root.x / root.gridStep) * root.gridStep
                        root.y = Math.round(root.y / root.gridStep) * root.gridStep
                    }
                }

                onTranslationChanged: {
                    const s = (root.parent && root.parent.scale) ? root.parent.scale : 1.0

                    const dx = (activeTranslation.x - lastX) / s
                    const dy = (activeTranslation.y - lastY) / s

                    root.x += dx
                    root.y += dy

                    lastX = activeTranslation.x
                    lastY = activeTranslation.y
                }
            }

            Text {
                anchors {
                    left: parent.left
                    right: closeButton.left

                    leftMargin: 12
                    rightMargin: 8
                    
                    verticalCenter: parent.verticalCenter
                }

                text: root.title
                color: "#dddddd"
                font.pixelSize: 14
                font.weight: Font.Medium
                elide: Text.ElideRight
            }

            Rectangle {
                id: closeButton

                width: 20
                height: 20

                radius: 4

                anchors {
                    right: parent.right
                    rightMargin: 10
                    verticalCenter: parent.verticalCenter
                }

                color: closeArea.containsMouse ? "#ff0000" : "transparent"

                MouseArea {
                    id: closeArea

                    anchors.fill: parent

                    hoverEnabled: true

                    onClicked: root.closeRequested()
                }

                Behavior on color {
                    ColorAnimation {
                        duration: 100
                    }
                }

                VectorImage {
                    id: closeIcon

                    source: "qrc:/assets/icons/close.svg"

                    anchors.fill: parent
                    layer.enabled: true

                    fillMode: VectorImage.PreserveAspectFit
                    smooth: true
                }
            }

            Rectangle {
                anchors.bottom: parent.bottom

                width: parent.width
                height: 1

                color: "#00000040"
            }
        }

        Item {
            id: contentArea

            anchors {
                top: titleBar.bottom
                left: parent.left
                right: parent.right
                bottom: parent.bottom
            }
        }
    }

    Item {
        id: resizeGrip

        width: 16
        height: 16

        anchors {
            right: parent.right
            bottom: parent.bottom
        }

        opacity: gripArea.containsMouse ? 0.6 : 0.15

        Behavior on opacity {
            NumberAnimation {
                duration: 120
            }
        }

        Rectangle {
            anchors.fill: parent

            radius: 3

            border.color: "#666"
            border.width: 1
            color: "transparent"
        }

        MouseArea {
            id: gripArea

            anchors.fill: parent

            hoverEnabled: true

            cursorShape: Qt.SizeFDiagCursor
        }

        DragHandler {
            target: null

            property real lastX: 0
            property real lastY: 0

            onActiveChanged: {
                if (active) {
                    lastX = 0
                    lastY = 0
                } else if (root.snapToGrid) {
                    root.width = Math.max(root.minWidth, Math.round(root.width / root.gridStep) * root.gridStep)
                    root.height = Math.max(root.minHeight, Math.round(root.height / root.gridStep) * root.gridStep)
                }
            }

            onTranslationChanged: {
                const s = (root.parent && root.parent.scale) ? root.parent.scale : 1.0
                
                const dx = (activeTranslation.x - lastX) / s
                const dy = (activeTranslation.y - lastY) / s

                root.width  = Math.max(root.minWidth, root.width + dx)
                root.height = Math.max(root.minHeight, root.height + dy)

                lastX = activeTranslation.x
                lastY = activeTranslation.y
            }
        }
    }
}
