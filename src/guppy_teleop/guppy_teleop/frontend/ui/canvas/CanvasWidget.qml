import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import ui

Item {
    id: root

    property string title: "Widget"
    property bool selected: false
    default property alias content: contentArea.data

    property real canvasZoom: 1.0
    property bool snapToGrid: false
    property int gridStep: 40

    signal closeRequested()

    width: 300
    height: 200

    property real minWidth: 160
    property real minHeight: 100

    readonly property int titleHeight: 28
    readonly property int cornerRadius: 8

    Rectangle {
        id: frame

        anchors.fill: parent
        radius: cornerRadius
        color: Theme.widgetBackground
        clip: true

        border.color: Theme.widgetBorder

        Rectangle {
            id: titleBar

            width: parent.width
            height: titleHeight

            topLeftRadius: 4
            topRightRadius: 4
            color: Theme.widgetTitleBar

            DragHandler {
                target: root

                grabPermissions: PointerHandler.CanTakeOverFromAnything

                cursorShape: active ? Qt.ClosedHandCursor : Qt.ArrowCursor

                onActiveChanged: {
                    if (!active && root.snapToGrid) {
                        root.x = Math.round(root.x / root.gridStep) * root.gridStep
                        root.y = Math.round(root.y / root.gridStep) * root.gridStep
                    }
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
                color: Theme.textPrimary
                font.pixelSize: 14
                font.weight: Font.Medium
                elide: Text.ElideRight
            }

            Button {
                id: closeButton

                width: 20
                height: 20

                anchors {
                    right: parent.right
                    rightMargin: 10
                    verticalCenter: parent.verticalCenter
                }

                background: Rectangle {
                    anchors.fill: parent
                    radius: 4
                    color: closeButton.hovered ? Theme.semanticError : "transparent"
                }

                contentItem: VectorImage {
                    source: "qrc:/assets/icons/close.svg"

                    anchors.fill: parent
                    layer.enabled: true

                    fillMode: VectorImage.PreserveAspectFit
                    smooth: true
                }

                onClicked: root.closeRequested()
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

        anchors.right: parent.right
        anchors.bottom: parent.bottom

        opacity: gripArea.containsMouse ? 0.6 : 0.15
        Behavior on opacity { NumberAnimation { duration: 120 } }

        Rectangle {
            anchors.fill: parent
            radius: 3
            border.color: Theme.resizeGrip
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
                const dx = (activeTranslation.x - lastX) / root.canvasZoom
                const dy = (activeTranslation.y - lastY) / root.canvasZoom

                root.width  = Math.max(root.minWidth, root.width + dx)
                root.height = Math.max(root.minHeight, root.height + dy)

                lastX = activeTranslation.x
                lastY = activeTranslation.y
            }
        }
    }
}
