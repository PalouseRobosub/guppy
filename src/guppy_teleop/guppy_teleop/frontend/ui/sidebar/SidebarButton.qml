import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import QtQuick.Effects
import ui

Button {
    id: sidebarButton

    property string iconSource: ""
    property string labelText: ""
    property color labelColor: Theme.textPrimary

    implicitWidth: parent ? parent.width : 0
    implicitHeight: SidebarProperties.itemHeight

    clip: true

    background: Rectangle {
        color: sidebarButton.hovered ? Theme.sidebarButtonHover : Theme.sidebarButton
    }

    contentItem: Item {
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
                anchors.verticalCenter: parent.verticalCenter

                visible: iconSource !== ""

                VectorImage {
                    id: buttonIcon

                    source: sidebarButton.iconSource

                    anchors.fill: parent
                    layer.enabled: true
                    visible: false

                    fillMode: VectorImage.PreserveAspectFit
                    smooth: true
                }

                MultiEffect {
                    source: buttonIcon
                    anchors.fill: parent
                    colorization: 1.0
                    colorizationColor: SidebarProperties.iconColor
                }
            }

            Text {
                text: sidebarButton.labelText
                color: sidebarButton.labelColor
                font.pixelSize: 13

                anchors.verticalCenter: parent.verticalCenter

                elide: Text.ElideRight
            }
        }
    }
}
