import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Control {
    id: root

    property alias title: headerText.text
    property bool expanded: true
    property int contentMargins: 12
    default property alias content: contentColumn.data

    readonly property int _headerHeight: 36
    readonly property int _radius: 0

    property int _contentImplicit: contentColumn.implicitHeight + 2 * contentMargins
    property int _targetHeight: expanded ? _contentImplicit : 0

    Layout.fillWidth: true
    Layout.preferredHeight: _headerHeight + _targetHeight
    Layout.minimumHeight: _headerHeight

    width: parent ? parent.width - 2 * contentMargins : 400
    implicitWidth: 400
    implicitHeight: _headerHeight + _targetHeight

    contentItem: Column {
        spacing: 0

        Rectangle {
            id: headerBar

            width: parent.width
            height: _headerHeight

            radius: _radius

            property bool hovered: false
            property color baseColor: "#2a2a2a"
            property color hoverColor: "#3a3a3a"

            color: hovered ? hoverColor : baseColor

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12

                spacing: 8

                Text {
                    id: headerText

                    text: "Section"
                    color: "#e0e0e0"
                    font.pixelSize: 14
                    font.bold: true
                    
                    elide: Text.ElideRight

                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }

                Text {
                    text: expanded ? "-" : "+"
                    color: "#cccccc"
                    font.pixelSize: 14

                    Layout.alignment: Qt.AlignVCenter
                }
            }

            MouseArea {
                anchors.fill: parent

                cursorShape: Qt.PointingHandCursor

                hoverEnabled: true
                
                onClicked: root.expanded = !root.expanded

                onEntered: headerBar.hovered = true
                onExited: headerBar.hovered = false
            }
        }

        Rectangle {
            id: contentDrawer

            //anchors.top: headerBar.bottom
            anchors.left: parent.left
            anchors.right: parent.right

            color: "transparent"

            clip: true

            height: root._targetHeight

            Behavior on height {
                NumberAnimation {
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }

            Item {
                anchors.fill: parent
                anchors.margins: contentMargins

                Column {
                    id: contentColumn

                    width: parent.width

                    spacing: 8
                }
            }
        }
    }
}