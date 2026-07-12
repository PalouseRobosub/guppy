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
    readonly property int _settledHeight: _headerHeight + (expanded ? contentColumn.implicitHeight + 2 * contentMargins : 0)

    width: parent ? parent.width : 0
    implicitHeight: _settledHeight

    Layout.fillWidth: true
    Layout.preferredHeight: _settledHeight

    contentItem: Item {
        implicitHeight: root._settledHeight

        Button {
            id: headerButton

            anchors {
                left: parent.left
                right: parent.right
                top: parent.top
            }
            height: _headerHeight

            onClicked: root.expanded = !root.expanded


            background: Rectangle {
                color: hovered ? root.palette.midlight : root.palette.mid
            }

            contentItem: RowLayout {
                anchors {
                    fill: parent
                    leftMargin: 12
                    rightMargin: 12
                }
                spacing: 8

                Text {
                    id: headerText

                    text: "Section"
                    color: root.palette.windowText
                    font.pixelSize: 14
                    font.bold: true
                    elide: Text.ElideRight

                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }

                Text {
                    text: root.expanded ? "\u2212" : "+"
                    color: root.palette.windowText
                    font.pixelSize: 14

                    Layout.alignment: Qt.AlignVCenter
                }
            }
        }

        Rectangle {
            id: contentDrawer

            anchors {
                left: parent.left
                right: parent.right
                top: headerButton.bottom
            }

            color: "transparent"
            clip: true

            height: expanded
                ? contentColumn.implicitHeight + 2 * contentMargins
                : 0

            Behavior on height {
                NumberAnimation {
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }

            Column {
                id: contentColumn

                anchors {
                    left: parent.left
                    right: parent.right
                    top: parent.top
                    margins: root.contentMargins
                }
                spacing: 8
            }
        }
    }
}
