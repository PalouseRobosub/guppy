import QtQuick
import ui

CanvasWidget {
    id: root

    title: widgetLabel !== "" ? widgetLabel : "Text Widget"

    property string widgetLabel: ""
    property string bodyText: "set bodyText in json description"

    Rectangle {
        anchors.fill: parent
        color: "transparent"

        Text {
            anchors {
                fill: parent
                margins: 12
            }

            text: root.bodyText
            color: "#cccccc"
            font.pixelSize: 13

            wrapMode: Text.Wrap
            verticalAlignment: Text.AlignTop
        }
    }
}
