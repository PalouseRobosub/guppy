import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import ui

Rectangle {
    id: control

    width: 36
    height: 36

    radius: width / 2

    color: hovered ? "#333333" : "#2A2A2A"
    
    opacity: 0.9

    property url iconSource: ""
    signal clicked()
    property bool hovered: false

    property int iconSize: 22
    property string tooltip: ""

    VectorImage {
        anchors.centerIn: parent

        width: control.iconSize
        height: control.iconSize

        source: control.iconSource !== undefined ? control.iconSource : ""

        visible: control.iconSource !== ""

        fillMode: VectorImage.PreserveAspectFit
        smooth: true
    }

    MouseArea {
        anchors.fill: parent

        hoverEnabled: true

        onEntered: control.hovered = true
        onExited: control.hovered = false

        onClicked: control.clicked()
    }

    ToolTip.visible: tooltip.length > 0 && hovered
    ToolTip.delay: 300
    ToolTip.text: tooltip
}