import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import ui

Button {
    id: button

    width: 36
    height: 36
    
    opacity: 0.9

    property url iconSource: ""
    property int iconSize: 22
    property string tooltip: ""

    background: Rectangle {
        color: hovered ? Theme.circleButtonHover : Theme.circleButton

        radius: width / 2
    }

    contentItem: VectorImage {
        anchors.centerIn: parent

        width: button.iconSize
        height: button.iconSize

        source: button.iconSource !== undefined ? button.iconSource : ""

        visible: button.iconSource !== ""

        fillMode: VectorImage.PreserveAspectFit
        smooth: true
    }

    ToolTip.visible: tooltip.length > 0 && hovered
    ToolTip.delay: 300
    ToolTip.text: tooltip
}