import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

CanvasWidget {
    id: parameterWidgetUI
    title: "Parameters"

    property var params: parameterWidget.parameters
    property bool readonly: true

    ScrollView {
        anchors.fill: parent
        contentWidth: availableWidth - 24

        Column {
            width: parent.width
            spacing: 12
            padding: 12


            move: Transition {
                NumberAnimation {
                    properties: "x,y"
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }

            MotorSection {
                params: parameterWidgetUI.params

                width: parent.width
            }

            PIDSection {
                params: parameterWidgetUI.params

                width: parent.width
            }


            DragSection {
                params: parameterWidgetUI.params

                width: parent.width
            }

            PhysicalSection {
                params: parameterWidgetUI.params

                width: parent.width
            }
        }
    }
}
