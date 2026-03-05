import QtQuick
import QtQuick.Controls
import ui

CanvasWidget {
    id: stateWidgetUI
    title: "State"

    property string currentState: stateWidget.state
    property var availableStates: ["STARTUP", "HOLDING", "NAV", "TASK", "TELEOP", "DISABLED", "FAULT"]
    property string outputMessage: ""
    property bool readonly: true

    Column {
        anchors.fill: parent
        spacing: 32
        padding: 12

        Column {
            spacing: 0

            Text {
                text: "Current State: "
                color: "#cccccc"
                font.pixelSize: 14

                wrapMode: Text.NoWrap
                elide: Text.ElideRight
            }

            Text {
                text: stateWidgetUI.currentState
                color: "#cccccc"
                font.pixelSize: 20

                wrapMode: Text.NoWrap
                elide: Text.ElideRight
            }

        }

        Row {
            spacing: 8

            ComboBox {
                id: stateCombo

                enabled: !stateWidgetUI.readonly

                model: stateWidgetUI.availableStates
                currentIndex: 0

                width: 150
            }

            Button {
                text: "Push"
                
                enabled: !stateWidgetUI.readonly
                
                onClicked: {
                    stateWidget.pushState(stateCombo.currentText)
                    var success = true // response in the future?

                    if (success)
                        toastManager.createMessage("Pushed state change to guppy.", {
                            type: "success",
                            position: Qt.TopRightCorner,
                            theme: "Dark",
                            closeOnClick: true,
                            autoClose: 5000,
                            hideProgressBar: false,
                        });
                    else
                        toastManager.createMessage("Failed to change state, see console!", {
                            type: "error",
                            position: Qt.TopRightCorner,
                            theme: "Dark",
                            closeOnClick: true,
                            autoClose: 5000,
                            hideProgressBar: false,
                        });
                }
            }
        }
    }

    function serialize() {
        return {
            "type": "StateWidget",
            "x": x,
            "y": y,
            "width": width,
            "height": height,
            "readonly": readonly
        }
    }
}
