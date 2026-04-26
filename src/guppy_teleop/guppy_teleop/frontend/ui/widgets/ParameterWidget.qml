    import QtQuick
    import QtQuick.Controls
    import QtQuick.Layouts
    import ui

    CanvasWidget {
        id: root
        title: "Parameters"

        property var params: JSON.parse(JSON.stringify(parameterWidget.parameters)) // TODO find better method of deep copy
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

                MotorSection { params: root.params }
                PIDSection { params: root.params }
                DragSection { params: root.params }
                PhysicalSection { params: root.params }
            }
        }

        Rectangle {
            id: actions

            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.margins: 10

            width: row.implicitWidth + 16
            height: row.implicitHeight + 16

            radius: 18
            color: '#AA222222'

            z: 1000

            Row {
                id: row
                
                anchors.fill: parent
                anchors.margins: 8

                spacing: 8

                CircleButton {
                    iconSource: "qrc:/assets/icons/undo.svg"

                    tooltip: "Reset fields to live parameters."

                    onClicked: {
                        Window.window.contentItem.forceActiveFocus()

                        params = JSON.parse(JSON.stringify(parameterWidget.parameters))

                        toastManager.createMessage("Reset fields to live parameters!", {
                            type: "info",
                            position: Qt.TopRightCorner,
                            theme: "Dark",
                            closeOnClick: true,
                            autoClose: 5000,
                            hideProgressBar: false,
                        });
                    }
                }

                CircleButton {
                    iconSource: "qrc:/assets/icons/push.svg"

                    tooltip: "Push parameter changes to guppy."

                    onClicked: {
                        parameterWidget.pushParameters(params)

                        toastManager.createMessage("Pushed parameter changes to guppy!", {
                            type: "info",
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
                "type": "ParameterWidget",
                "x": x,
                "y": y,
                "width": width,
                "height": height,
                "readonly": readonly
            }
        }
    }
