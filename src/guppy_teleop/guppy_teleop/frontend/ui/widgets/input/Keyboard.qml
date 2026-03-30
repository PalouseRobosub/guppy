import QtQuick
import QtQuick.Layouts
import ui

RowLayout {
    id: keyboard

    property var name: "no inputs..."
    property var input: ({
        "q": 1,
        "w": 0,
        "e": 0,
        "a": 0,
        "s": 0,
        "d": 0,
        "o": 0,
        "p": 0,
        "up": 0,
        "down": 0,
        "left": 0,
        "right": 0
    })

    spacing: 20

    ColumnLayout {
        spacing: 8

        RowLayout {
            spacing: 8

            Repeater {
                model: ["Q","W","E"]

                delegate: Rectangle {
                    Layout.preferredWidth: 50
                    Layout.preferredHeight: 50
                    
                    radius: 6

                    color: input[modelData.toLowerCase()] ? "#4CAF50" : "#555555"
                    border.color: "#333"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent

                        text: modelData
                        color: "white"
                        font.bold: true
                    }
                }
            }
        }

        RowLayout {
            spacing: 8

            Repeater {
                model: ["A","S","D"]

                delegate: Rectangle {
                    Layout.preferredWidth: 50
                    Layout.preferredHeight: 50
                    
                    radius: 6

                    color: input[modelData.toLowerCase()] ? "#4CAF50" : "#555555"
                    border.color: "#333"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent

                        text: modelData
                        color: "white"
                        font.bold: true
                    }
                }
            }
        }
    }

    RowLayout {
        spacing: 8

        Repeater {
            model: ["O","P"]

            delegate: Rectangle {
                Layout.preferredWidth: 50
                Layout.preferredHeight: 50
                
                radius: 6

                color: input[modelData.toLowerCase()] ? "#4CAF50" : "#555555"
                border.color: "#333"
                border.width: 1

                Text {
                    anchors.centerIn: parent

                    text: modelData
                    color: "white"
                    font.bold: true
                }
            }
        }
    }

    ColumnLayout {
        spacing: 8

        RowLayout {
            spacing: 8

            Layout.fillWidth: true

            Item {
                Layout.fillWidth: true
            }

            Repeater {
                model: ["Up"]

                Rectangle {
                    Layout.preferredHeight: 50
                    Layout.preferredWidth: 50
                    
                    radius: 6

                    color: input["up"] ? "#4CAF50" : "#555555"
                    border.color: "#333"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent

                        text: "Up"
                        color: "white"
                        font.bold: true
                    }
                }
            }

            Item {
                Layout.fillWidth: true
            }
        }

        RowLayout {
            spacing: 8

            Repeater {
                model: ["Left","Down","Right"]

                delegate: Rectangle {
                    Layout.preferredWidth: 50
                    Layout.preferredHeight: 50
                    
                    radius: 6

                    color: input[modelData.toLowerCase()] ? "#4CAF50" : "#555555"
                    border.color: "#333"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent

                        text: modelData
                        color: "white"
                        font.bold: true
                    }
                }
            }
        }
    }
}