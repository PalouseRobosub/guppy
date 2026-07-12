import QtQuick
import QtQuick.Layouts
import ui

Column {
    id: controller

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

    spacing: 8

    Row {
        spacing: 8

        Repeater {
            model: ["Q","W","E"]

            delegate: Rectangle {
                width: 50;
                height: 50;
                
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

    Row {
        spacing: 8

        Repeater {
            model: ["A","S","D"]

            delegate: Rectangle {
                width: 50;
                height: 50;
                
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