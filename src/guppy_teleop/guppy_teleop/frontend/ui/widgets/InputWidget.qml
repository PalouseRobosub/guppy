import QtQuick
import ui

CanvasWidget {
    id: root

    title: "Input Widget"

    property var name: "no inputs..."
    property string format: "keyboard"
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

    Item {
        anchors.fill: parent
        anchors.margins: 8
        
        Keyboard {
            name: root.name
            input: root.input
        }
    }

    function serialize() {
        return {
            "type": "TextWidget",
            "x": x,
            "y": y,
            "width": width,
            "height": height
        }
    }

    Connections {
        target: inputWidget

        function onInputUpdated() {
            root.name = inputWidget.inputPackage["name"]
            root.format = inputWidget.inputPackage["format"]
            root.input = inputWidget.inputPackage["input"]
        }
    }
}
