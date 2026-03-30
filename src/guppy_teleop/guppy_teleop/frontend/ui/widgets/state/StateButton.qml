import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Button {
    id: button

    property string stateName

    Layout.preferredWidth: 100
    Layout.preferredHeight: 40

    background: Rectangle {
        radius: 6

        border.color: "#333"
        border.width: 1

        color: {
            const active = root.currentState === button.stateName

            if (active) {
                if (button.pressed) return '#22aaee'
                if (button.hovered) return '#44ccff'
                return "#33bbff"
            } else {
                if (button.pressed) return '#454545'
                if (button.hovered) return '#555555'
                return "#505050"
            }
        }
    }

    contentItem: Text {
        text: button.stateName
        anchors.fill: parent

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        color: "white"
        font.bold: true
    }

    onClicked: {
        var success = stateWidget.pushState(button.stateName)

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