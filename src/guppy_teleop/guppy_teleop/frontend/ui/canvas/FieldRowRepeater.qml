import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

RowLayout {
    id: root

    spacing: 6

    property int count: 0
    property var values: []

    Repeater {
        model: root.count

        delegate: TextField {
            Layout.preferredWidth: 70
            Layout.preferredHeight: 32

            horizontalAlignment: Text.AlignHCenter

            validator: DoubleValidator {}

            placeholderTextColor: Theme.textPlaceholder
            color: Theme.textSecondary // TODO add readonly

            /*background: Rectangle {
                implicitWidth: 200
                implicitHeight: 40
                color: control.enabled ? "transparent" : "#353637"
                border.color: control.enabled ? "#21be2b" : "transparent"
            }*/ //TODO CUSTOMIZE BACKGROUNDS

            property int i: index

            text: (root.values && i < root.values.length)
                ? String(root.values[i])
                : "0.0"

            onEditingFinished: {
                if (!root.values)
                    root.values = []
                
                root.values[i] = parseFloat(text)
            }
        }
    }
}