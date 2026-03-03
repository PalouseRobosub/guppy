import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

RowLayout {
    id: root

    spacing: 6

    property int count: 0
    property var getValue: (i) => ""

    Repeater {
        model: root.count

        delegate: TextField {
            Layout.preferredWidth: 70
            Layout.preferredHeight: 32

            horizontalAlignment: Text.AlignHCenter

            validator: DoubleValidator {}

            text: root.getValue(index)
        }
    }
}