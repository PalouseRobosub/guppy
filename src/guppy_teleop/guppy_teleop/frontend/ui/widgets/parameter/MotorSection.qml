import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

CollapsibleSection {
    id: root

    title: "Motor Parameters"

    contentMargins: 12

    RowLayout {
        width: parent.width
        spacing: 26

        ColumnLayout {
            spacing: 10

            Label {
                text: "Motor Positions"
                font.pixelSize: 13
                color: "#cccccc"
            }

            GridLayout {
                id: positionGrid

                rows: 8
                columns: 5

                rowSpacing: 6
                columnSpacing: 6

                Repeater {
                    model: positionGrid.rows * positionGrid.columns

                    delegate: TextField {
                        Layout.preferredWidth: 70
                        Layout.preferredHeight: 32

                        horizontalAlignment: Text.AlignHCenter
                        validator: DoubleValidator {}

                        property int i: index

                        text: root.params?.motor_positions?.[i] ?? "0.0"

                        onEditingFinished: {
                            if (!root.params.motor_positions)
                                root.params.motor_positions = []

                            root.params.motor_positions[i] = parseFloat(text)
                        }
                    }
                }
            }
        }

        ColumnLayout {
            spacing: 10

            Label {
                text: "Motor Upper Bounds"
                font.pixelSize: 13
                color: "#cccccc"
            }

            FieldRowRepeater {
                count: 8

                values: root.params.motor_upper_bounds
            }

            Label {
                text: "Motor Lower Bounds"
                font.pixelSize: 13
                color: "#cccccc"
            }

            FieldRowRepeater {
                count: 8

                values: root.params.motor_lower_bounds
            }
        }
    }

    property var params
}