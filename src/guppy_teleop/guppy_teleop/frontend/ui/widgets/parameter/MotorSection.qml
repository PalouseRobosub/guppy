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

                        text: {
                            const data = root.params?.motor_positions

                            return (data && index < data.length) ? String(data[index]) : "0.0"
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

                getValue: (i) => {
                    const data = root.params?.motor_upper_bounds

                    return (data && i < data.length) ? String(data[i]) : "0.0"
                }
            }

            Label {
                text: "Motor Lower Bounds"
                font.pixelSize: 13
                color: "#cccccc"
            }

            FieldRowRepeater {
                count: 8

                getValue: (i) => {
                    const data = root.params?.motor_lower_bounds

                    return (data && i < data.length) ? String(data[i]) : "0.0"
                }
            }
        }
    }

    property var params
}