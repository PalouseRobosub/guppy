import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

CollapsibleSection {
    id: root

    title: "Physical Constants"

    contentMargins: 12

    ColumnLayout {
        width: parent.width

        spacing: 10

        RowLayout {
            spacing: 26

            ColumnLayout {
                Label {
                    text: "Robot Volume"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                TextField {
                    Layout.preferredWidth: 70
                    Layout.preferredHeight: 32

                    horizontalAlignment: Text.AlignHCenter

                    validator: DoubleValidator {}

                    text: root.params.robot_volume ?? "0.0"

                    onEditingFinished: root.params.robot_volume = parseFloat(text)
                }
            }

            ColumnLayout {
                Label {
                    text: "Robot Mass"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                TextField {
                    Layout.preferredWidth: 70
                    Layout.preferredHeight: 32

                    horizontalAlignment: Text.AlignHCenter

                    validator: DoubleValidator {}

                    text: root.params.robot_mass ?? "0.0"

                    onEditingFinished: root.params.robot_mass = parseFloat(text)
                }
            }
        }

        ColumnLayout {
            Label {
                text: "Center of Buoyancy"
                color: "#cccccc"
                font.pixelSize: 13
            }


            FieldRowRepeater {
                count: 3

                values: root.params.center_of_buoyancy
            }
        }

        RowLayout {
            spacing: 26

            ColumnLayout {
                Label {
                    text: "Water Desnity"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                TextField {
                    Layout.preferredWidth: 70
                    Layout.preferredHeight: 32

                    horizontalAlignment: Text.AlignHCenter

                    validator: DoubleValidator {}

                    text: root.params.water_density ?? "0.0"

                    onEditingFinished: root.params.water_density = parseFloat(text)
                }
            }

            ColumnLayout {
                Label {
                    text: "QP Epsilon"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                TextField {
                    Layout.preferredWidth: 70
                    Layout.preferredHeight: 32

                    horizontalAlignment: Text.AlignHCenter

                    validator: DoubleValidator {}

                    text: root.params.qp_epsilon ?? "0.0"

                    onEditingFinished: root.params.qp_epsilon = parseFloat(text)
                }
            }
        }
    }

    property var params
}