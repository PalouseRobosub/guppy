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

                    text: {
                        const data = root.params?.robot_volume

                        return (data) ? String(data) : "0.0"
                    }
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

                    text: {
                        const data = root.params?.robot_mass

                        return (data) ? String(data) : "0.0"
                    }
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

                getValue: (i) => {
                    const data = root.params?.center_of_buoyancy

                    return (data && i < data.length) ? String(data[i]) : "0.0"
                }
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

                    text: {
                        const data = root.params?.water_density

                        return (data) ? String(data) : "0.0"
                    }
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

                    text: {
                        const data = root.params?.qp_epsilon

                        return (data) ? String(data) : "0.0"
                    }
                }
            }
        }
    }

    property var params
}