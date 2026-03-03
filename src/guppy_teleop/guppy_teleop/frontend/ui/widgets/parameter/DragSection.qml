import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

CollapsibleSection {
    id: root

    title: "Drag Parameters"

    contentMargins: 12

    ColumnLayout {
        width: parent.width
        spacing: 10

        RowLayout {
            spacing: 26

            ColumnLayout {
                spacing: 10

                Label {
                    text: "Axis Weights"
                    font.pixelSize: 13
                    color: "#cccccc"
                }

                GridLayout {
                    id: weightGrid

                    rows: 6
                    columns: 6

                    rowSpacing: 6
                    columnSpacing: 6

                    Repeater {
                        model: weightGrid.rows * weightGrid.columns

                        delegate: TextField {
                            Layout.preferredWidth: 70
                            Layout.preferredHeight: 32

                            horizontalAlignment: Text.AlignHCenter
                            validator: DoubleValidator {}

                            text: {
                                const data = root.params?.axis_weight_matrix

                                return (data && index < data.length) ? String(data[index]) : "0.0"
                            }
                        }
                    }
                }
            }

            ColumnLayout {
                spacing: 10

                Label {
                    text: "Drag Effect"
                    font.pixelSize: 13
                    color: "#cccccc"
                }

                GridLayout {
                    id: dragGrid

                    rows: 6
                    columns: 6

                    rowSpacing: 6
                    columnSpacing: 6

                    Repeater {
                        model: dragGrid.rows * dragGrid.columns

                        delegate: TextField {
                            Layout.preferredWidth: 70
                            Layout.preferredHeight: 32

                            horizontalAlignment: Text.AlignHCenter
                            validator: DoubleValidator {}

                            text: {
                                const data = root.params?.drag_effect_matrix

                                return (data && index < data.length) ? String(data[index]) : "0.0"
                            }
                        }
                    }
                }
            }
        }

        RowLayout {
            spacing: 26

            ColumnLayout {
                Label {
                    text: "Drag Coefficients"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 6

                    getValue: (i) => {
                        const data = root.params?.drag_coefficients

                        return (data && i < data.length) ? String(data[i]) : "0.0"
                    }
                }
            }

            ColumnLayout {
                Label {
                    text: "Drag Areas"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 6

                    getValue: (i) => {
                        const data = root.params?.drag_areas

                        return (data && i < data.length) ? String(data[i]) : "0.0"
                    }
                }
            }
        }
    }

    property var params
}