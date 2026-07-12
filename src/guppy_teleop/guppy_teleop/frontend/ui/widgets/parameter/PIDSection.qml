import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ui

CollapsibleSection {
    id: root

    title: "PID Parameters"

    contentMargins: 12

    ColumnLayout {
        width:parent.width

        spacing: 10

        RowLayout {
            spacing: 26

            ColumnLayout {
                Label {
                    text: "PID Gains Velocity Linear"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 3

                    values: root.params.pid_gains_vel_linear
                }
            }

            ColumnLayout {
                Label {
                    text: "PID Gains Velocity Angular"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 3

                    values: root.params.pid_gains_vel_angular
                }
            }
        }

        RowLayout {
            spacing: 26

            ColumnLayout {
                Label {
                    text: "PID Gains Pose Linear"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 3

                    values: root.params.pid_gains_pose_linear
                }
            }

            ColumnLayout {
                Label {
                    text: "PID Gains Pose Angular"
                    color: "#cccccc"
                    font.pixelSize: 13
                }

                FieldRowRepeater {
                    count: 3

                    values: root.params.pid_gains_pose_angular
                }
            }
        }

        Label {
            text: "Pose Lock Deadband"
            color: "#cccccc"
            font.pixelSize: 13
        }

        FieldRowRepeater {
            count: 6

            values: root.params.pose_lock_deadband
        }
    }

    property var params
}