pragma Singleton

import QtQuick

QtObject {
    readonly property color canvasBackground: "#181818"
    readonly property color canvasGrid: "#2e2e2e"

    readonly property color sidebarBackground: "#1e1e1e"
    readonly property color sidebarDivider: "#2e2e2e"
    readonly property color sidebarButton: "transparent"
    readonly property color sidebarButtonHover: "#2a2a2a"

    readonly property color textPrimary:     "#e8e8e8"
    readonly property color textSecondary:   "#aaaaaa"
    readonly property color textDisabled:    "#555555"
    readonly property color textPlaceholder: "#666666"

    readonly property color circleButton: "#2A2A2A"
    readonly property color circleButtonHover: "#333333"

    readonly property color widgetBackground: "#2b2b2b"
    readonly property color widgetBorder: "#404040"
    readonly property color widgetTitleBar: "#313131"
    readonly property color resizeGrip: "#666"

    readonly property color semanticError: "#e74c3c"
    readonly property color semanticWarning: "#f1c40f"
    readonly property color semanticSuccess: "#07bc0c"
    readonly property color semanticInfo: "#3498db"
}