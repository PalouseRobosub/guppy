import QtQuick
import QtQuick.Controls
import QtQuick.VectorImage
import QtQuick.Dialogs
import ui

Item {
    id: root

    clip: true

    property real zoom: 1.0
    property real minZoom: 0.2
    property real maxZoom: 4.0

    property int gridStep: 40
    property bool snapToGrid: false

    property real offsetX: 0
    property real offsetY: 0

    property var widgetRegistry: ({
        "TextWidget": "qrc:/widgets/ui/widgets/TextWidget.qml",
        "StateWidget": "qrc:/widgets/ui/widgets/StateWidget.qml",
        "ParameterWidget": "qrc:/widgets/ui/widgets/ParameterWidget.qml"
    })

    function centerOn(x, y) {
        root.offsetX = root.width / 2 - x * root.zoom
        root.offsetY = root.height / 2 - y *root. zoom
    }

    function loadWorkspace(data) {
        offsetX = 0
        offsetY = 0
        zoom = 1.0

        clearWidgets()

        var widgets = data["widgets"] || []

        widgets.forEach(function(json) {
            var path = widgetRegistry[json["type"]]

            var component = Qt.createComponent(path)
            if (component.status === Component.Error)
                console.error("rrror loading component... ", component.errorString())

            var properties = {}
            for (var key in json) if (key !== "type")
                properties[key] = json[key]
            
            properties.snapToGrid = Qt.binding(function(){ return root.snapToGrid })
            properties.gridStep = Qt.binding(function(){ return root.gridStep })

            var object = component.createObject(workspace, properties)

            object.closeRequested.connect(function() { object.destroy() })
        })
    }

    function clearWidgets() {
        workspace.children.forEach(function(child) {
            if (typeof child.closeRequested !== "undefined")
                child.destroy()
        })
    }

    FileDialog {
        id: saveDialog

        title: "Save Workspace"

        nameFilters: ["JSON files (*.json)"]
        fileMode: FileDialog.SaveFile

        onAccepted: {
            workspaceManager.saveWorkspace(selectedFile, workspace.children)
            // TODO transform children?
        }
    }

    FileDialog {
        id: loadDialog

        title: "Load Workspace"

        nameFilters: ["JSON files (*.json)"]
        fileMode: FileDialog.OpenFile

        onAccepted: workspaceManager.loadWorkspaceFromUrl(selectedFile)
    }

    Canvas {
        id: grid

        anchors.fill: parent

        z: -1

        opacity: 0.4

        onPaint: {
            const context = getContext("2d")
            context.clearRect(0, 0, width, height)

            let step = Math.max(4, root.gridStep * root.zoom) // step offset in pixels

            const ox = ((root.offsetX % step) + step) % step
            const oy = ((root.offsetY % step) + step) % step

            context.strokeStyle = "#2e2e2e"
            context.lineWidth = 1
            context.beginPath()

            for (var x = ox; x < width;  x += step) {
                context.moveTo(x, 0)
                context.lineTo(x, height)
            }

            for (var y = oy; y < height; y += step) { 
                context.moveTo(0, y)
                context.lineTo(width, y)
            }

            context.stroke()
        }

        Connections {
            target: root
            
            function onZoomChanged() {
                grid.requestPaint()
            }
        }

        Connections {
            target: root
            
            function onOffsetXChanged() {
                grid.requestPaint()
            }
        }

        Connections {
            target: root
            
            function onOffsetYChanged() {
                grid.requestPaint()
            }
        }
    }

    Item {
        id: contentLayer

        width: root.width
        height: root.height

        x: root.offsetX
        y: root.offsetY

        transformOrigin: Item.TopLeft

        scale: root.zoom
        
        Item {
            id: workspace

            width: 100000
            height: 100000

            clip: false
        }
    }

    TapHandler {
        onTapped: workspace.children.forEach(function(child) {
            if (child) child.selected = false
        })
    }

    DragHandler {
        target: null

        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchScreen | PointerDevice.TouchPad
        acceptedButtons: Qt.LeftButton
        grabPermissions: PointerHandler.TakeOverForbidden

        cursorShape: active ? Qt.ClosedHandCursor : Qt.ArrowCursor

        property real pressOffsetX: 0.0
        property real pressOffsetY: 0.0

        onActiveChanged: {
            if (active) {
                pressOffsetX = root.offsetX
                pressOffsetY = root.offsetY
            }
        }

        onTranslationChanged: {
            root.offsetX = pressOffsetX + activeTranslation.x
            root.offsetY = pressOffsetY + activeTranslation.y
        }
    }

    PinchHandler {
        target: null

        onScaleChanged: (delta) => {
            let newZoom = Math.max(root.minZoom, Math.min(root.maxZoom, root.zoom * delta))

            let ratio = newZoom / root.zoom

            let cx = centroid.position.x
            let cy = centroid.position.y

            root.offsetX += (1 - ratio) * (cx - root.offsetX)
            root.offsetY += (1 - ratio) * (cy - root.offsetY)
            root.zoom = newZoom
        }
    }

    Rectangle {
        id: hud

        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 10

        width: row.implicitWidth + 16
        height: row.implicitHeight + 16

        radius: 18
        color: '#AA222222'

        z: 1000

        Row {
            id: row
            
            anchors.fill: parent
            anchors.margins: 8

            spacing: 8

            CircleButton {
                iconSource: root.snapToGrid ? "qrc:/assets/icons/snap_enabled.svg" : "qrc:/assets/icons/snap_disabled.svg"

                tooltip: root.snapToGrid ? "Disable grid snapping." : "Enable grid snapping."

                onClicked: root.snapToGrid = !root.snapToGrid
            }

            CircleButton {
                iconSource: "qrc:/assets/icons/home.svg"

                tooltip: "Return to the center of the workspace."

                onClicked: {
                    root.offsetX = 0
                    root.offsetY = 0

                    root.zoom = 1.0
                }
            }

            CircleButton {
                iconSource: "qrc:/assets/icons/save.svg"

                tooltip: "Save workspace to file."

                onClicked: saveDialog.open()
            }

            CircleButton {
                iconSource: "qrc:/assets/icons/upload.svg"

                tooltip: "Load workspace from file."

                onClicked: loadDialog.open()
            }
        }
    }
}