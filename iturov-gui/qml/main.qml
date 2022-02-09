import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 6.2
import Qt5Compat.GraphicalEffects
import "controls"

Window {
    id: mainWindow
    width: 640
    height: 480
    visible: true
    color: "#00ffffff"
    title: qsTr("course QT Quick")
    // Remove title bar
    flags: Qt.Window | Qt.FramelessWindowHint

    // Properties
    property int windowStatus: 0
    property int windowMargin: 10
    // Internal functions
    QtObject{
        id: internal
        function resetResizeBorders(){
            resizeLeft.visible = true
            resizeRight.visible = true
            resizeBottom.visible = true
            resizeWindow.visible = true
        }

        function maximizeRestore(){
            if(windowStatus == 0){
                mainWindow.showMaximized()
                windowStatus = 1
                windowMargin = 0
                // Resize visibility
                resizeLeft.visible = false
                resizeRight.visible = false
                resizeBottom.visible = false
                resizeWindow.visible = false
                btnMaximizeRestore.btnIconSource = "../../../Downloads/restore_icon.svg"
            }
            else{
                mainWindow.showNormal()
                windowStatus = 0
                windowMargin = 10
                // Resize visibility
                internal.resetResizeBorders()
                btnMaximizeRestore.btnIconSource = "../../../Downloads/maximize_icon.svg"
            }
        }
        function ifMaximizedWindowsRestore(){
            if(windowStatus == 1){
                mainWindow.showNormal()
                windowStatus = 0
                windowMargin = 10
                // Resize visibility
                internal.resetResizeBorders()
                btnMaximizeRestore.btnIconSource = "../../../Downloads/maximize_icon.svg"
            }
        }
        function restoreMargins(){
            windowStatus = 0
            windowMargin = 10
            // Resize visibility
            internal.resetResizeBorders()
            btnMaximizeRestore.btnIconSource = "../../../Downloads/maximize_icon.svg"
        }
    }

    Rectangle {
        id: bg
        x: 220
        y: 127
        color: "#4f4f4f"
        border.color: "#4f4f4f"
        border.width: 1
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.rightMargin: windowMargin
        anchors.leftMargin: windowMargin
        anchors.bottomMargin: windowMargin
        anchors.topMargin: windowMargin
        z: 1

        Rectangle {
            id: appContainer
            color: "#424242"
            border.width: 1
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.rightMargin: 1
            anchors.leftMargin: 1
            anchors.bottomMargin: 1
            anchors.topMargin: 1

            Rectangle {
                id: topBar
                height: 71
                color: "#212121"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                anchors.topMargin: 0

                ToggleBTN{
                    x: 0
                    y: 0
                    width: 79
                    height: 71
                    onClicked: animationMenu.running = true

                }

                Row {
                    id: row
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.rightMargin: 0
                    anchors.leftMargin: 513
                    anchors.bottomMargin: 36
                    anchors.topMargin: 0

                    TopBarButton {
                        id: btnMaximizeRestore
                        text: ""
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        btnIconSource: "../../../Downloads/maximize_icon.svg"
                        anchors.bottomMargin: 4
                        bottomPadding: 5
                        anchors.leftMargin: 35
                        anchors.rightMargin: 35
                        anchors.topMargin: 0
                        onClicked: internal.maximizeRestore()
                    }

                    TopBarButton {
                        id: btnMinimize
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.bottom
                        anchors.bottom: parent.bottom
                        anchors.bottomMargin: 4
                        bottomPadding: 5
                        anchors.leftMargin: 0
                        anchors.rightMargin: 70
                        anchors.topMargin: -35
                        onClicked: {
                            mainWindow.showMinimized()
                            internal.restoreMargins()
                        }
                    }
                }
            }

            Rectangle {
                id: content
                color: "#424242"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: topBar.bottom
                anchors.bottom: parent.bottom
                anchors.topMargin: 0

                Rectangle {
                    id: leftMenu
                    width: 88
                    color: "#212121"
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    anchors.leftMargin: 0
                    anchors.bottomMargin: 0
                    anchors.topMargin: 0

                    PropertyAnimation{
                        id: animationMenu
                        target: leftMenu
                        property: "width"
                        to: if(leftMenu.width == 88) return 200; else return 88
                        duration: 750
                        easing.type: Easing.OutInCirc
                    }

                    LeftMenuBtn {
                        id: btnHome
                        x: 0
                        y: 0
                        width: leftMenu.width
                        height: 60
                        text: qsTr("  Home")
                        isActiveMenu: true
                        onClicked: {
                            btnHome.isActiveMenu = true
                            btnSettings.isActiveMenu = false
                            //stackView.push(Qt.resolvedUrl("pages/homePage.qml"))
                            pagesView.setSource(Qt.resolvedUrl("pages/homePage.qml"))
                        }
                    }

                    LeftMenuBtn {
                        id: btnOpen
                        x: 0
                        y: 60
                        width: leftMenu.width
                        height: 60
                        text: qsTr("  Open")
                        isActiveMenu: false
                        btnIconSource: "../../../Downloads/open_icon.svg"
                    }

                    LeftMenuBtn {
                        id: btnSave
                        x: 0
                        y: 120
                        width: leftMenu.width
                        height: 60
                        text: qsTr("  Save")
                        isActiveMenu: false
                        btnIconSource: "../../../Downloads/save_icon.svg"
                    }

                    LeftMenuBtn {
                        id: btnSettings
                        x: 0
                        y: 327
                        width: leftMenu.width
                        height: 60
                        text: qsTr("  Settings")
                        anchors.bottom: parent.bottom
                        anchors.bottomMargin: 25
                        btnIconSource: "../../../Downloads/settings_icon.svg"
                        isActiveMenu: false
                        onClicked: {
                            btnHome.isActiveMenu = false
                            btnSettings.isActiveMenu = true
                            //stackView.push(Qt.resolvedUrl("pages/settingsPage.qml"))
                            pagesView.setSource(Qt.resolvedUrl("pages/settingsPage.qml"))
                        }
                    }
                }




                Rectangle {
                    id: topBarDescription
                    color: "#424242"
                    border.width: 0
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.top
                    anchors.bottomMargin: 0
                    anchors.topMargin: -40
                    anchors.rightMargin: 0
                    anchors.leftMargin: 80

                    Label {
                        id: labelTopInfo
                        color: "#a9a9a9"
                        text: qsTr("Application Discription")
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                        anchors.rightMargin: 200
                        anchors.leftMargin: 10
                        anchors.bottomMargin: 0
                        anchors.topMargin: 0

                        Column {
                            id: columnMenus
                            anchors.left: parent.left
                            anchors.right: parent.left
                            anchors.top: parent.bottom
                            anchors.bottom: parent.bottom
                            anchors.leftMargin: -90
                            anchors.rightMargin: 10
                            anchors.topMargin: 0
                            anchors.bottomMargin: -260
                        }
                    }

                    Label {
                        id: labelRightInfo
                        color: "#a9a9a9"
                        text: qsTr("I HOME  ")
                        anchors.left: labelTopInfo.right
                        anchors.right: parent.right
                        anchors.top: parent.bottom
                        anchors.bottom: parent.bottom
                        horizontalAlignment: Text.AlignRight
                        verticalAlignment: Text.AlignVCenter
                        anchors.leftMargin: 0
                        anchors.bottomMargin: 0
                        anchors.rightMargin: 0
                        anchors.topMargin: -40
                    }
                }

                Rectangle {
                    id: titleBar
                    color: "#212121"
                    border.color: "#212121"
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.top
                    anchors.rightMargin: 105
                    anchors.leftMargin: 80
                    anchors.bottomMargin: 41
                    anchors.topMargin: -71

                    DragHandler {
                        onActiveChanged: if(active){
                                            mainWindow.startSystemMove()
                                            internal.ifMaximizedWindowsRestore()
                                         }
                    }


                    Label {
                        id: label
                        color: "#ffffff"
                        text: qsTr("My Application Title")
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        verticalAlignment: Text.AlignVCenter
                        styleColor: "#ffffff"
                        anchors.leftMargin: 35
                    }
                }

                Image {
                    id: iconApp
                    width: 22
                    height: 22
                    anchors.left: titleBar.left
                    anchors.top: titleBar.top
                    anchors.bottom: titleBar.bottom
                    source: "../../Downloads/icon_app_top.svg"
                    anchors.leftMargin: 0
                    anchors.bottomMargin: 0
                    anchors.topMargin: 0
                    fillMode: Image.PreserveAspectFit
                }

                Rectangle {
                    id: contantPages
                    color: "#5e5e5e"
                    anchors.left: leftMenu.right
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    clip: false
                    anchors.rightMargin: 0
                    anchors.leftMargin: -8
                    anchors.bottomMargin: 25
                    anchors.topMargin: 0

                    Rectangle {
                        id: rectangle
                        color: "#ffffff"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.bottom
                        anchors.bottom: parent.bottom
                        anchors.topMargin: 0
                        anchors.rightMargin: 0
                        anchors.leftMargin: 0
                        anchors.bottomMargin: 0

                        Label {
                            id: labelTopInfo1
                            color: "#c0c0c0"
                            text: qsTr("Application Discription")
                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignTop
                            anchors.leftMargin: 20
                            anchors.bottomMargin: 0
                            anchors.rightMargin: 30
                            anchors.topMargin: 0
                        }
                    }

                    Rectangle {
                        id: necessary
                        color: "#424242"
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: rectangle.bottom
                        anchors.bottom: rectangle.top
                        anchors.rightMargin: 524
                        anchors.leftMargin: 0
                        anchors.bottomMargin: -25
                        anchors.topMargin: 0
                    }

                    MouseArea {
                        id: mouseArea
                        anchors.left: parent.right
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: parent.bottom
                        anchors.rightMargin: 0
                        anchors.leftMargin: -25
                        anchors.bottomMargin: -25
                        anchors.topMargin: 362
                        cursorShape: Qt.SizeFDiagCursor

                        DragHandler{
                            target: null
                            onActiveChanged: if (active){
                                                 mainWindow.startSystemResize(Qt.RightEdge | Qt.BottomEdge)
                                             }
                        }

                        Image {
                            id: image
                            anchors.fill: parent
                            source: "../../Downloads/resize_icon.svg"
                            fillMode: Image.PreserveAspectFit
                            antialiasing: false
                        }
                    }

//                   StackView {
//                        id: stackView
//                        anchors.fill: parent
//                        anchors.rightMargin: 0
//                        layer.enabled: false
//                        antialiasing: false
//                        initialItem: Qt.resolvedUrl("pages/homePage.qml")
//                    }
                    Loader{
                        id: pagesView
                        anchors.fill: parent
                        source: Qt.resolvedUrl("pages/homePage.qml")
                    }

                }
            }
        }

        TopBarButton {
            id: btnClose
            anchors.left: parent.right
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.top
            btnColorClicked: "#f10000"
            btnIconSource: "../../../Downloads/close_icon.svg"
            anchors.rightMargin: 0
            anchors.leftMargin: -35
            anchors.bottomMargin: -32
            anchors.topMargin: 1
            bottomPadding: 5
            onClicked: mainWindow.close()
        }
    }

    DropShadow{
        anchors.fill: bg
        horizontalOffset: 0
        verticalOffset: 0
        radius: 10
        color: "#80000000"
        source: bg
        layer.samples: 16
        z: 0
    }
    MouseArea {
        id: resizeLeft
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.rightMargin: 630
        anchors.leftMargin: 0
        anchors.bottomMargin: 10
        anchors.topMargin: 10
        cursorShape: Qt.SizeHorCursor

        DragHandler{
            target: null
            onActiveChanged: if (active) { mainWindow.startSystemResize(Qt.LeftEdge) }
        }
    }

    MouseArea {
        id: resizeRight
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.rightMargin: 0
        anchors.leftMargin: 630
        anchors.bottomMargin: 10
        anchors.topMargin: 10
        cursorShape: Qt.SizeHorCursor

        DragHandler{
            target: null
            onActiveChanged: if (active) { mainWindow.startSystemResize(Qt.RightEdge) }
        }
    }

    MouseArea {
        id: resizeBottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        layer.enabled: true
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.bottomMargin: 0
        anchors.topMargin: 470
        cursorShape: Qt.SizeVerCursor

        DragHandler{
            target: null
            onActiveChanged: if (active) { mainWindow.startSystemResize(Qt.BottomEdge) }
        }
    }
}