import QtQuick 2.15
import QtQuick.Controls 2.15
import Qt5Compat.GraphicalEffects

Button{
    id: btnToggle
    //  CUSTOM PROPERTİES
    property url btnIconSource: "../../../Downloads/menu_icon.svg"
    property color btnColorDefault: "#212121"
    property color btnColorMouseOver: "#23272E"
    property color btnColorClicked: "#00a1f1"
    text: ""

    QtObject{
        id: internal

        // MOUSE OVER AND CLİCK CHANGE COLOR
        property var dynamicColor: if(btnToggle.down){
                                       btnToggle.down ? btnColorClicked: btnColorDefault
                                   } else {
                                       btnToggle.hovered ? btnColorMouseOver : btnColorDefault
                                   }
    }

    implicitWidth: 70
    implicitHeight: 60

    background: Rectangle{
        id: bgBtn
        color: internal.dynamicColor

        Image {
            id: iconBtn
            anchors.verticalCenter: parent.verticalCenter
            source: btnIconSource
            anchors.horizontalCenter: parent.horizontalCenter
            height: 25
            width: 25
            fillMode: Image.PreserveAspectFit
            visible: false
        }
        ColorOverlay{
            anchors.fill: iconBtn
            source: iconBtn
            color: "#ffffff"
            antialiasing: false
        }
    }

}

