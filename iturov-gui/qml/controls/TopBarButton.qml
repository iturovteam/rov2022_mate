import QtQuick 2.15
import QtQuick.Controls 2.15
import Qt5Compat.GraphicalEffects

Button{
    id: btnTopBar
    //  CUSTOM PROPERTİES
    property url btnIconSource: "../../../Downloads/minimize_icon.svg"
    property color btnColorDefault: "#212121"
    property color btnColorMouseOver: "#23272E"
    property color btnColorClicked: "#00a1f1"
    text: ""

    QtObject{
        id: internal

        // MOUSE OVER AND CLİCK CHANGE COLOR
        property var dynamicColor: if(btnTopBar.down){
                                       btnTopBar.down ? btnColorClicked: btnColorDefault
                                   } else {
                                       btnTopBar.hovered ? btnColorMouseOver : btnColorDefault
                                   }
    }

    width: 35
    height: 35

    background: Rectangle{
        id: bgBtn
        color: internal.dynamicColor

        Image {
            id: iconBtn
            anchors.verticalCenter: parent.verticalCenter
            source: btnIconSource
            anchors.horizontalCenter: parent.horizontalCenter
            height: 16
            width: 16
            visible: false
            fillMode: Image.PreserveAspectFit  
            antialiasing: false
        }
        ColorOverlay{
            anchors.fill: iconBtn
            source: iconBtn
            color: "#ffffff"
            antialiasing: false
        }
    }

}
