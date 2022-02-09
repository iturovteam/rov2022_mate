import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 6.2
import Qt5Compat.GraphicalEffects

Item {
    width: 538
    height: 362

    Rectangle {
        id: rectangle
        color: "#5e5e5e"
        anchors.fill: parent

        Label {
            id: label
            color: "#ffffff"
            text: qsTr("Settings Page")
            anchors.verticalCenter: parent.verticalCenter
            font.pointSize: 16
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }

}
