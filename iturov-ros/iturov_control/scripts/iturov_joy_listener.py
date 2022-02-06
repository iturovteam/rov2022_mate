#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy

rospy.loginfo(" -- ITUROV JOY -- ")


def callback(data):
    rospy.loginfo(f"{rospy.get_caller_id()} \n Axes: {data.axes} \n Buttons: {data.buttons}")


def listener():
    rospy.init_node('joy_listener', anonymous=False)
    rospy.Subscriber('joy', Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
