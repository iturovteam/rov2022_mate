#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from dynamic_reconfigure.server import Server
from iturov_control.cfg import JoyConfig

def callback(config, level):
    rospy.loginfo("Reconfigure Request for Joystick")
    return config

if __name__ == "__main__":
    rospy.init_node("Joy_Params", anonymous = False)

    srv = Server(JoyConfig, callback)
    while not rospy.is_shutdown():
        continue
