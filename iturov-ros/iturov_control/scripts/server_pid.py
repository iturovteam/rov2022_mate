#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from dynamic_reconfigure.server import Server
from iturov_control.cfg import PIDConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: 
Roll(P:{P_Roll}, I:{I_Roll}, D:{D_Roll})
Pitch(P:{P_Pitch}, I:{I_Pitch}, D:{D_Pitch})""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("PID_Params", anonymous = False)

    srv = Server(PIDConfig, callback)
    while not rospy.is_shutdown():
        continue
