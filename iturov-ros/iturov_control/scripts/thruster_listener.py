#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import yaml
import rospy
import std_msgs.msg
import uuv_gazebo_ros_plugins_msgs.msg


class TopicClass:
    def __init__(self, name: str, class_type):
        self.__name = name
        self.__classType = class_type
        self.__data = None

    def set_data(self, data):
        self.__data = data

    def get_data(self):
        return self.__data

    def get_type(self):
        return self.__classType

    def get_name(self):
        return self.__name


class RosHandler:
    def __init__(self):
        self.rate = 30
        self.connected = False

    def connect(self, node: str, rate: int):
        rospy.init_node(node, anonymous=False)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("Rospy is up ...")
        # rospy.spin()

    def disconnect(self):
        if self.connected:
            rospy.logwarn("shutting down rospy ...")
            rospy.signal_shutdown("disconnect")
            self.connected = False

    @staticmethod
    def topic_publisher(topic: TopicClass):
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())

    @staticmethod
    def topic_subscriber(topic: TopicClass):
        rospy.Subscriber(topic.get_name(), topic.get_type(), topic.set_data)

    @staticmethod
    def service_caller(service: TopicClass, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo("waiting for ROS service:" + srv)
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo("ROS service is up:" + srv)
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None


class ThrusterListener(RosHandler):
    def __init__(self):
        super().__init__()
        self.uuv_name = "/iturov"

        self.TOPIC_THRUSTER_0 = TopicClass(self.uuv_name + "/thrusters/0/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_1 = TopicClass(self.uuv_name + "/thrusters/1/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_2 = TopicClass(self.uuv_name + "/thrusters/2/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_3 = TopicClass(self.uuv_name + "/thrusters/3/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_4 = TopicClass(self.uuv_name + "/thrusters/4/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_5 = TopicClass(self.uuv_name + "/thrusters/5/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_6 = TopicClass(self.uuv_name + "/thrusters/6/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTER_7 = TopicClass(self.uuv_name + "/thrusters/7/input",
                                           uuv_gazebo_ros_plugins_msgs.msg.FloatStamped)
        self.TOPIC_THRUSTERS_INPUT = TopicClass(self.uuv_name + "/thrusters/input",
                                                std_msgs.msg.Int16MultiArray)

    def start_topic_subscribe(self):
        self.topic_subscriber(self.TOPIC_THRUSTER_0)
        self.topic_subscriber(self.TOPIC_THRUSTER_1)
        self.topic_subscriber(self.TOPIC_THRUSTER_2)
        self.topic_subscriber(self.TOPIC_THRUSTER_3)
        self.topic_subscriber(self.TOPIC_THRUSTER_4)
        self.topic_subscriber(self.TOPIC_THRUSTER_5)
        self.topic_subscriber(self.TOPIC_THRUSTER_6)
        self.topic_subscriber(self.TOPIC_THRUSTER_7)

    def main(self):
        self.connect(node="thruster_listener", rate=50)
        self.start_topic_subscribe()

        while not rospy.is_shutdown():
            if self.connected:
                thruster_0_data = self.TOPIC_THRUSTER_0.get_data()
                thruster_1_data = self.TOPIC_THRUSTER_1.get_data()
                thruster_2_data = self.TOPIC_THRUSTER_2.get_data()
                thruster_3_data = self.TOPIC_THRUSTER_3.get_data()
                thruster_4_data = self.TOPIC_THRUSTER_4.get_data()
                thruster_5_data = self.TOPIC_THRUSTER_5.get_data()
                thruster_6_data = self.TOPIC_THRUSTER_6.get_data()
                thruster_7_data = self.TOPIC_THRUSTER_7.get_data()

                data = std_msgs.msg.Int16MultiArray()
                if thruster_0_data and thruster_1_data and thruster_2_data and thruster_3_data and \
                   thruster_4_data and thruster_5_data and thruster_6_data and thruster_7_data is not None:
                    data.data = [int(thruster_0_data.data),
                                 int(thruster_1_data.data),
                                 int(thruster_2_data.data),
                                 int(thruster_3_data.data),
                                 int(thruster_4_data.data),
                                 int(thruster_5_data.data),
                                 int(thruster_6_data.data),
                                 int(thruster_7_data.data)]

                    self.TOPIC_THRUSTERS_INPUT.set_data(data)
                    self.topic_publisher(self.TOPIC_THRUSTERS_INPUT)
                    print("DATA =", data.data)
            else:
                rospy.logerr("CONNECTION LOST")
            self.rate.sleep()
        self.disconnect()


if __name__ == '__main__':
    thruster_listener = ThrusterListener()
    try:
        thruster_listener.main()
    except rospy.ROSInterruptException:
        thruster_listener.disconnect()
