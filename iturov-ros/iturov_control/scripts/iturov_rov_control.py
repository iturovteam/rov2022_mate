#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import yaml
import rospy
import sensor_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv


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


class RovControl(RosHandler):
    def __init__(self):
        super().__init__()
        self.args = rospy.myargv(argv=sys.argv)
        if len(self.args) != 2:
            rospy.logerr("Wrong args!!!")
            sys.exit(1)
        self.joy_config_file = self.args[1]

        self.armed = False
        self.joy_config = {}
        self.get_joy_config()

        self.TOPIC_STATE = TopicClass("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicClass("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.TOPIC_JOY_INPUT = TopicClass("/joy", sensor_msgs.msg.Joy)
        self.TOPIC_RC_OUTPUT = TopicClass("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)

    def main(self):
        self.connect(node="rov_control", rate=30)
        self.start_topic_subscribe()
        while not rospy.is_shutdown():
            if self.connected:
                # state_data = self.TOPIC_STATE.get_data()
                joy_data = self.TOPIC_JOY_INPUT.get_data()
                if joy_data is not None:
                    if not self.armed and joy_data.buttons[self.joy_config["arm"]]:
                        self.arm(True)
                        self.armed = True
                    elif self.armed and joy_data.buttons[self.joy_config["disarm"]]:
                        self.arm(False)
                        self.armed = False
                    elif self.armed:
                        self.generate_rc_override(joy_data)
                    else:
                        pass
            else:
                rospy.logerr("CONNECTION LOST")
            self.rate.sleep()
        self.disconnect()

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()

        data.value = status

        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def generate_rc_override(self, joy_data):
        data = mavros_msgs.msg.OverrideRCIn()

        data.channels[0] = 1500 + int(joy_data.axes[self.joy_config["pitch"]] * 400)
        data.channels[1] = 1500 + int(joy_data.axes[self.joy_config["roll"]] * 400)
        data.channels[2] = 1500 + int(joy_data.axes[self.joy_config["throttle"]] * 400)
        data.channels[3] = 1500 + int(joy_data.axes[self.joy_config["yaw"]] * 400)
        data.channels[4] = 1500 + int(joy_data.axes[self.joy_config["forward"]] * 400)
        data.channels[5] = 1500 + int(joy_data.axes[self.joy_config["lateral"]] * 400)

        self.TOPIC_RC_OUTPUT.set_data(data)
        self.topic_publisher(self.TOPIC_RC_OUTPUT)
        print("STATE: ")
        sys.stdout.write("\033[K")
        print(f"RC OVERRIDE: {data.channels}")
        sys.stdout.write("\033[K")
        sys.stdout.write("\033[A")
        sys.stdout.write("\033[F")

    def get_joy_config(self):
        with open(self.joy_config_file, "r") as file:
            self.joy_config = yaml.load(file)

    def start_topic_subscribe(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_JOY_INPUT)


if __name__ == '__main__':
    rov = RovControl()
    try:
        rov.main()
    except rospy.ROSInterruptException:
        rov.disconnect()
