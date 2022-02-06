#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import dynamic_reconfigure.client

import sensor_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv


class ParamClass:
    def __init__(self, name: str, timeout: int):
        self.__name = name
        self.__timeout = timeout
        self.__config = None
        self.reconfigure_flag = True

    def set_params(self, config):
        self.reconfigure_flag = True
        self.__config = config

    def get_params(self):
        return self.__config

    def get_name(self):
        return self.__name

    def get_timeout(self):
        return self.__timeout


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

    def connect(self, node: str, rate: int = 30):
        rospy.init_node(node, anonymous=False)
        self.rate = rospy.Rate(rate)
        self.connected = True
        rospy.loginfo("PID Rov Control is up ...")
        # rospy.spin()

    def disconnect(self):
        if self.connected:
            rospy.logwarn("shutting down PID Rov Control ...")
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
    def param_subscriber(param: ParamClass):
        dynamic_reconfigure.client.Client(param.get_name(), param.get_timeout(), param.set_params)

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


class PIDRovControl(RosHandler):
    def __init__(self):
        super().__init__()

        self.armed = False
        self.pid_config = None
        self.joy_config = None
        self.joy_data = None

        self.PARAM_PID = ParamClass("/PID_Params", 10)
        self.PARAM_JOY = ParamClass("/Joy_Params", 10)
        self.TOPIC_STATE = TopicClass("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicClass("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.TOPIC_JOY_INPUT = TopicClass("/joy", sensor_msgs.msg.Joy)
        self.TOPIC_RC_OUTPUT = TopicClass("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)

        self.param_subscriber(self.PARAM_PID)
        self.param_subscriber(self.PARAM_JOY)
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_JOY_INPUT)

    def main(self):
        self.connect(node="rov_control", rate=30)

        while not rospy.is_shutdown():
            if self.connected:
                if self.PARAM_PID.reconfigure_flag:
                    self.get_pid_config()
                if self.PARAM_JOY.reconfigure_flag:
                    self.get_joy_config()
                self.get_joy_data()

                if self.joy_data is not None:
                    if not self.armed and self.joy_data.buttons[self.joy_config["Arm_Button"]]:
                        self.arm(True)
                        self.armed = True
                    elif self.armed and self.joy_data.buttons[self.joy_config["Disarm_Button"]]:
                        self.arm(False)
                        self.armed = False
                    elif self.armed:
                        self.generate_rc_override()
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

    def generate_rc_override(self):
        data = mavros_msgs.msg.OverrideRCIn()

        data.channels[0] = 1500 + int(self.joy_data.axes[self.joy_config["Pitch_Axis"]] * 400)
        data.channels[1] = 1500 + int(self.joy_data.axes[self.joy_config["Roll_Axis"]] * 400)
        data.channels[2] = 1500 + int(self.joy_data.axes[self.joy_config["Z_Axis"]] * 400)
        data.channels[3] = 1500 + int(self.joy_data.axes[self.joy_config["Yaw_Axis"]] * 400)
        data.channels[4] = 1500 + int(self.joy_data.axes[self.joy_config["X_Axis"]] * 400)
        data.channels[5] = 1500 + int(self.joy_data.axes[self.joy_config["Y_Axis"]] * 400)

        self.TOPIC_RC_OUTPUT.set_data(data)
        self.topic_publisher(self.TOPIC_RC_OUTPUT)
        """
        print(f"P: {self.pid_config['P_Roll']}  I: {self.pid_config['I_Roll']}  D: {self.pid_config['D_Roll']}")
        sys.stdout.write("\033[K")
        print(f"RC OVERRIDE: {data.channels[:6]}")
        sys.stdout.write("\033[K")
        sys.stdout.write("\033[A")
        sys.stdout.write("\033[F")
        """

    def get_pid_config(self):
        self.pid_config = self.PARAM_PID.get_params()
        self.PARAM_PID.reconfigure_flag = False

    def get_joy_config(self):
        self.joy_config = self.PARAM_JOY.get_params()
        self.PARAM_JOY.reconfigure_flag = False

    def get_joy_data(self):
        self.joy_data = self.TOPIC_JOY_INPUT.get_data()


if __name__ == "__main__":
    rov = PIDRovControl()
    try:
        rov.main()
    except rospy.ROSInterruptException:
        rov.disconnect()
