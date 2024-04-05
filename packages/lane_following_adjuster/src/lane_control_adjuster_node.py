#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped, WheelsCmdStamped, WheelEncoderStamped, FSMState
from duckietown_msgs.srv import SetFSMState
from std_msgs.msg import Bool

import os


class LaneControlAdjusterNode(DTROS):
    def __init__(self, node_name):
        super(LaneControlAdjusterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.get_state = ""
        self.get_state_sub = rospy.Subscriber('~mode', FSMState, self.state_call_back, queue_size=1)
        self.set_state = rospy.ServiceProxy("~set_state", SetFSMState)

        self.threshold = DTParam("~threshold", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.check_interval = DTParam("~check_interval", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.full_throtle_time = DTParam("~full_throtle_time", param_type=ParamType.FLOAT, min_value=0, max_value=1000)

        vehicle_name = os.environ['VEHICLE_NAME']

        self.left_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped,
                                                   self.left_encoder_call_back, queue_size=1)
        self.right_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/right_wheel_encoder_node/tick",
                                                    WheelEncoderStamped, self.right_encoder_call_back, queue_size=1)
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.switcher_pub = rospy.Publisher("~switcher", Bool, queue_size=1)


        self.left_wheel_encoder_value = 0
        self.right_wheel_encoder_value = 0

        self.current_delta_left = 0
        self.current_delta_right = 0

    def state_call_back(self, msg):
        self.get_state = msg.state

    def right_encoder_call_back(self, msg):
        if self.get_state == "LANE_FOLLOWING":
            self.current_delta_right = abs(msg.data - self.right_wheel_encoder_value)
            self.right_wheel_encoder_value = msg.data
            rospy.sleep(self.check_interval.value)


    def left_encoder_call_back(self, msg):
        if self.get_state == "LANE_FOLLOWING":
            self.current_delta_left = abs(msg.data - self.left_wheel_encoder_value)
            self.left_wheel_encoder_value = msg.data
            rospy.sleep(self.check_interval.value)

    def run(self):
        while True:
            print(self.current_delta_left, end=", ")
            print(self.current_delta_right)
            self.log(self.get_state)
            if self.get_state == "LANE_FOLLOWING" and self.current_delta_left < self.threshold.value and self.current_delta_right < self.threshold.value:
                self.log("Performing adjustment")
                self.set_state("NORMAL_JOYSTICK_CONTROL")
                switcher_msg = Bool()
                switcher_msg.data = False
                self.switcher_pub.publish(switcher_msg)
                message = WheelsCmdStamped(vel_left=1,
                                           vel_right=1)
                self.wheel_publisher.publish(message)
                rospy.sleep(self.full_throtle_time.value)
                message = WheelsCmdStamped(vel_left=0,
                                           vel_right=0)
                self.wheel_publisher.publish(message)
                switcher_msg = Bool()
                switcher_msg.data = True
                self.switcher_pub.publish(switcher_msg)
                self.set_state("LANE_FOLLOWING")

            rospy.sleep(self.check_interval.value)


if __name__ == "__main__":
    node = LaneControlAdjusterNode(node_name="lane_control_adjuster_node")
    node.run()
    rospy.spin()
