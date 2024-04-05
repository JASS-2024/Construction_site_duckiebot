#!/usr/bin/env python3
import os
import time

import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import Range



class CheckDistanceNode(DTROS):


    def __init__(self, node_name):
        super(CheckDistanceNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        # Initialize params
        self.params = dict()

        self.params["distance"] = DTParam("~distance", param_type=ParamType.FLOAT, min_value=0, max_value=10)

        self.working = True
        self.distance_was_shorter = True

        vehicle_name = os.environ['VEHICLE_NAME']

        # Open subscription for type of movement

        range_topic = f"/{vehicle_name}/front_center_tof_driver_node/range"

        self.check_distance_sub = rospy.Subscriber(
            range_topic,
            Range,
            self.check_distance, queue_size=1
        )

        self.switch_state_sub = rospy.Subscriber(
            '~switch_check_distance', BoolStamped, self.switch_state, queue_size=1
        )

        # Open publisher for wheels

        emergency_stop_topic = f"/{vehicle_name}/wheels_driver_node/emergency_stop"

        self.wheel_publisher = rospy.Publisher(
            emergency_stop_topic, BoolStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )


        self.log("Initialized!")


    def check_distance(self, msg):
        if not self.working:
            return

        # Access distance data from the Range message
        distance = msg.range * 100
        # Do something with the distance data, such as printing it

        bool_stamped_msg = BoolStamped()
        bool_stamped_msg.header.stamp = rospy.Time.now()

        if distance <= self.params["distance"].value and not self.distance_was_shorter:
            bool_stamped_msg.data = True
            self.distance_was_shorter = True

            self.wheel_publisher.publish(bool_stamped_msg)

            print("Send emergency stop message")
        elif distance > self.params["distance"].value and self.distance_was_shorter:
            bool_stamped_msg.data = False
            self.distance_was_shorter = False

            self.wheel_publisher.publish(bool_stamped_msg)

            print("Send stop emergency stop message")

    def switch_state(self, msg):
        if msg.data:
            self.working = True
        else:
            self.working = False

            bool_stamped_msg = BoolStamped()
            bool_stamped_msg.header.stamp = rospy.Time.now()

            bool_stamped_msg.data = False
            self.distance_was_shorter = False

            self.wheel_publisher.publish(bool_stamped_msg)

if __name__ == "__main__":
    # Initialize the node
    check_distance_node = CheckDistanceNode(node_name="check_distance_node")
    # Keep it spinning
    rospy.spin()
