#!/usr/bin/env python3
import os
import time

import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String


class ParkingNode(DTROS):


    def __init__(self, node_name):
        super(ParkingNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        # Initialize params
        self.params = dict()

        self.params["move_speed"] = DTParam("~move_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["rotation_speed"] = DTParam("~rotation_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["move_time"] = DTParam("~move_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["rotation_time"] = DTParam("~rotation_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        # Open subscription for type of movement
        self.make_move_name = rospy.Subscriber(
            '~make_move_name', String, self.make_move, queue_size=1
        )

        # Open publisher for wheels

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self.wheel_publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        # Open publisher for stop signal

        self.move_stop = rospy.Publisher(
            "~move_stop", BoolStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.log("Initialized!")

    def make_move(self, msg):
        if msg.data == "FORWARD": # Move forward

            message = WheelsCmdStamped(vel_left=self.params["move_speed"].value,
                                       vel_right=self.params["move_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["move_time"].value)

        elif msg.data == "BACK": # Move Back

            message = WheelsCmdStamped(vel_left=-self.params["move_speed"].value,
                                       vel_right=-self.params["move_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["move_time"].value)

        elif msg.data == "RIGHT": # Rotate Right

            message = WheelsCmdStamped(vel_left=self.params["rotation_speed"].value,
                                       vel_right=-self.params["rotation_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["rotation_time"].value)

        elif msg.data == "LEFT": # Rotate Left

            message = WheelsCmdStamped(vel_left=-self.params["rotation_speed"].value,
                                       vel_right=self.params["rotation_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["rotation_time"].value)

        # Stop any move

        message = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.wheel_publisher.publish(message)

        # Inform about movement stop

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True

        self.move_stop.publish(msg)


if __name__ == "__main__":
    # Initialize the node
    parking_node = ParkingNode(node_name="parking_node")
    # Keep it spinning
    rospy.spin()
