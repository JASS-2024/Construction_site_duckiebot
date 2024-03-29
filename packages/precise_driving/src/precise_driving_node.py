#!/usr/bin/env python3
import time

import numpy as np
import rospy
import cv2
import std_msgs.msg
import os

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import AprilTagDetectionArray, WheelsCmdStamped
from std_msgs.msg import String, Float32MultiArray, Bool

class PreciseDrivingNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(PreciseDrivingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the updatable parameters to the parameters dictionary
        self.params = dict()
        self.params["image_width"] = DTParam("~image_width", param_type=ParamType.INT, min_value=0, max_value=1000)
        self.params["image_height"] = DTParam("~image_height", param_type=ParamType.INT, min_value=0, max_value=1000)
        self.params["center_threshold"] = DTParam("~center_threshold", param_type=ParamType.INT, min_value=0, max_value=255)

        # Read constant parameters from yaml
        self.patterns = rospy.get_param("~patterns")

        # Subscribers
        self.optitrack_coordinates_subscriber = rospy.Subscriber("~optitrack_coordinates", Float32MultiArray, self.optitrack_coordinates_callback, queue_size=1)
        self.apriltags_subscriber = rospy.Subscriber("~april_tags", AprilTagDetectionArray, self.apriltags_callback, queue_size=1)
        self.execution_patterns_sub = rospy.Subscriber("~execution_pattern", String, self.set_pattern, queue_size=1)


        # Publishers
        self.is_finished_publisher = rospy.Publisher("~is_finished", Bool, queue_size=1)
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Initialize variables
        self.execution_pattern = ""

        self.optitrack_timestamp = 0
        self.x_coord = 0
        self.y_coord = 0

        self.apriltags_timestamp = 0
        self.apriltags = {}

        self.log("Initialized!")

    def set_pattern(self, msg: std_msgs.msg.String):
        self.log(f"Received message {msg.data}")
        print(f"Received message {msg.data}")
        if self.execution_pattern != "" and msg.data != "STOP":
            self.log(f"Received pattern {msg.data}, but {self.execution_pattern} is in use. Pattern hasn't been set.")
            return
        if msg.data == "STOP":
            self.log(f"Received")
            self.execution_pattern = ""
            return
        self.execution_pattern = msg.data
        self.execute_pattern()
        self.log(f"Successfully changed pattern to {msg.data}")

    def optitrack_coordinates_callback(self, msg: std_msgs.msg.Float32MultiArray):
        self.optitrack_timestamp = int(msg.data[-1])
        self.x_coord = msg.data[0]
        self.y_coord = msg.data[1]

    def apriltags_callback(self, msg: AprilTagDetectionArray):
        self.apriltags_timestamp = msg.header.stamp.to_nsec()
        for element in msg.detections:
            self.apriltags[element.tag_id] = element.corners

    def execute_pattern(self):
        self.log(f"Executing pattern")
        print("Executing pattern")
        self.log(len(self.patterns["forward"]))

        current_tag_index = 0
        current_move = self.patterns[self.execution_pattern]["initial_move"]
        while True:
            self.perform_action(current_move, current_tag_index)
            if current_tag_index >= len(self.patterns["forward"]["tags_sequence"]) - 1:
                self.execution_pattern = ""
                break

    def perform_action(self, action, current_tag_id):
        self.log(f"Performing action {action}")
        if action == "FORWARD" or action == "BACKWARD":
            for i in range(10):
                self.move(action)

    def move(self, direction):
        self.log(f"Moving {direction}")
        rate = rospy.Rate(0.1)
        self.wheel_publisher.publish(WheelsCmdStamped(vel_left=0.5, vel_right=0.5))
        # rate.sleep()
        time.sleep(0.1)
        self.wheel_publisher.publish(WheelsCmdStamped(vel_left=0.0, vel_right=0.0))


    def turn(self, direction):
        pass


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = PreciseDrivingNode(node_name="precise_driving_node")
    # Keep it spinning
    rospy.spin()
