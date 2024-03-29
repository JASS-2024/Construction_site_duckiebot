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

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()

        self.params["move_speed"] = DTParam("~move_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["rotation_speed"] = DTParam("~rotation_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["move_time"] = DTParam("~move_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params["rotation_time"] = DTParam("~rotation_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        # Initialize variables
        # self.bridge = CvBridge()

        # TODO: publisher

        # Construct publishers
        # self.pub_obstacle_image = rospy.Publisher(
        #     "~debug/segmentation/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        # )
        #
        # self.pub_stop = rospy.Publisher(
        #     "~stop_request", BoolStamped, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        # )

        # Construct subscribers
        # self.duckie_image = rospy.Subscriber(
        #     "~duckie_image",  CompressedImage, self.processImageMessage, buff_size=10000000, queue_size=1
        # )

        # TODO:
        # make some decisions on format of the data you will get from ivan
        # and subscribe to it when you will decide on the names and formats

        # self.duckie_image = rospy.Subscriber(
        #     "~",  CompressedImage, self.processImageMessage, buff_size=10000000, queue_size=1
        # )

        self.make_move_name = rospy.Subscriber(
            '~make_move_name', String, self.make_move, queue_size=1
        )

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self.wheel_publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.move_stop = rospy.Publisher(
            "~move_stop", BoolStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.log("Initialized!")

    def make_move(self, msg):
        print("Start moving")

        print("Current Values:")
        print("    move_speed: ", self.params["move_speed"].value)
        print("    rotation_speed: ", self.params["rotation_speed"].value)
        print("    move_time: ", self.params["move_time"].value)
        print("    rotation_time:", self.params["rotation_time"].value)

        if msg.data == "FORWARD":
            print("Move forward")
            message = WheelsCmdStamped(vel_left=self.params["move_speed"].value,
                                       vel_right=self.params["move_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["move_time"].value)

        elif msg.data == "BACK":
            print("Move back")

            message = WheelsCmdStamped(vel_left=-self.params["move_speed"].value,
                                       vel_right=-self.params["move_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["move_time"].value)

        elif msg.data == "RIGHT":
            print("Rotate right")

            message = WheelsCmdStamped(vel_left=self.params["rotation_speed"].value,
                                       vel_right=-self.params["rotation_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["rotation_time"].value)

        elif msg.data == "LEFT":
            print("Rotate Left")

            message = WheelsCmdStamped(vel_left=-self.params["rotation_speed"].value,
                                       vel_right=self.params["rotation_speed"].value)
            self.wheel_publisher.publish(message)
            time.sleep(self.params["rotation_time"].value)

        print("Stop move or rotate")

        message = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.wheel_publisher.publish(message)

        print("Send BoolStamped True")

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True

        self.move_stop.publish(msg)


if __name__ == "__main__":
    # Initialize the node
    parking_node = ParkingNode(node_name="parking_node")
    # Keep it spinning
    rospy.spin()
