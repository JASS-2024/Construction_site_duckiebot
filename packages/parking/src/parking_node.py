#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from enum import Enum

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped, AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray, String
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

SEC_TO_NSEC = 1000000000


class States(Enum):
    OFF = 1
    WAITING_FOR_TIMER = 2
    WAITING_FOR_WHEELS = 3
    WAITING_FOR_APRIL_TAG = 4
    WAITING_FOR_OPTITRACK = 5


class MovementType(Enum):
    OFF = 1
    APRIL_TAG_MOVE = 2
    APRIL_TAG_ROT = 3
    OPTITRACK_MOVE = 4
    OPTITRACK_ROT = 5


class ParkingNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ParkingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS

        # Subscriber
        self.april_tag_sub = rospy.Subscriber("~april_tags", AprilTagDetectionArray, self.april_tag_callback, queue_size=1)
        self.wheels_result = rospy.Subscriber("~wheels_result", BoolStamped, None, queue_size=1)
        self.optitrack_sub = rospy.Subscriber("~optitrack", Float32MultiArray, self.optitrack_callback, queue_size=1)
        self.timer_sub = rospy.Subscriber("~timer", BoolStamped, None, queue_size=1)
        self.fsm_sub = rospy.Subscriber("~fsm_signal", BoolStamped, self.fsm_callback, queue_size=1)

        # Publisher
        self.wheels_pub = rospy.Publisher("~wheels", String, queue_size=1)
        self.fsm_result = rospy.Publisher("~fsm_result", BoolStamped, queue_size=1)

        # Variables

        self.optitrack_ts = rospy.Time.now()
        self.april_tag_ts = rospy.Time.now()
        self.wheels_ts = rospy.Time.now()
        self.timer_ts = rospy.Time.now()
        self.status = False
        self.state = States.OFF
        self.movement = MovementType.OFF
        self.x_optitrack = 0
        self.y_optitrack = 0
        self.theta_optitrack = 0

        self.tags_id = []
        self.tags_coords = []
        self.tags_family = []

        self.log("Initialized!")

    def fsm_callback(self, msg):
        if self.status:
            self.log("Node is already active, message ignored")
            return
        self.status = True
        self.state = States.WAITING_FOR_OPTITRACK
        self.movement = MovementType.OPTITRACK_MOVE

    def optitrack_callback(self, msg: Float32MultiArray):
        self.optitrack_ts = rospy.Time.from_sec(msg.data[-1] / SEC_TO_NSEC)
        self.x_optitrack = msg.data[0]
        self.y_optitrack = msg.data[1]
        self.theta_optitrack = msg.data[2]

    def april_tag_callback(self, msg: AprilTagDetectionArray):
        self.april_tag_ts = msg.header.stamp
        self.tags_id = []
        self.tags_family = []
        self.tags_coords = []

        for element in msg.detections:
            self.tags_id.append(element.tag_id)
            self.tags_coords.append(element.corners)
            self.tags_family.append(element.hamming)



if __name__ == "__main__":
    # Initialize the node
    parking_node = ParkingNode(node_name="parking_node")
    # Keep it spinning
    rospy.spin()
