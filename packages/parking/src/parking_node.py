#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from enum import Enum

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped, AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray, String, Int32
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
    APRIL_TAG = 2
    OPTITRACK = 3


class ParkingNode(DTROS):
    """
    Configuration:
        ~parking_patterns (dict):
            "pattern_number":
                    x_opt (float): X coordinate of point, where the turn should be performed

                    y_opt (float): Y coordinate of point, where the turn should be performed

                    target_april_tag (int): Id of the apriltag that is used as final marker

                    initial_rotation (str): Rotation that should be performed after reaching the (x_opt, y_opt) position

        ~optitrack_positional_threshold (float): The maximum allowed difference between position provided by Optitrack and target position

        ~optitrack_angle_threshold (float): The maximum allowed difference between angle provided by Optitrack and target angle

        ~april_tag_centering_threshold (float): The maximum allowed difference between apriltag center and real center of the picture

        ~april_tag_check_position_x (float): X coordinate of center of target bounding box of the apriltag

        ~april_tag_check_position_y (float): Y coordinate of center of target bounding box of the apriltag

        ~april_tag_area_of_covering_threshold (float): percentage of the bounding box area, which should be covered by apriltag

        ~april_tag_average_size_threshold (float): The maximum allowed difference between average size of apriltag edge and bounding box's edge

        ~april_tag_bounding_box_size (float): length of the apriltag bounding box edge

    Publishers:
        ~wheels (std_msgs::msg::String): Command for movement node to perform a small move

        ~fsm_result (duckietown_msgs::msg::BoolStamped): Signal that node is finished its process

    Subscribers:
        ~april_tags" (duckietown_msgs::msg::AprilTagDetectionArray): Receive list of the detected apriltags

        ~wheels_result" (duckietown_msgs::msg::BoolStamped): Receive signal from wheels that action was executed

        ~optitrack" (std_msgs::msg::Float32MultiArray): Receive positional information from the Optitrack

        ~timer" (duckietown_msgs::msg::BoolStamped): Receive signal every timer_node/signal seconds

        ~fsm_signal" (std_msgs::msg::Int32): Receive signal to start work and number of pattern to execute
    """

    def __init__(self, node_name):
        super(ParkingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Read parameters:
        self.parking_patterns: dict = rospy.get_param('~parking_patterns')
        self.optitrack_positional_threshold = DTParam('~optitrack_positional_threshold', ParamType=ParamType.FLOAT,
                                                      min_value=0, max_value=1)
        self.optitrack_angle_threshold = DTParam('~optitrack_angle_threshold', ParamType=ParamType.FLOAT, min_value=0,
                                                 max_value=1)
        self.april_tag_centering_threshold = DTParam('~april_tag_centering_threshold', ParamType=ParamType.FLOAT,
                                                     min_value=0,
                                                     max_value=1000)
        self.april_tag_check_position_x = DTParam('~april_tag_check_position_x', ParamType=ParamType.FLOAT, min_value=0,
                                                  max_value=1000)
        self.april_tag_check_position_y = DTParam('~april_tag_check_position_y', ParamType=ParamType.FLOAT, min_value=0,
                                                  max_value=1000)
        self.april_tag_area_of_covering_threshold = DTParam('~april_tag_area_of_covering_threshold',
                                                            ParamType=ParamType.FLOAT, min_value=0, max_value=1)
        self.april_tag_average_size_threshold = DTParam('~april_tag_average_size_threshold', ParamType=ParamType.FLOAT,
                                                        min_value=0, max_value=1000)
        self.april_tag_bounding_box_size = DTParam('~april_tag_bounding_box_size', ParamType=ParamType.FLOAT,
                                                   min_value=0, max_value=1000)

        # Subscriber
        self.april_tag_sub = rospy.Subscriber("~april_tags", AprilTagDetectionArray, self.april_tag_callback,
                                              queue_size=1)
        self.wheels_result = rospy.Subscriber("~wheels_result", BoolStamped, self.wheels_callback, queue_size=1)
        self.optitrack_sub = rospy.Subscriber("~optitrack", Float32MultiArray, self.optitrack_callback, queue_size=1)
        self.timer_sub = rospy.Subscriber("~timer", BoolStamped, self.timer_callback, queue_size=1)
        self.fsm_sub = rospy.Subscriber("~fsm_signal", Int32, self.fsm_callback, queue_size=1)

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

        self.current_pattern = ""

        self.log("Initialized!")

    def fsm_callback(self, msg):
        if self.status:
            self.log("Node is already active, message ignored")
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
            return
        if msg.data not in self.parking_patterns.keys():
            self.log("Got unexpected parking pattern")
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
            return
        self.status = True
        self.state = States.WAITING_FOR_OPTITRACK
        self.movement = MovementType.OPTITRACK

    def optitrack_callback(self, msg: Float32MultiArray):
        if not self.state == States.WAITING_FOR_OPTITRACK:
            return
        self.optitrack_ts = rospy.Time.from_sec(msg.data[-1] / SEC_TO_NSEC)
        self.x_optitrack = msg.data[0]
        self.y_optitrack = msg.data[1]
        self.theta_optitrack = msg.data[2]
        self.state = States.WAITING_FOR_TIMER

    def april_tag_callback(self, msg: AprilTagDetectionArray):
        if not self.state == States.WAITING_FOR_APRIL_TAG:
            return
        self.april_tag_ts = msg.header.stamp
        self.tags_id = []
        self.tags_family = []
        self.tags_coords = []

        for element in msg.detections:
            self.tags_id.append(element.tag_id)
            self.tags_coords.append(element.corners)
            self.tags_family.append(element.hamming)
        self.state = States.WAITING_FOR_TIMER

    def wheels_callback(self, msg):
        if not (self.state == States.WAITING_FOR_WHEELS):
            self.log("Got unexpected message from wheels")
            return
        if self.movement == MovementType.OFF:
            self.log("Got unexpected message from wheels, while self.movement is OFF")
        self.wheels_ts = msg.header.stamp
        if self.movement == MovementType.OPTITRACK or self.movement == MovementType.OPTITRACK:
            self.state = States.WAITING_FOR_OPTITRACK
        else:
            self.state = States.WAITING_FOR_APRIL_TAG

    def timer_callback(self, msg):
        self.timer_ts = msg.header.stamp
        if not self.status:
            return
        if not (self.state == States.WAITING_FOR_TIMER):
            return
        if self.movement == MovementType.OPTITRACK:
            movement_command = self.calculate_optitrack_next_move()
            self.log(f"Moving in optitrack mode, movement command is {movement_command}")
            self.log(f"Bot coords: x: {self.x_optitrack}, y: {self.y_optitrack}, theta: {self.theta_optitrack}")
        else:
            movement_command = self.calculate_april_tag_next_move()
            self.log(f"Moving in april_tag mode, movement command is {movement_command}")
            if self.parking_patterns[self.current_pattern]["target_april_tag"] not in self.tags_id:
                self.log("April tag wasn't found")
            else:
                self.log(
                    f"Current April tag params: \n "
                    f"april_tag_centering_threshold: {self.april_tag_centering_threshold} \n "
                    f"april_tag_check_position_x: {self.april_tag_check_position_x} \n "
                    f"april_tag_check_position_y: {self.april_tag_check_position_y} \n "
                    f"april_tag_area_of_covering_threshold: {self.april_tag_area_of_covering_threshold} \n "
                    f"april_tag_average_size_threshold: {self.april_tag_average_size_threshold} \n "
                    f"april_tag_bounding_box_size : {self.april_tag_bounding_box_size} \n")
                tag_index = 0
                for i, elem in self.tags_id:
                    if elem == self.parking_patterns[self.current_pattern]["target_april_tag"]:
                        tag_index = i
                        break
                corners = self.tags_coords[tag_index]
                self.log(
                    f"Current april tag position is {corners}"
                )

        if movement_command == "NONE":
            self.log(f"No need to move, current movement is {self.movement}")
            return
        elif movement_command == "STOP":
            self.log("Parking is finished")
            self.status = False
            self.state = States.OFF
            self.movement = MovementType.OFF
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
        else:
            wheels_msg = String()
            wheels_msg.data = movement_command
            self.state = States.WAITING_FOR_WHEELS
            self.wheels_pub.publish(wheels_msg)

    def calculate_april_tag_next_move(self) -> str:
        if abs(self.x_optitrack - self.parking_patterns[self.current_pattern]["x_opt"]) \
                < self.optitrack_positional_threshold.value:
            self.state = States.WAITING_FOR_TIMER
            self.movement = MovementType.APRIL_TAG
            return "NONE"
        else:
            movement = "FORWARD" if (self.x_optitrack - self.parking_patterns[self.current_pattern]["x_opt"]) * np.cos(
                self.theta_optitrack) > 0 else "BACK"
            return movement

    def calculate_optitrack_next_move(self) -> str:
        if self.parking_patterns[self.current_pattern]["target_april_tag"] not in self.tags_id:
            return self.parking_patterns[self.current_pattern]["initial_rotation"]
        tag_index = 0
        for i, elem in self.tags_id:
            if elem == self.parking_patterns[self.current_pattern]["target_april_tag"]:
                tag_index = i
                break
        corners = self.tags_coords[tag_index]
        if abs(corners[0] - 2 * self.april_tag_check_position_x + corners[2]) > self.april_tag_centering_threshold:
            movement = "RIGHT" if corners[0] - 2 * self.april_tag_check_position_x + corners[2] > 0 else "LEFT"
            return movement
        average_edge_length = 0
        for i in range(4):
            average_edge_length += np.sqrt(
                (corners[2 * i] - corners[(2 * i + 2) % 8]) ** 2 + (corners[2 * i + 1] - corners[(2 * i + 3) % 8]) ** 2)
        average_edge_length /= 4
        if abs(average_edge_length - self.april_tag_bounding_box_size) > self.april_tag_average_size_threshold:
            movement = "FORWARD" if average_edge_length - self.april_tag_bounding_box_size < 0 else "BACK"
            return movement
        return "STOP"


if __name__ == "__main__":
    parking_node = ParkingNode(node_name="parking_node")
    rospy.spin()
