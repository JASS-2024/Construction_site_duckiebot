#!/usr/bin/env python3
# import time
from typing import Optional

import numpy as np
import rospy
# import cv2
from enum import Enum
# from cv_bridge import CvBridge
import os

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped, AprilTagDetectionArray, WheelsCmdStamped, WheelEncoderStamped
# from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String, Int32
# from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

SEC_TO_NSEC = 1000000000


class States(Enum):
    OFF = 1
    WAITING_FOR_TIMER = 2
    WAITING_FOR_APRIL_TAG = 4
    WAITING_FOR_LEFT_ENCODER = 8
    WAITING_FOR_RIGHT_ENCODER = 9


class MovementType(Enum):
    OFF = 1
    APRIL_TAG = 2
    OPTITRACK = 3


class MovementPattern(Enum):
    MOVING_RIGHT = 9
    MOVING_LEFT = 10
    MOVING_FORWARD = 11
    MOVING_BACKWARD = 12
    MOVING_FORWARD_SLOW = 13
    MOVING_BACKWARD_SLOW = 14
    MOVING_RIGHT_SLOW = 16
    MOVING_LEFT_SLOW = 17
    STOPPED = 15


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
        self.optitrack_positional_threshold = DTParam('~optitrack_positional_threshold', param_type=ParamType.FLOAT,
                                                      min_value=0, max_value=1)
        self.optitrack_angle_threshold = DTParam('~optitrack_angle_threshold', param_type=ParamType.FLOAT, min_value=0,
                                                 max_value=1)
        self.april_tag_centering_threshold = DTParam('~april_tag_centering_threshold', param_type=ParamType.FLOAT,
                                                     min_value=0,
                                                     max_value=1000)
        self.april_tag_check_position_x = DTParam('~april_tag_check_position_x', param_type=ParamType.FLOAT,
                                                  min_value=0,
                                                  max_value=1000)
        self.april_tag_check_position_y = DTParam('~april_tag_check_position_y', param_type=ParamType.FLOAT,
                                                  min_value=0,
                                                  max_value=1000)
        self.april_tag_area_of_covering_threshold = DTParam('~april_tag_area_of_covering_threshold',
                                                            param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.april_tag_average_size_threshold = DTParam('~april_tag_average_size_threshold', param_type=ParamType.FLOAT,
                                                        min_value=0, max_value=1000)
        self.april_tag_bounding_box_size = DTParam('~april_tag_bounding_box_size', param_type=ParamType.FLOAT,
                                                   min_value=0, max_value=1000)

        self.speed_changing_threshold = DTParam('~speed_changing_threshold', param_type=ParamType.FLOAT,
                                                min_value=0, max_value=1000)

        self.move_speed = DTParam("~move_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.rotation_speed = DTParam("~rotation_speed", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        self.move_speed_slow = DTParam("~move_speed_slow", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.rotation_speed_slow = DTParam("~rotation_speed_slow", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        self.move_time = DTParam("~move_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.rotation_time = DTParam("~rotation_time", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        self.move_time_slow = DTParam("~move_time_slow", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.rotation_time_slow = DTParam("~rotation_time_slow", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        self.sleep_after_move = DTParam('~sleep_after_move', param_type=ParamType.FLOAT,
                                                min_value=0, max_value=10)
        self.counter = 0
        self.LED_light = "BLUE"


        # Topic for movement
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self.wheel_publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        # Subscriber
        # self.april_tag_sub = rospy.Subscriber("~april_tags", AprilTagDetectionArray, self.april_tag_callback,
        #                                       queue_size=1)
        #
        self.april_tag_sub = rospy.Subscriber("~april_tags", AprilTagDetectionArray, None,
                                              queue_size=1)

        self.fsm_sub = rospy.Subscriber("~fsm_signal", Int32, None, queue_size=1)

        self.left_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, None, queue_size=1)
        self.right_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, None, queue_size=1)
        #
        # self.fsm_sub = rospy.Subscriber("~fsm_signal", Int32, self.fsm_callback, queue_size=1)
        #
        # self.left_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_encoder_call_back, queue_size=1)
        # self.right_wheel_encoder = rospy.Subscriber(f"/{vehicle_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_encoder_call_back, queue_size=1)



        # Publisher
        # self.wheels_pub = rospy.Publisher("~wheels", String, queue_size=1)
        self.fsm_result = rospy.Publisher("~fsm_result", BoolStamped, queue_size=1)
        self.change_pattern = rospy.Publisher('~change_pattern', String, queue_size=1)


        # self.bridge = CvBridge()

        # Variables
        self.left_encoder_last_value = 0
        self.right_encoder_last_value = 0
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
        self.centers = []

        self.current_pattern = ""
        self.current_movement = MovementPattern.STOPPED
        self.movement_command = MovementPattern.STOPPED

        self.last_seen_a_tag_center = []

        self.make_move(MovementPattern.STOPPED)
        self.log("Initialized!")

    def left_encoder_call_back(self, msg):
        if self.state == States.WAITING_FOR_LEFT_ENCODER:
            print(f"Left encoder delta {msg.data - self.left_encoder_last_value}", end=", ")
            self.left_encoder_last_value = msg.data
            self.state = States.WAITING_FOR_RIGHT_ENCODER

    def right_encoder_call_back(self, msg):
        if self.state == States.WAITING_FOR_RIGHT_ENCODER:
            print(f"Right encoder delta {msg.data - self.right_encoder_last_value}")
            self.right_encoder_last_value = msg.data
            rospy.sleep(self.sleep_after_move.value)
            self.state = States.WAITING_FOR_APRIL_TAG

    def send_LED_request(self, color="RED"):
        """the LED color changing function"""
        if self.LED_light == color:
            pass
        else:
            pattern = String()
            pattern.data = color
            self.change_pattern.publish(pattern)
            self.LED_light = color

    def fsm_callback(self, msg):
        self.log("Received FSM")
        if self.status:
            self.log("Node is already active, message ignored")
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
            return
        if str(msg.data) not in self.parking_patterns.keys():
            self.log("Got unexpected parking pattern")
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
            return
        self.log("Node started")
        self.status = True
        self.current_pattern = str(msg.data)
        self.state = States.WAITING_FOR_APRIL_TAG
        self.movement = MovementType.APRIL_TAG

    def april_tag_callback(self, msg: AprilTagDetectionArray):
        if not self.state == States.WAITING_FOR_APRIL_TAG:
            return
        self.april_tag_ts = msg.header.stamp
        self.tags_id = []
        self.tags_family = []
        self.tags_coords = []
        self.centers = []
        for element in msg.detections:
            self.tags_id.append(element.tag_id)
            self.tags_coords.append(element.corners)
            self.tags_family.append(element.hamming)
            self.centers.append(element.center)
            if element.tag_id == self.parking_patterns[self.current_pattern]["target_april_tag"]:
                self.last_seen_a_tag_center = element.center
        # self.state = States.WAITING_FOR_TIMER

        self.movement_command = self.calculate_april_tag_next_move()
        self.log(f"Moving in april_tag mode, movement command is {self.movement_command}")
        if self.parking_patterns[self.current_pattern]["target_april_tag"] not in self.tags_id:
            self.log("April tag wasn't found")
        else:
            self.log(
                f"Current April tag params: \n "
                f"april_tag_centering_threshold: {self.april_tag_centering_threshold.value} \n "
                f"april_tag_check_position_x: {self.april_tag_check_position_x.value} \n "
                f"april_tag_check_position_y: {self.april_tag_check_position_y.value} \n "
                f"april_tag_area_of_covering_threshold: {self.april_tag_area_of_covering_threshold.value} \n "
                f"april_tag_average_size_threshold: {self.april_tag_average_size_threshold.value} \n "
                f"april_tag_bounding_box_size : {self.april_tag_bounding_box_size.value} \n")
            tag_index = 0
            for i, elem in enumerate(self.tags_id):
                if elem == self.parking_patterns[self.current_pattern]["target_april_tag"]:
                    tag_index = i
                    break
            corners = self.tags_coords[tag_index]
            average_edge_length = 0
            for i in range(4):
                average_edge_length += np.sqrt(
                    (corners[2 * i] - corners[(2 * i + 2) % 8]) ** 2 + (
                            corners[2 * i + 1] - corners[(2 * i + 3) % 8]) ** 2)
            average_edge_length /= 4
            self.log(
                f"Current april tag position is {corners}"
                f"Its centers are {self.centers[tag_index]}"
                f"Its edge length is {average_edge_length}"
            )

        if self.movement_command is None:
            self.log(f"No need to change move, current movement is {self.current_movement}")
            self.state = States.WAITING_FOR_APRIL_TAG
            return
        elif self.state == States.OFF:
            self.send_LED_request("PINK")
            self.log("Parking is finished")
            rospy.sleep(3)
            self.make_move(MovementPattern.STOPPED)
            self.status = False
            self.state = States.OFF
            self.movement = MovementType.OFF
            self.x_optitrack = 0
            self.y_optitrack = 0
            self.theta_optitrack = 0

            self.tags_id = []
            self.tags_coords = []
            self.tags_family = []
            self.centers = []

            self.current_movement = MovementPattern.STOPPED
            self.movement_command = MovementPattern.STOPPED

            self.status = True
            self.current_pattern = str(-1 * int(self.current_pattern))
            self.state = States.WAITING_FOR_APRIL_TAG
            self.movement = MovementType.APRIL_TAG

            self.last_seen_a_tag_center = []
            fsm_message = BoolStamped()
            fsm_message.data = True
            fsm_message.header.stamp = rospy.Time.now()
            self.fsm_result.publish(fsm_message)
        else:
            self.send_LED_request("GREEN")
            self.log("Published for wheels")
            self.current_movement = self.movement_command
            self.make_move(self.movement_command)
            # wheels_msg = String()
            # wheels_msg.data = movement_command
            self.state = States.WAITING_FOR_LEFT_ENCODER
            # self.wheels_pub.publish(wheels_msg)

    # def timer_callback(self, msg):
    #     self.timer_ts = msg.header.stamp
    #     if not self.status:
    #         return
    #     if self.movement == MovementType.APRIL_TAG:
    #         self.movement_command = self.calculate_april_tag_next_move()
    #         self.log(f"Moving in april_tag mode, movement command is {self.movement_command}")
    #         if self.parking_patterns[self.current_pattern]["target_april_tag"] not in self.tags_id:
    #             self.log("April tag wasn't found")
    #         else:
    #             self.log(
    #                 f"Current April tag params: \n "
    #                 f"april_tag_centering_threshold: {self.april_tag_centering_threshold.value} \n "
    #                 f"april_tag_check_position_x: {self.april_tag_check_position_x.value} \n "
    #                 f"april_tag_check_position_y: {self.april_tag_check_position_y.value} \n "
    #                 f"april_tag_area_of_covering_threshold: {self.april_tag_area_of_covering_threshold.value} \n "
    #                 f"april_tag_average_size_threshold: {self.april_tag_average_size_threshold.value} \n "
    #                 f"april_tag_bounding_box_size : {self.april_tag_bounding_box_size.value} \n")
    #             tag_index = 0
    #             for i, elem in enumerate(self.tags_id):
    #                 if elem == self.parking_patterns[self.current_pattern]["target_april_tag"]:
    #                     tag_index = i
    #                     break
    #             corners = self.tags_coords[tag_index]
    #             average_edge_length = 0
    #             for i in range(4):
    #                 average_edge_length += np.sqrt(
    #                     (corners[2 * i] - corners[(2 * i + 2) % 8]) ** 2 + (
    #                                 corners[2 * i + 1] - corners[(2 * i + 3) % 8]) ** 2)
    #             average_edge_length /= 4
    #             self.log(
    #                 f"Current april tag position is {corners}"
    #                 f"Its centers are {self.centers[tag_index]}"
    #                 f"Its edge length is {average_edge_length}"
    #             )
    #
    #     if self.movement_command is None:
    #         self.log(f"No need to change move, current movement is {self.current_movement}")
    #         self.state = States.WAITING_FOR_APRIL_TAG
    #         return
    #     elif self.state == States.OFF:
    #         self.send_LED_request("PINK")
    #         self.log("Parking is finished")
    #         rospy.sleep(3)
    #         # self.send_LED_request("WHITE")
    #         self.make_move(MovementPattern.STOPPED)
    #         self.status = False
    #         self.state = States.OFF
    #         self.movement = MovementType.OFF
    #         self.x_optitrack = 0
    #         self.y_optitrack = 0
    #         self.theta_optitrack = 0
    #
    #         self.tags_id = []
    #         self.tags_coords = []
    #         self.tags_family = []
    #         self.centers = []
    #
    #         self.current_movement = MovementPattern.STOPPED
    #         self.movement_command = MovementPattern.STOPPED
    #
    #         self.status = True
    #         self.current_pattern = str(-1 * int(self.current_pattern))
    #         self.state = States.WAITING_FOR_APRIL_TAG
    #         self.movement = MovementType.APRIL_TAG
    #
    #         self.last_seen_a_tag_center = []
    #         fsm_message = BoolStamped()
    #         fsm_message.data = True
    #         fsm_message.header.stamp = rospy.Time.now()
    #         self.fsm_result.publish(fsm_message)
    #     else:
    #         self.log("Published for wheels")
    #         self.current_movement = self.movement_command
    #         self.make_move(self.movement_command)
    #         # wheels_msg = String()
    #         # wheels_msg.data = movement_command
    #         self.state = States.WAITING_FOR_LEFT_ENCODER
    #         # self.wheels_pub.publish(wheels_msg)

    def calculate_april_tag_next_move(self) -> Optional[MovementPattern]:
        if self.parking_patterns[self.current_pattern]["target_april_tag"] not in self.tags_id:
            if len(self.last_seen_a_tag_center) == 0:
                self.log("Exiting first if")
                initial_rotation = MovementPattern.MOVING_LEFT \
                    if self.parking_patterns[self.current_pattern]["initial_rotation"] == "LEFT" \
                    else MovementPattern.MOVING_RIGHT
                # if initial_rotation == self.current_movement:
                #     return None
                # else:
                return initial_rotation
            else:
                if self.last_seen_a_tag_center[0] > self.april_tag_check_position_x.value:
                    self.log("April tag went left, going right")
                    result_rot = MovementPattern.MOVING_RIGHT
                else:
                    self.log("April tag went right, going left")
                    result_rot = MovementPattern.MOVING_LEFT
                # if result_rot == self.current_movement:
                #     return None
                # else:
                return result_rot
        tag_index = 0
        for i, elem in enumerate(self.tags_id):
            if elem == self.parking_patterns[self.current_pattern]["target_april_tag"]:
                tag_index = i
                break
        corners = self.tags_coords[tag_index]
        centers = self.centers[tag_index]
        if abs(centers[0] - self.april_tag_check_position_x.value) > self.april_tag_centering_threshold.value:
            movement = MovementPattern.MOVING_RIGHT_SLOW \
                if centers[0] - self.april_tag_check_position_x.value > 0 else MovementPattern.MOVING_LEFT_SLOW
            self.log(f"center_x {centers[0]}")
            self.log(f"desired_position {self.april_tag_check_position_x.value}")
            self.log("Exiting second if")
            return movement
        elif (self.current_movement == MovementPattern.MOVING_LEFT or
              self.current_movement == MovementPattern.MOVING_RIGHT or
              self.current_movement == MovementPattern.MOVING_RIGHT_SLOW or
              self.current_movement == MovementPattern.MOVING_LEFT_SLOW):
            return MovementPattern.STOPPED
        average_edge_length = 0
        for i in range(4):
            average_edge_length += np.sqrt(
                (corners[2 * i] - corners[(2 * i + 2) % 8]) ** 2 + (corners[2 * i + 1] - corners[(2 * i + 3) % 8]) ** 2)
        average_edge_length /= 4
        if (abs(average_edge_length - self.april_tag_bounding_box_size.value) >
                self.april_tag_average_size_threshold.value):
            if average_edge_length - self.april_tag_bounding_box_size.value >= 0:
                movement = MovementPattern.MOVING_BACKWARD
            elif average_edge_length < self.speed_changing_threshold.value:
                movement = MovementPattern.MOVING_FORWARD
            else:
                movement = MovementPattern.MOVING_FORWARD_SLOW
            self.log("Exiting third if")
            # if movement == self.current_movement:
            #     return None
            # else:
            return movement
        self.state = States.OFF
        return MovementPattern.STOPPED

    def make_move(self, next_pattern):
        # self.counter+=1
        # self.log(f"Make move is called with pattern {next_pattern}")
        start_moving = rospy.Time.now()
        # start = rospy.Time.now()
        if next_pattern == MovementPattern.MOVING_FORWARD:  # Move forward
            # self.send_LED_request("BLUE")

            message = WheelsCmdStamped(vel_left=self.move_speed.value,
                                       vel_right=self.move_speed.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.move_time.value)

        elif next_pattern == MovementPattern.MOVING_BACKWARD:  # Move Back
            # self.send_LED_request("BLUE")

            message = WheelsCmdStamped(vel_left=-self.move_speed.value,
                                       vel_right=-self.move_speed.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.move_time.value)

        elif next_pattern == MovementPattern.MOVING_RIGHT:  # Rotate Right
            # self.send_LED_request("BLUE")

            message = WheelsCmdStamped(vel_left=self.rotation_speed.value,
                                       vel_right=-self.rotation_speed.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.rotation_time.value)

        elif next_pattern == MovementPattern.MOVING_LEFT:  # Rotate Left
            # self.send_LED_request("BLUE")

            message = WheelsCmdStamped(vel_left=-self.rotation_speed.value,
                                       vel_right=self.rotation_speed.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.rotation_time.value)
        elif next_pattern == MovementPattern.MOVING_FORWARD_SLOW:  # Move forward
            # self.send_LED_request("GREEN")

            message = WheelsCmdStamped(vel_left=self.move_speed_slow.value,
                                       vel_right=self.move_speed_slow.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.move_time_slow.value)
        elif next_pattern == MovementPattern.MOVING_LEFT_SLOW:  # Move forward
            # self.send_LED_request("GREEN")

            message = WheelsCmdStamped(vel_left=-self.rotation_speed_slow.value,
                                       vel_right=self.rotation_speed_slow.value)
            self.wheel_publisher.publish(message)
            rospy.sleep(self.rotation_time_slow.value)
        elif next_pattern == MovementPattern.MOVING_RIGHT_SLOW:  # Move forward
            # self.send_LED_request("GREEN")

            message = WheelsCmdStamped(vel_left=self.rotation_speed_slow.value,
                                       vel_right=-self.rotation_speed_slow.value)
            start_moving = rospy.Time.now()
            self.wheel_publisher.publish(message)
            # start = rospy.Time.now()
            rospy.sleep(self.rotation_time_slow.value)
        # self.log(f"Sleeping time is {rospy.Time.now() - start}")
        message = WheelsCmdStamped(vel_left=0, vel_right=0)
        # self.log(f"Moving time is {rospy.Time.now() - start_moving}")
        self.wheel_publisher.publish(message)

    def on_shutdown(self):
        self.make_move(MovementPattern.STOPPED)


if __name__ == "__main__":
    parking_node = ParkingNode(node_name="parking_node")
    rospy.spin()
