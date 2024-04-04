#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import apriltag
import os
# from threading import Thread
# from concurrent.futures import ThreadPoolExecutor
# from turbojpeg import TurboJPEG, TJPF_GRAY
# from image_geometry import PinholeCameraModel
# from std_msgs.msg import Bool
# from std_msgs.msg import Int32MultiArray
# from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import Transform, Vector3, Quaternion
from duckietown_msgs.msg import BoolStamped, AprilTagDetection, AprilTagDetectionArray, FSMState, WheelsCmdStamped
from duckietown_msgs.srv import ChangePattern, SetFSMState
from std_msgs.msg import String, Int32, Bool, Float32


class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION
        )
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.bridge = CvBridge()
        self.tag_queue = [False] * 10
        self.queue = False
        self.log('apriltag_init')
        self._type_dict = rospy.get_param("~type_dict")
        self.parking_state = False
        self.slow = False
        self.parking_finished = True
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        # debug_values
        self.params = dict()
        self.params["cent_x"] = DTParam("~cent_x", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_y"] = DTParam("~cent_y", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_x_d"] = DTParam("~cent_x_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_y_d"] = DTParam("~cent_y_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["size"] = DTParam("~size", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["size_d"] = DTParam("~size_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["resize"] = DTParam("~resize", param_type=ParamType.INT, min_value=0, max_value=1000)

        # faking the parking-triger apriltag number
        self.params["parking_trigger_apriltag"] = DTParam("~parking_trigger_apriltag", param_type=ParamType.INT,
                                                          min_value=0, max_value=1000)

        # Publisher

        self.changeVBarPub = rospy.Publisher(
            "~change_vbar", Float32, queue_size=1
        )
        self.tags_message = rospy.Publisher('~tags_array', AprilTagDetectionArray, queue_size=1)
        self.change_pattern = rospy.Publisher('~change_pattern', String, queue_size=1)
        self.set_state = rospy.ServiceProxy("~set_state", SetFSMState)
        self.set_parking = rospy.Publisher("~fsm_signal", Int32, queue_size=1)
        self.switcher_pub = rospy.Publisher("~switcher", Bool, queue_size=1)
        self.wheel_publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        # self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)

        # Subscriber
        self._img_sub = rospy.Subscriber("~image", CompressedImage, self.process_image, queue_size=1, buff_size="20MB")
        self.get_state = rospy.Subscriber('~mode', FSMState, queue_size=1)
        self.parking_finished_trigger = rospy.Subscriber('~parking_finished', BoolStamped, self.parking_finished_reset,
                                                         queue_size=1)

    def set_manual(self):
        if self.get_state != "NORMAL_JOYSTICK_CONTROL":
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            switcher_msg = Bool()
            switcher_msg.data = False
            self.switcher_pub.publish(switcher_msg)

    def set_autonomous(self):
        if self.get_state != "LANE_FOLLOWING":
            switcher_msg = Bool()
            switcher_msg.data = True
            self.switcher_pub.publish(switcher_msg)
            self.set_state("LANE_FOLLOWING")

    def parking_finished_reset(self, msg):
        self.parking_finished = msg.data

    def switcher(self, tag_msg):
        print(tag_msg.tag_id)
        if not self.parking_state and not self.parking_finished and tag_msg.tag_id == self.params[
            'parking_trigger_apriltag'].value:
            print(f"self.slow {self.slow}")
            if self.relative_placing_of_AT(tag_msg,
                                            (self.params["cent_x"].value, self.params["cent_y"].value),
                                            self.params["size"].value,
                                            (self.params["cent_x_d"].value, self.params["cent_y_d"].value),
                                            self.params["size_d"].value)[2] == 0:
                self.set_manual()
                self.send_LED_request("GREEN")
                self.set_parking.publish(1)
                # print(f"Start parking request sent, parking_state {self.parking_state}, parking_finished {self.parking_finished}")
                self.parking_state = True
            else:
                self.send_LED_request("CYAN")
                self.changeVBarPub.publish(0.4)
                self.slow = True
                print("slowed down")
        if self.parking_state and self.parking_finished and tag_msg.tag_id == self.params[
            'parking_trigger_apriltag'].value and self.relative_placing_of_AT(tag_msg,
                                            (self.params["cent_x"].value, self.params["cent_y"].value),
                                            self.params["size"].value,
                                            (self.params["cent_x_d"].value, self.params["cent_y_d"].value),
                                            self.params["size_d"].value)[2] == 0:
            self.parking_state = False
            self.set_parking.publish(0)
            # print(f"Stop parking request sent, parking_state {self.parking_state}, parking_finished {self.parking_finished}")
            self.send_LED_request("WHITE")
            self.set_autonomous()

            # message = WheelsCmdStamped(vel_left=0.7,
            #                            vel_right=0.7)
            # self.wheel_publisher.publish(message)
        print(f"{self.parking_state} and {self.parking_finished} and {tag_msg.tag_id}")

    def send_LED_request(self, color="RED"):
        pattern = String()
        pattern.data = color
        self.change_pattern.publish(pattern)
        # pattern = ChangePattern()
        # pattern.pattern_name = color
        # self.changePattern(pattern)

    def relative_placing_of_AT(self, apTagDetection, center, size, center_delta=(10, 20), size_delta=5):
        """Ansewr is in format [int, int, int], where
        first  number means x position on the picture,
        second means y position,
        and third means size
            - if more than delta: 1,
            - if less: -1,
            - if in the delta: 0
        """
        answer = [0, 0, 0]
        center_x, center_y = apTagDetection.center
        (corner_0_x, corner_0_y, corner_1_x, corner_1_y,
         corner_2_x, corner_2_y, corner_3_x, corner_3_y) = apTagDetection.corners
        if abs(center_x - center[0]) > center_delta[0]:
            answer[0] += np.sign(center_x - center[0])
        if abs(center_y - center[1]) > center_delta[1]:
            answer[1] += np.sign(center_y - center[1])
        size_of_AT = (abs(corner_0_x - corner_2_x) + abs(corner_0_y - corner_2_y) + abs(corner_1_y - corner_3_y) + abs(
            corner_1_x - corner_3_x)) / 4
        if abs(size - size_of_AT) > size_delta and size_of_AT < size:
            answer[2] += np.sign(size_of_AT - size)
        print(f"Tag is: {answer}")
        return answer

    def _findAprilTags(self, image):
        '''
        Gets the image in the RGB format, converts to grayscale, and detects all apriltags
        :param image: cv2 rgb format imageparking_state {self.parking_state}, parking_finished {self.parking_finished}")
        :return: the list of the detections.
        The detections are in format
        {tag_id: int,
        corners: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        }
        '''
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def process_image(self, image_msg):
        rospy.sleep(0.1)
        img = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        tag_list = self._findAprilTags(img)
        detections_list = []
        # print(f"parking_state {self.parking_state}, parking_finished {self.parking_finished}")

        for tag in tag_list:
            tag_msg = AprilTagDetection()
            corners = tag.corners
            tag_msg.tag_id = tag.tag_id


            # elif  tag_msg.tag_id == 30:
            #     self.send_LED_request("PURPLE")
            # elif tag_msg.tag_id == 8:
            #     self.send_LED_request("OBSTACLE_07")

            tag_msg.hamming = self._type_dict[str(tag_msg.tag_id)] if str(tag_msg.tag_id) in self._type_dict else 0
            tag_msg.center = [(corners[2][0] + corners[0][0]) / 2, (corners[0][1] + corners[2][1]) / 2]
            tag_msg.corners = [corners[3][0], corners[3][1], corners[0][0], corners[0][1], corners[1][0], corners[1][1],
                               corners[2][0], corners[2][1]]


            # print(tag.tag_id)
            # print(tag_msg)
            if self.parking_state == False and tag_msg.tag_id == 5 and self.parking_finished == True:
                self.send_LED_request("BLUE")
                print(f"Ready to park, parking_state {self.parking_state}, parking_finished {self.parking_finished}")
                self.parking_finished = False
                self.slow = True
                print(f"self.slow {self.slow}")
            self.switcher(tag_msg)

            detections_list.append(tag_msg)

        tags_message = AprilTagDetectionArray()
        tags_message.header = image_msg.header
        tags_message.detections = detections_list
        # if self.get_state != "NORMAL_JOYSTICK_CONTROL":
        #     return
        self.tags_message.publish(tags_message)


if __name__ == "__main__":
    node = AprilTagDetector()
    rospy.spin()
