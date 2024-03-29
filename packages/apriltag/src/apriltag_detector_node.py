#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import apriltag
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion
from duckietown_msgs.msg import BoolStamped, AprilTagDetection, AprilTagDetectionArray
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String


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


        # Publisher

        self.tags_message = rospy.Publisher( '~tags_array', AprilTagDetectionArray, queue_size=1)
        self.change_pattern = rospy.Publisher('~change_pattern', String)
        # self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)


        #Subscriber
        self._img_sub = rospy.Subscriber( "~image", CompressedImage, self.process_image, queue_size=1, buff_size="20MB")



    def send_LED_request(self, color = "RED"):
        pattern = String()
        pattern.data = color
        self.change_pattern.publish(pattern)
        # pattern = ChangePattern()
        # pattern.pattern_name = color
        # self.changePattern(pattern)

    def _findAprilTags(self, image):
        '''
        Gets the image in the RGB format, converts to grayscale, and detects all apriltags
        :param image: cv2 rgb format image
        :return: the list of the detections.
        The detections are in format
        {tag_id: int,
        corners: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        }
        '''
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def process_image(self, image_msg):
        img = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        tag_list = self._findAprilTags(img)
        detections_list = []


        for tag in tag_list:
            tag_msg = AprilTagDetection()
            corners = tag.corners

            tag_msg.tag_id = tag.tag_id


            if tag_msg.tag_id == 5:
                self.send_LED_request("OBSTACLE_4")
            elif  tag_msg.tag_id == 67:
                self.send_LED_request("OBSTACLE_2")
            elif tag_msg.tag_id == 8:
                self.send_LED_request("OBSTACLE_07")


            tag_msg.hamming = self._type_dict[str(tag_msg.tag_id)] if str(tag_msg.tag_id) in self._type_dict else 0
            tag_msg.center = [(corners[0][0] + corners[1][0])/2, (corners[0][1] + corners[2][1])/2]
            tag_msg.corners = [corners[0][0], corners[0][1], corners[1][0], corners[1][1], corners[2][0], corners[2][1], corners[3][0], corners[3][1]]

            # print(tag.tag_id)
            # print(tag_msg)

            detections_list.append(tag_msg)

        if tag_list:
            self.tag_queue.pop()
            self.tag_queue.append(True)
            self.queue = True
        else:
            self.tag_queue.pop()
            self.tag_queue.append(False)
            if not any(self.tag_queue) and  self.queue:
                self.send_LED_request("light_off")
                self.queue = False


        tags_message = AprilTagDetectionArray()
        tags_message.header = image_msg.header
        tags_message.detections = detections_list
        self.tags_message.publish(tags_message)


if __name__ == "__main__":
    node = AprilTagDetector()
    rospy.spin()
