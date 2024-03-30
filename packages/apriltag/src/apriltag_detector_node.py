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
from std_msgs.msg import Int32MultiArray, Int32
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
        self.col_change = True


        # Publisher

        self.tags_message = rospy.Publisher( '~tags_array', AprilTagDetectionArray, queue_size=1)
        self.change_pattern = rospy.Publisher('~change_pattern', String)
        self.start_parking = rospy.Publisher('~fsm_signal', Int32, queue_size=1)
        # self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)

        #debug_values
        self.params = dict()
        self.params["cent_x"] = DTParam("~cent_x", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_y"] = DTParam("~cent_y", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_x_d"] = DTParam("~cent_x_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["cent_y_d"] = DTParam("~cent_y_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["size"] = DTParam("~size", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["size_d"] = DTParam("~size_d", param_type=ParamType.FLOAT, min_value=0, max_value=1000)

        #Subscriber
        self._img_sub = rospy.Subscriber( "~image", CompressedImage, self.process_image, queue_size=1, buff_size="20MB")


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
        center_x,  center_y = apTagDetection.center
        (corner_0_x, corner_0_y, corner_1_x, corner_1_y,
         corner_2_x, corner_2_y, corner_3_x, corner_3_y)= apTagDetection.corners
        if abs(center_x - center[0]) > center_delta[0]:
            answer[0]+= np.sign(center_x - center[0])
        if abs(center_y - center[1]) > center_delta[1]:
            answer[1]+= np.sign(center_y - center[1])
        size_of_AT = (abs(corner_0_x- corner_2_x)+abs(corner_0_y- corner_2_y)+abs(corner_1_y- corner_3_y)+abs(corner_1_x- corner_3_x))/4
        if abs(size - size_of_AT) > size_delta:
            answer[2]+= np.sign(size - size_of_AT)
        print(f"Tag is: {answer}")
        return answer


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
            print(f"found {tag_msg.tag_id}")
            tag_msg.hamming = self._type_dict[str(tag_msg.tag_id)] if str(tag_msg.tag_id) in self._type_dict else 0
            tag_msg.center = [(corners[2][0] + corners[0][0])/2, (corners[0][1] + corners[2][1])/2]
            tag_msg.corners = [corners[3][0], corners[3][1], corners[0][0], corners[0][1], corners[1][0], corners[1][1], corners[2][0], corners[2][1]]

            detections_list.append(tag_msg)

            if tag_msg.tag_id == 239:

                place = self.relative_placing_of_AT(tag_msg, (self.params["cent_x"].value, self.params["cent_y"].value), self.params["size"].value
                                                    ,(self.params["cent_x_d"].value, self.params["cent_y_d"].value),self.params["size_d"].value)
                if place[1] == 0 and place[2] == 0 and place[0] > -1:
                    self.send_LED_request("RED")
                    self.start_parking.publish(1)
                    self.col_change = False

                elif self.col_change: self.send_LED_request("YELLOW")

        # if tag_list:
        #     self.tag_queue.pop()
        #     self.tag_queue.append(True)
        #     self.queue = True
        # else:
        #     self.tag_queue.pop()
        #     self.tag_queue.append(False)
        #     if not any(self.tag_queue) and  self.queue:
        #         self.send_LED_request("light_off")
        #         self.queue = False


        tags_message = AprilTagDetectionArray()
        tags_message.header = image_msg.header
        tags_message.detections = detections_list
        self.tags_message.publish(tags_message)


if __name__ == "__main__":
    node = AprilTagDetector()
    rospy.spin()
