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
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String


class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION
        )
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.start_detect = False
        self.bridge = CvBridge()
        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )
        self.marker_id_pub = rospy.Publisher(
            '~tags_id', Int32MultiArray, queue_size=1
        )

        self.improved_image_pub = rospy.Publisher(
            '~debug/improved_image/compressed', CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.change_pattern = rospy.Publisher(
            '~change_pattern', String)

        self.improved_image_pub_debug = rospy.Publisher(
            '~debug/improved_image_debug/compressed', CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.stop_sub = rospy.Subscriber(
            '~start_detection', BoolStamped, self.change_stop_val, queue_size=1
        )

        self.start_sub = rospy.Subscriber(
            '~stop_detection', Bool, self.change_start_val, queue_size=1
        )

        self.switcher_sub = rospy.Subscriber(
            '~switcher', Bool, self.update_switcher, queue_size=1
        )


        self.log('apriltag_init')
        self.switcher = True

    def update_switcher(self, msg):
        self.switcher = True

    def change_start_val(self, msg):
        self.log("stop detection")
        self.start_detect = False

    def change_stop_val(self, msg):
        if self.switcher:
            self.start_detect = True
            self.switcher = False

    def _findAprilTags(self, image):
        '''
        Gets the image in the RGB format, converts to grayscale, and detecta all apriltags
        :param image: cv2 rgb format image
        :return: the list of the detections.
        The detections are in format
        {tag_id: int,
        corners: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        }
        '''
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def cb_image(self, msg):
        # if self.start_detect:
        if True:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            markers = self._findAprilTags(img)
            # print(markers[0])
            marker_id = [i.tag_id for i in markers]
            # self.log(f'detected marker from apriltag {marker_id}')
            self.imagemsg_removed_apriltag(img, markers, msg)
            if len(marker_id) != 0:
                self.log(f'detected marker from apriltag {marker_id}')
                marker_msg = Int32MultiArray(data=marker_id)
                self.marker_id_pub.publish(marker_msg)
                # self.start_detect = False

    def imagemsg_removed_apriltag(self, image, detections, msg_image):
        '''
        :param image:
        :param detection:
        :param msg_header:
        :return:
        '''
        if len(detections) == 0:
            if self.improved_image_pub.get_num_connections() > 0:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
                image_msg.header = msg_image.header
                self.improved_image_pub.publish(image_msg)

        else:
            pattern = String()
            pattern.data = "RED"
            self.change_pattern.publish(pattern)
            print("Trying to change the pattern!!!!")


            resulted_image = image.copy()
            marker = detections[0]
            # print(marker)
            cv2.fillPoly(resulted_image, [marker.corners.astype(np.int32)], (0, 0, 0))
            image_msg = self.bridge.cv2_to_compressed_imgmsg(resulted_image)
            image_msg.header = msg_image.header
            try:
                self.improved_image_pub.publish(image_msg)
            except:
                self.improved_image_pub_debug.publish(msg_image)


if __name__ == "__main__":
    node = AprilTagDetector()
    # spin forever
    rospy.spin()
