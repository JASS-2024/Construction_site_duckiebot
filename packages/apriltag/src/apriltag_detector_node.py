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
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def cb_image(self, msg):
        if self.start_detect:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            markers = self._findAprilTags(img)
            marker_id = [i.tag_id for i in markers]
            if len(marker_id) != 0:
                self.log(f'detected marker from apriltag {marker_id}')
                marker_msg = Int32MultiArray(data=marker_id)
                self.marker_id_pub.publish(marker_msg)
                self.start_detect = False


if __name__ == "__main__":
    node = AprilTagDetector()
    # spin forever
    rospy.spin()
