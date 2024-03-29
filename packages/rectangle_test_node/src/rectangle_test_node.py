#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

class RectangleTestNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(RectangleTestNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params["center_x"] = DTParam("~center_x", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["center_y"] = DTParam("~center_y", param_type=ParamType.FLOAT, min_value=0, max_value=1000)
        self.params["side"] = DTParam("~side", param_type=ParamType.INT, min_value=1, max_value=1000)

        # Initialize variables
        self.bridge = CvBridge()


        # Construct publishers
        self.pub_rectangle= rospy.Publisher(
            "~debug/rectangle_test/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )


        # Construct subscribers
        self.duckie_image = rospy.Subscriber(
            "~duckie_image",  CompressedImage, self.processImageMessage, buff_size=10000000, queue_size=1
        )

        self.log("Initialized!")

    def processImageMessage(self, image_msg):

        #convert raw image file to cv2 image
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr(f"Could not decode image: {e}")
            return

        half_side = self.params["side"].value // 2
        top_left_x = self.params["center_x"].value - half_side
        top_left_y = self.params["center_y"].value - half_side
        bottom_right_x = self.params["center_x"].value + half_side
        bottom_right_y = self.params["center_y"].value + half_side
        cv2.rectangle(image, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 0, 0), -1)


        #Publishing the masked image
        if self.pub_rectangle.get_num_connections() > 0:
            target_areas_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            target_areas_msg.header = image_msg.header
            self.pub_rectangle.publish(target_areas_msg)




if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = RectangleTestNode(node_name="rectangle_test_node")
    # Keep it spinning
    rospy.spin()
