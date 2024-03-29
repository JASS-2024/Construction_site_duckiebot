#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

class ObstacleDetectorNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObstacleDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params["h_sat"] = DTParam("~h_sat", param_type=ParamType.INT, min_value=0, max_value=180)
        self.params["l_sat"] = DTParam("~l_sat", param_type=ParamType.INT, min_value=0, max_value=255)
        self.params["h_hue"] = DTParam("~h_hue", param_type=ParamType.INT, min_value=0, max_value=255)
        self.params["l_hue"] = DTParam("~l_hue", param_type=ParamType.INT, min_value=0, max_value=180)
        self.params["h_val"] = DTParam("~h_val", param_type=ParamType.INT, min_value=0, max_value=255)
        self.params["l_val"] = DTParam("~l_val", param_type=ParamType.INT, min_value=0, max_value=255)
        self.params['erode_kernel_size'] = DTParam("~erode_kernel_size", param_type=ParamType.INT, min_value=0, max_value=16)
        self.params['mean_pix_val'] = DTParam("~mean_pix_val", param_type=ParamType.FLOAT, min_value=0, max_value=1)
        self.params['cutoff'] = DTParam("~cutoff", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        # Initialize variables
        self.bridge = CvBridge()

        #TODO: publisher

        # Construct publishers
        self.pub_obstacle_image = rospy.Publisher(
            "~debug/segmentation/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.pub_stop = rospy.Publisher(
            "~stop_request", BoolStamped, queue_size=1, dt_topic_type=TopicType.PERCEPTION
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

        # K = np.array([[700, 0, image.shape[1] / 2],
        #               [0, 700, image.shape[0] / 2],
        #               [0, 0, 1]])
        #
        # D = np.array([0, 0, 0, 0])
        #
        # # Выпрямление рыбьего глаза
        # image = cv2.fisheye.undistortImage(image, K, D)

        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # HSV diapason for target color (for now it is green)
        upper_green = np.array([self.params['h_hue'].value, self.params['h_sat'].value, self.params['h_val'].value])  # Higher threshold of the target color
        lower_green = np.array([self.params['l_hue'].value, self.params['l_sat'].value, self.params['l_val'].value])  # Lower threshold of the target color

        # The mask for target color
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        kernel = np.ones((self.params['erode_kernel_size'].value, self.params['erode_kernel_size'].value), np.uint8)
        eroded_mask = cv2.erode(mask, kernel, iterations=1)



        # Applying the mask to original image
        target_areas = cv2.bitwise_and(image, image, mask=eroded_mask)

        msg = BoolStamped()
        msg.header.stamp = image_msg.header.stamp

        if (target_areas[int(target_areas.shape[0] * (1 - self.params['cutoff'].value)):, :, :].mean() > self.params['mean_pix_val'].value):
            msg.data = True
        else:
            msg.data = False
        self.pub_stop.publish(msg)



        #Publishing the masked image
        if self.pub_obstacle_image.get_num_connections() > 0:
            target_areas_msg = self.bridge.cv2_to_compressed_imgmsg(target_areas)
            target_areas_msg.header = image_msg.header
            self.pub_obstacle_image.publish(target_areas_msg)




if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = ObstacleDetectorNode(node_name="obstacle_detection_node")
    # Keep it spinning
    rospy.spin()
