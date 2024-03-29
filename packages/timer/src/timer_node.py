#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern
import time

class TimerNode(DTROS):
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
        super(TimerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.interval = DTParam("~interval", param_type=ParamType.FLOAT, min_value=0, max_value=1)

        #TODO: publisher

        # Construct publishers
        self.timer_pub = rospy.Publisher(
            "~timer", BoolStamped, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.log("Initialized!")

    def timer(self):
        while True:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.timer_pub.publish(msg)
            time.sleep(self.interval.value)


if __name__ == "__main__":
    # Initialize the node
    timer_node = TimerNode(node_name="timer_node")
    timer_node.timer()
    # Keep it spinning
    rospy.spin()
