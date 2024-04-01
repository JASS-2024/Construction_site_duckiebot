#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

class TimerNode(DTROS):
    """
    Node with timer. It publishes to <BOT_NAME>/timer_node/timer topic message with time stamp every ~interval seconds

    :parameter
        ~interval: defines period of sending messages to <BOT_NAME>/timer_node/timer topic. Can be updated in runtime
    """
    def __init__(self, node_name):
        super(TimerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.interval = DTParam("~interval", param_type=ParamType.FLOAT, min_value=0, max_value=1)

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
            rospy.sleep(self.interval.value)


if __name__ == "__main__":
    timer_node = TimerNode(node_name="timer_node")
    timer_node.timer()
    rospy.spin()
