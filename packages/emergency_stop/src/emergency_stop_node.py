#!/usr/bin/env python3
import os
import time

import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from sensor_msgs.msg import Range



class EmergencyStopNode(DTROS):


    def __init__(self, node_name):
        super(EmergencyStopNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        # Initialize params
        self.params = dict()

        self.params["distance"] = DTParam("~distance", param_type=ParamType.FLOAT, min_value=0, max_value=10)

        # Open subscription for type of movement
        self.check_distance = rospy.Subscriber(
            '~range',
            Range,
            self.check_distance, queue_size=1
        )

        


        self.log("Initialized!")


    def check_distance(self, msg):
        print("1000")



if __name__ == "__main__":
    # Initialize the node
    emergence_stop_node = EmergencyStopNode(node_name="emergency_stop_node")
    # Keep it spinning
    rospy.spin()
