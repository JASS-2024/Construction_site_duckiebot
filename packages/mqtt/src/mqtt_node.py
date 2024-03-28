#!/usr/bin/env python3
import numpy as np
import rospy
import cv2

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern

from std_msgs.msg import Float32MultiArray

from paho.mqtt import client as mqtt

import json


# TODO: 'Coordinates for parking slots x1,y1,x2,y2,x3,y3,x4,y4 to share parking slot's corners'

class MQTTNode(DTROS):
    """Retrieves data from OptiTrack via MQTT and publishes it in [x, y, theta, timestamp]: Float32MultiArray. Timestamp
    is in nanosecond.

    Configuration:
        ~object_id (:obj:`Int32`): ID of the robot in OptiTrack system

        ~mqtt_broker_ip (:obj:`String`): Host's IP address

        ~mqtt_topic (:obj:`String`): Name of the required topic in MQTT broker

    Publisher:
        ~telemetry (:obj:`Float32MultiArray`): The data from OptiTrack through MQTT. [x, y, theta, timestamp]
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(MQTTNode, self).__init__(node_name=node_name, node_type=NodeType.DATA_PROCESSING)

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        self.params["object_id"] = DTParam("~object_id", param_type=ParamType.INT32)
        self.params["mqtt_broker_ip"] = DTParam("~mqtt_broker_ip", param_type=ParamType.INT32)
        self.params["mqtt_topic"] = DTParam("~mqtt_topic", param_type=ParamType.INT32)

        # Getting predefined params
        self.mqtt_broker_ip = self.mqtt_broker_ip
        self.mqtt_topic = self.mqtt_topic

        # MQTT Client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_msg
        self.log(f'Connecting to the {self.mqtt_broker_ip}')
        self.mqtt_client.connect(self.mqtt_broker_ip, 1883, 60)
        self.mqtt_client.loop_start()

        # Construct publishers
        self.pub_telemetry_params = rospy.Publisher(
            "~telemetry", Float32MultiArray, queue_size=1, dt_topic_type=TopicType.SERVICE
        )

        self.log("Initialized!")

    # On successful connect
    def on_connect(self, rc):
        print("Connected to OptiTrack MQTT Broker with code " + str(rc))
        self.mqtt_client.subscribe(self.mqtt_topic)


    # Assuming that we receive msg in a 'decodable'format and we're able to decode it to JSON with
    # 'position' key - value for required object

    # TODO: 'Make sure that we receive timestamp in nanoseconds. For this we can use something like rospy.time.now toNanoSec'
    # On receiving message
    def on_msg(self, msg):
        print(f"!!! Received message on topic {msg.topic} !!!")
        try:
            data = json.loads(msg.payload.decode())
            telemetry_msg = Float32MultiArray()
            telemetry_msg.data = [data['position']['x'], data['position']['y'], data['position']['theta'], data['position']['timestamp']]
            self.pub_telemetry_params.publish(telemetry_msg)
        except Exception as e:
            print(f"Something upalo with error: {e}")




if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = MQTTNode(node_name="mqtt_node")
    # Keep it spinning
    rospy.spin()
