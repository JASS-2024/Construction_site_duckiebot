#!/usr/bin/env python3
import numpy as np
import rospy
import json
from paho.mqtt import client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from std_msgs.msg import Float32MultiArray
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType

from datetime import datetime
class MQTTNode(DTROS):
    def __init__(self, node_name):
        """Retrieves data from OptiTrack via MQTT and publishes it in [x, y, theta, timestamp]: Float32MultiArray. Timestamp
          is in nanosecond.

          Configuration:
              ~object_name (:obj:`String`): ID of the robot in OptiTrack system

              ~mqtt_broker_ip (:obj:`String`): Host's IP address

              ~mqtt_topic (:obj:`String`): Name of the required topic in MQTT broker

          Publisher:
              ~telemetry (:obj:`Float32MultiArray`): The data from OptiTrack through MQTT. [x, y, theta, timestamp]
          """

        super(MQTTNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)


        # Getting params from config default.yaml
        self.object_name = DTParam("~object_name", param_type=ParamType.STRING),
        self.mqtt_broker_ip = DTParam("~mqtt_broker_ip", param_type=ParamType.STRING),
        self.mqtt_topic = DTParam("~mqtt_topic", param_type=ParamType.STRING)

        # Adjusting mqtt_topic using our object_name at the '.../*/...'
        self.mqtt_topic = self.mqtt_topic.replace('*', self.object_name)

        # MQTT Client setup
        self.mqtt_client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION1) # ATTENTION: This API version (1) is deprecated. Currentrly using just because it works
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.log(f'Connecting to {self.mqtt_broker_ip}')
        self.mqtt_client.connect(self.mqtt_broker_ip, 1883, 60)
        self.mqtt_client.loop_start()

        # Construct publisher
        self.pub_telemetry = rospy.Publisher("~telemetry", Float32MultiArray, queue_size=1)
        self.log("Initialized!")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.log("Connected to MQTT Broker!")
            self.mqtt_client.subscribe(self.mqtt_topic)
        else:
            self.log(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            coordinates = data['data']['coordinates']
            yaw_in_radians = np.deg2rad(coordinates['yaw'])
            timestamp = data['data']['timestamp']
            nanoseconds = self.getNanoSec(timestamp)
            telemetry_msg = Float32MultiArray()
            telemetry_msg.data = [coordinates['x'], coordinates['y'], yaw_in_radians, timestamp]
            self.pub_telemetry.publish(telemetry_msg)
            print(telemetry_msg.data)
            print(telemetry_msg)
        except Exception as e:
            rospy.logerr(f"Error in on_message: {e}")


    def getNanoSec(self, timestamp_str):
        timestamp_dt = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%f")
        posix_timestamp = timestamp_dt.timestamp()
        ros_time = rospy.Time.from_sec(posix_timestamp)

        nanoseconds = int(timestamp_dt.timestamp() * 1e9)

        return nanoseconds

    # DEBUGGING
    # def on_message(self, client, userdata, msg):
    #     print(f"Message received: {msg.payload.decode()} on topic {msg.topic}")

    def on_shutdown(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super(MQTTNode, self).on_shutdown()


if __name__ == "__main__":
    mqtt_node = MQTTNode(node_name="mqtt_node")
    rospy.spin()
