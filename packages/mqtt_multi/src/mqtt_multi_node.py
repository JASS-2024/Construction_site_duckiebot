#!/usr/bin/env python3
import json
import os
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from paho.mqtt.enums import CallbackAPIVersion


class MQTTMultiNode(DTROS):
    def __init__(self, node_name):
        super(MQTTMultiNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']
        temp_for_mapping_optitrack_and_regular_bot_name = "~" + self.vehicle_name

        print("temp_for_mapping_optitrack_and_regular_bot_name " + temp_for_mapping_optitrack_and_regular_bot_name)

        # Adapting vehicle name for optitrack naming
        vehicle_name_for_topic = DTParam(temp_for_mapping_optitrack_and_regular_bot_name, param_type=ParamType.STRING)
        vehicle_name_for_topic_with_tilda = "~" + vehicle_name_for_topic.value
        self.vehicle_name_optitrack = vehicle_name_for_topic.value

        self.mqtt_broker_host = DTParam("~mqtt_broker_host", param_type=ParamType.STRING)
        print(self.mqtt_broker_host.value)
        self.license_plate = DTParam(vehicle_name_for_topic_with_tilda, param_type=ParamType.STRING)
        print("Vehicle_name_for_topic_with_tilda " + vehicle_name_for_topic_with_tilda)
        print("Mapped license plate " + self.license_plate.value)
        self.mqtt_broker_port = DTParam("~mqtt_broker_port", param_type=ParamType.FLOAT)

        self.client = mqtt.Client(callback_api_version=CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        # Connecting to MQTT Broker
        self.client.connect(self.mqtt_broker_host.value, self.mqtt_broker_port.value, 60)
        self.client.loop_start()

        # Initializing Publisher and Subscribers
        self.booked_slot_pub = rospy.Publisher("~booked_slot", String, queue_size=1)
        self.is_parked_sub = rospy.Subscriber("~parking_node/parking_finished", BoolStamped, self.on_park_trigger)

        # Saving place id that we get from MQTT garage/knockknock topic
        self.place_id = None




    def adapt_name_for_optitrack(self, name):
        basename = "vehicle/*/status"
        computed_topic_name = basename.replace('*', name)
        print(f"New topic name for {name} is {computed_topic_name}")
        return computed_topic_name

    def on_disconnect(client, userdata, rc, Properties=None, reason=None):
        if rc != 0:
            print(f"Unexpected disconnection. Reason: {reason}")
        else:
            print("Disconnected successfully.")

    def on_connect(self, client, userdata, connect_flags, rc, properties, ):
        print(f"Connected with result code {rc}")
        # Subscribing to required mqtt topics
        self.client.subscribe("garage/knockknock")

    def on_message(self, client, userdata, msg):
        if msg.topic == "garage/knockknock":
            try:
                print("Yes, it was garage/knockknock")
                message = json.loads(msg.payload.decode("utf-8"))
                print("Message.get('license_plate') " + message.get('license_plate'))
                print("self.license_plate " + self.license_plate.value)
                if message.get('license_plate') == self.license_plate.value:
                    self.place_id = message.get('place_id')
                    # Publishing received place_id to the ROS topic
                    self.booked_slot_pub.publish(self.place_id)
                    print(f"Published! For this bot place_id is {self.place_id}")
            except json.JSONDecodeError as e:
                rospy.logerr(f"Error decoding JSON: {e}")

    def on_park_trigger(self, msg):
        print(self.place_id + self.license_plate.value)
        if self.place_id and self.license_plate.value:
            print('Mqtt: TRIGGERED ON PARK')
            message = {"name": self.vehicle_name_optitrack, "place_id": self.place_id, "license_plate": self.license_plate.value}
            self.client.publish("isParked", json.dumps(message))



if __name__ == "__main__":
    # Create the MQTTMultiNode object
    mqtt_multi_node = MQTTMultiNode(node_name="mqtt_multi")
    # Keep it spinning to keep the node alive
    rospy.spin()