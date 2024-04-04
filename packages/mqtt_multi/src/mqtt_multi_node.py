#!/usr/bin/env python4
import time
import rospy



from paho-mqtt import mqtt
from paho.mqtt.enums import CallBackAPIVersion


from duckietown.dtros import DTROS, TopicType, NodeType


class MQTTMultiNode(DTROS):
    """MQTT Node that is responsible of communicating with mqtt_broker. 

    Events:
        ~ Received booked slot: on this event this node publishes this booked 
        slot to the ROS topic
        
        ~ Received isParked: on this event this node publishes message with 
        bot's name to the isParked mqtt topic
        
        ~ Received: on this event this node publishes this booked slot to the ROS topic

    Configuration:
        ~ mqtt_broker_host (:obj:`str`): address of mqtt broker  

        ~ mqtt_broker_port (:obj:`str`): address of mqtt broker  
    
    Publishers:
        ~isParked (:obj:`{"name": "_"}`): publisher for publishing the success 
        message once bot is parked

    Subscribers:
        ~isParked_node (:obj:`BoolStamped`): listens to the parking node 
        and once it receive data it triggers node to publish to the mqtt 

    """

    # TODO: 'Remove all unneccessary prints and logs from the console!' 


    # TODO: 'Subscribe to the parking_node topic to get data and trigger on the events' 

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(MQTTMultiNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Something useful, idk, just copy paste
        self.robot_type = rospy.get_param("~robot_type")

        # Getting the real bot's name 
        self.vehicle_name = os.environ['VEHICLE_NAME']
 

        # Getting adjusted name for OptiTrack from yaml file
        vehicle_name_for_topic = DTParam(self.vehicle_name, param_type=ParamType.STRING)
 
        # Getting broker's params 
        self.mqtt_broker_host = DTParam("~mqtt_broker_host", param_type=ParamType.STRING)
        self.mqtt_broker_port = DTParam("~mqtt_broker_port", param_type=ParamType.INT)
        self.mqtt_broker_timeout = DTParam("~mqtt_broker_timeout", param_type=ParamType.INT)
        
        # Formatted message for isParked mqtt topic
        isParked_msg_snippet = {
            "name": vehicle_name_for_topic.value, # ATTENTION: May be without value,
        } 

        # Defining variable + fallback
        booked_slot_name = "for some reason wasn't updated" # Fallback, just for debug

        # Binding custom methods to mqtt client
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        
        # Creating instance of mqtt client
        client = mqtt.Client(callback_api_version=CallBackAPIVersion.VERSION2)
        

        # Trying to connect
        self.establish_connection()

        # Publishers
        self.booked_slot = rospy.Publisher("~booked_slot", String, on_get_parking_slot, queue_size=1)


        # Subscribers
        self.is_parked_sub = rospy.Subscriber("~parking_node/parking_finished", BoolStamped, 
            on_park_trigger, queue_size=1)
        
        
        # Staring mqtt client loop
        client.loop_forever()


    def on_connect(client, userdata, flags, rc, properties=None):
        """ Default required method for mqtt client """
        print(f"Connected with result code {rc}")
        client.subscribe(compute_topics_name(vehicle_name_for_topic))

        
    def compute_topics_name(self, name):
        """ Adds robot's name to the topic template and return a real mqtt topic name
            Arguments:
                ~name (str): Name of your robot, already compared with yaml config and adjusted for OptiTrack naming

        """
        basename = "vehicle/*/status"
        computed_topic_name = basename.replace('*', name)
        print(f"New topic name for {name} is {computed_topic_name}")
        return computed_topic_name


    def on_get_parking_slot(self, booked_slot_name):
        """ Once it triggered, this function publishes booked_slot_name to the ~booked_slot ROS topic 

        Arguments:
            ~booked_slot_name (str): Name that we get from the mqtt topic with booked slot's name for this robot
        
        """
        self.booked_slot.publish(booked_slot_name)
        print("Successfully published booked slot name")


    def on_park_trigger(self):
        """ Once it's triggered, this function notifies "isParked" mqtt topic """
        self.mqtt_send_message("isParked","Succesfully notified isParked topic in mqtt")


    def mqtt_send_message(mqtt_topic_name, message):
        """ Sends a message to required topic through mqtt. As simple as that.

        Arguments:
            ~message (str): Message that you want to send
            
            ~topic_name (str): Topic's name to which you're sending your message'

        """
        client.publish(mqtt_topic_name, message)



if __name__ == "__main__":
    # Create the MQTTMultiNode object
    mqtt_multi_node = MQTTMultiNode(node_name="mqtt_multi")
    # Keep it spinning to keep the node alive
    rospy.spin()
