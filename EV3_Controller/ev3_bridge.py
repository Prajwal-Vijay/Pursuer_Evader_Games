"""
EV3 Bridge Node

This module defines the EV3Bridge class, which acts as a bridge between ROS 2 and the EV3 bots using MQTT.
It subscribes to motor command messages from a ROS 2 topic and forwards them to the EV3 via MQTT.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import json
 
class EV3Bridge(Node):
    """
    A ROS 2 node that bridges motor command messages to an EV3 robot via MQTT.

    Subscribes to the 'motor_commands' topic and publishes received commands to the EV3 over MQTT.
    
    'motor_commands' topic expects messages of type Float32MultiArray, which must be size 3.
    They represent the percentage of the maximum speed at which each motor should run.
    """
    def __init__(self):
        """
        Initialize the EV3Bridge node, set up ROS 2 subscription and MQTT client.
        """
        super().__init__('ev3_bridge')

        # ROS 2 Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_commands',  # ROS 2 topic
            self.command_callback,
            10
        )

        # MQTT Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set("esb201", "esb@201")  # Set MQTT username and password
        self.mqtt_client.connect("192.168.43.177", 1883, 60)  # Replace with PC's IP (On which MQTT broker is running)
        self.mqtt_client.loop_start()

    def command_callback(self, msg):
        """
        Callback function for ROS 2 subscriber.

        Converts the received motor command to JSON and publishes it to the EV3 via MQTT.

        Args:
            msg (Float32MultiArray): The motor command message from ROS 2.
        """
        command = msg.data
        json_command = json.dumps(list(command)) # MQTT server handles only strings, so here we convert to json, and in the ev3 receiver, it is converted back to list from json
        self.get_logger().info(f"Sending command to EV3: {json_command}")
        self.mqtt_client.publish("ev3/motor_commands", json_command)

def main(args=None):
    """
    Entry point for the EV3Bridge node.

    Initializes ROS 2, creates the EV3Bridge node, and spins it.
    """
    rclpy.init(args=args)
    bridge = EV3Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
