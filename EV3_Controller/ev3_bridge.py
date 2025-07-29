import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import json
 
class EV3Bridge(Node):
    def __init__(self):
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
        command = msg.data
        json_command = json.dumps(list(command)) # MQTT server handles only strings, so here we convert to json, and in the ev3 receiver, it is converted back to list from json
        self.get_logger().info(f"Sending command to EV3: {json_command}")
        self.mqtt_client.publish("ev3/motor_commands", json_command)

def main(args=None):
    rclpy.init(args=args)
    bridge = EV3Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
