import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import keyboard

class EV3ControlNode(Node):
    def __init__(self):
        super().__init__('ev3_remote')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_timer(0.1, self.publish_key)

    def publish_key(self):
        try:
            event = keyboard.read_event(suppress=True)  # Read keyboard event
            if event.event_type == keyboard.KEY_DOWN:  # Check if the event is a key press
                key = event.name
                msg = Float32MultiArray()
                if key == 'up':
                    msg.data = [-50,50,0]
                elif key == "down":
                    msg.data = [50,-50,0]
                elif key == "left":
                    msg.data = [50,50,50]
                elif key == "right":
                    msg.data = [-50, -50, -50]
                else:
                    msg.data = [0,0,0]
                self.publisher.publish(msg)
                self.get_logger().info(f"Key pressed: {key}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    remote = EV3ControlNode()
    rclpy.spin(remote)
    remote.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()