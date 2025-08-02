import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import keyboard

class EV3ControlNode(Node):
    """
    ROS2 Node for remotely controlling EV3 motors using keyboard input.
    
    Publishes Float32MultiArray messages to the 'motor_commands' topic based on arrow key inputs.
    """
    def __init__(self):
        """
        Initialize the EV3ControlNode.

        Sets up the publisher to send motor commands and starts a periodic timer
        to listen for keyboard events.
        """
        super().__init__('ev3_remote')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_timer(0.1, self.publish_key)

    def publish_key(self):
        """
        Read the current keyboard event and publish motor command messages.

        Maps arrow key presses to specific motor command arrays and publishes them.
        Other keys send a stop command [0, 0, 0].
        """
        try:
            event = keyboard.read_event(suppress=True) # Read keyboard event
            if event.event_type == keyboard.KEY_DOWN: # Check if the event is a key press
                key = event.name
                msg = Float32MultiArray()

                # Assign motor commands based on key
                if key == 'up':
                    msg.data = [-50, 50, 0]
                elif key == "down":
                    msg.data = [50, -50, 0]
                elif key == "left":
                    msg.data = [50, 50, 50]
                elif key == "right":
                    msg.data = [-50, -50, -50]
                else:
                    msg.data = [0, 0, 0]

                self.publisher.publish(msg)
                self.get_logger().info(f"Key pressed: {key}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    """
    Entry point for the EV3 remote control node.

    Initializes the ROS2 environment, creates an EV3ControlNode instance,
    and spins the node to process callbacks.
    """
    rclpy.init(args=args)
    remote = EV3ControlNode()
    rclpy.spin(remote)
    remote.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
