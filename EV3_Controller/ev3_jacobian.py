import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

class EV3Jacobian(Node):
    def __init__(self):
        super().__init__('ev3_jacobian')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cmd_vel',  # ROS 2 topic
            self.velocity_callback,
            10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

    def velocity_callback(self, msg):
        r = 2.4 # radius of the wheel in cm
        d = 14 # distance from the centre of the bot in cm
        velocity = np.array(msg.data).T
        print(velocity)
        wheel_velocities_msg = Float32MultiArray()
        jacobian = 1/r*np.array([[-d,1,0],
                                [-d,-0.5,math.sin(math.pi/3)],
                                [-d,-0.5,math.sin(math.pi/3)]])
        wheel_velocities = jacobian @ velocity
        print(wheel_velocities)
        wheel_velocities_msg.data = list(wheel_velocities.T)
        self.publisher.publish(wheel_velocities_msg)

def main(args=None):
    rclpy.init(args=args)
    mocap = EV3Jacobian()
    rclpy.spin(mocap)
    mocap.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
