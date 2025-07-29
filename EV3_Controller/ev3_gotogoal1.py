import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import signal
import sys
import math

class EV3GotogoalNode(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('ev3_gotogoal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.pursuer_pose_subscriber = self.create_subscription(PoseStamped, 'pursuer/pose', self.pursuer_callback, 10)
        self.yaw = None
        self.x = None
        self.y = None
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_timer(0.1, self.gotogoal)


    def pursuer_callback(self, q):
        q_w, q_x, q_y, q_z = q.pose.orientation.x, q.pose.orientation.y, q.pose.orientation.z, q.pose.orientation.w
        self.x = q.pose.position.x
        self.y = q.pose.position.y
        # Calculate yaw, pitch, roll
        self.yaw = math.atan2(2 * (q_y * q_z + q_w * q_x), q_w**2 - q_x**2 - q_y**2 + q_z**2)   

    def gotogoal(self):

        if self.x is not None and abs(self.goal_x-self.x) != 0 and abs(self.yaw-math.atan2((self.goal_y-self.y),(self.goal_x-self.x))) > 0.5:
            msg = Float32MultiArray()
            print(self.yaw)
            msg.data = [50,50,50] # turn anticlockwise
            self.publisher.publish(msg)
        elif self.x is not None and math.sqrt((self.x-self.goal_x)**2 + (self.y-self.goal_y)**2) > 0.5:
            msg = Float32MultiArray()
            msg.data = [-50,50,0] # move front
            self.publisher.publish(msg)
        else:
            msg = Float32MultiArray()
            msg.data = [0,0,0] # stop at goal
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    goal_x = 0.1
    goal_y = 0.00836120080202818
    gotogoal = EV3GotogoalNode(goal_x=goal_x, goal_y=goal_y)
    rclpy.spin(gotogoal)
    gotogoal.destroy_node()
    rclpy.shutdown()

def custom_handler(sig, frame):
    node = Node("simple_publisher")  # Create a temporary node
    pub = node.create_publisher(Float32MultiArray, 'motor_commands', 10)
    msg = Float32MultiArray()
    msg.data = [0,0,0] # turn anticlockwise
    pub.publish(msg)
    sys.exit(0)  # Exit gracefully

# Register the signal handler
signal.signal(signal.SIGINT, custom_handler)


if __name__ == "__main__":
    main()
    
