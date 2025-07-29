import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import signal
import sys
import math
import numpy as np
import gogoal

class EV3GotogoalNode(Node):

    def __init__(self, goal_x, goal_y):
        super().__init__('ev3_gotogoal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        
        # Initialize all pose variables
        self.x = None
        self.y = None
        self.yaw = None
        self.pose_received = False
        
        self.robot = None
        
        self.pursuer_pose_subscriber = self.create_subscription(
            PoseStamped, 'pursuer0/pose', self.pursuer_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        
        self.val = [0.0, 0.0, 0.0]
        
        # Timer for publishing commands (reduced frequency for stability)
        self.create_timer(0.1, self.gotogoal)  # 10 Hz instead of 100 Hz
        
        self.get_logger().info(f'EV3 Goto Goal Node initialized. Target: ({goal_x}, {goal_y})')

    def pursuer_callback(self, msg):
        """Callback function to handle pose updates"""
        try:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            
            q_x = msg.pose.orientation.x
            q_y = msg.pose.orientation.y
            q_z = msg.pose.orientation.z
            q_w = msg.pose.orientation.w
            
            # Convert quaternion to yaw angle
            self.yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 
                                 1 - 2 * (q_y**2 + q_z**2))
            print(self.yaw)
            self.pose_received = True
            
            # Calculate distance to goal
            distance_to_goal = np.linalg.norm(np.array([self.x - self.goal_x, self.y - self.goal_y]))
            
            if distance_to_goal > 0.05:  # If not at goal
                if self.robot is None:
                    self.robot = gogoal.OmniRobot(np.array([self.x, self.y]), self.yaw)
                else:
                    # Update robot state
                    self.robot.position = np.array([self.x, self.y])
                    self.robot.orientation = self.yaw
                
                # Get motor commands
                self.val = self.robot.move_to_target(
                    np.array([self.goal_x, self.goal_y]), 
                    max_speed=0.5,  # Reduced speed for stability
                    dt=0.1
                )
                
                # Ensure val is a list of floats
                if isinstance(self.val, np.ndarray):
                    self.val = self.val.tolist()
                self.val = [float(v) for v in self.val]
                
                self.get_logger().info(
                    f'Position: ({self.x:.3f}, {self.y:.3f}), '
                    f'Distance to goal: {distance_to_goal:.3f}, '
                    f'Motor commands: {self.val}'
                )
            else:
                # At goal - stop motors
                self.val = [0.0, 0.0, 0.0]
                self.get_logger().info('Goal reached! Stopping motors.')
                
        except Exception as e:
            self.get_logger().error(f'Error in pursuer_callback: {str(e)}')
            self.val = [0.0, 0.0, 0.0]

    def gotogoal(self):
        """Timer callback to publish motor commands"""
        if self.pose_received:
            try:
                msg = Float32MultiArray()
                msg.data = self.val
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing motor commands: {str(e)}')
        else:
            # No pose received yet - publish zero commands
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0]
            self.publisher.publish(msg)

    def shutdown_handler(self):
        """Clean shutdown - stop all motors"""
        self.get_logger().info('Shutting down - stopping all motors')
        try:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0]
            self.publisher.publish(msg)
            # Give time for message to be sent
            rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {str(e)}')

# Global reference to node for signal handler
global_node = None

# def custom_handler(sig, frame):
#     """Signal handler for clean shutdown"""
#     global global_node
#     print('\nReceived interrupt signal. Shutting down...')
    
#     if global_node is not None:
#         global_node.shutdown_handler()
    
#     # Alternative method to stop motors if node method fails
#     try:
#         rclpy.init()
#         temp_node = Node("emergency_stop")
#         pub = temp_node.create_publisher(Float32MultiArray, 'motor_commands', 10)
#         msg = Float32MultiArray()
#         msg.data = [0.0, 0.0, 0.0]
#         pub.publish(msg)
#         rclpy.spin_once(temp_node, timeout_sec=0.1)
#         temp_node.destroy_node()
#         rclpy.shutdown()
#     except:
#         pass
    
#     sys.exit(0)

def main(args=None):
    global global_node
    
    try:
        rclpy.init(args=args)
        
        # Set goal coordinates
        goal_x = 1.0
        goal_y = 1.0
        
        # Create node
        global_node = EV3GotogoalNode(goal_x=goal_x, goal_y=goal_y)
        
        # Set up signal handler
        signal.signal(signal.SIGINT, custom_handler)
        
        # Spin the node
        rclpy.spin(global_node)
        
    except KeyboardInterrupt:
        print('Interrupted by user')
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Clean shutdown
        if global_node is not None:
            global_node.shutdown_handler()
            global_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
