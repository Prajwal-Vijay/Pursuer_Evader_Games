import motioncapture
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math
class EV3_MocapNode(Node):
    def __init__(self):
        super().__init__('ev3_mocap')
        self.pursuer_publisher = self.create_publisher(PoseStamped, "pursuer0/pose", 10)
        self.evader1_publisher = self.create_publisher(PoseStamped, "pursuer1/pose", 10)
        self.evader2_publisher = self.create_publisher(PoseStamped, "evader/pose", 10)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mc = motioncapture.connect('vrpn', {"hostname":"192.168.43.17"})

    def timer_callback(self):
        msg = PoseStamped()
        msg.pose.position = Point()
        msg.pose.orientation = Quaternion()
        self.mc.waitForNextFrame()
        for name, obj in self.mc.rigidBodies.items():
            position = list(obj.position)
            msg.header.frame_id = str(name)
            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])
            msg.pose.orientation.x = obj.rotation.x
            msg.pose.orientation.y = obj.rotation.y
            msg.pose.orientation.z = obj.rotation.z
            msg.pose.orientation.w = obj.rotation.w
            print("\n"+str(name)+"\n"+str(position))
            if str(name) == "pursuer0":
                self.pursuer_publisher.publish(msg)
                self.get_logger().info('Publishing: "{}":"{}" "{}"'.format(msg.header.frame_id, msg.pose.position, msg.pose.orientation))
            elif str(name) == "pursuer1":
                self.evader1_publisher.publish(msg)
                self.get_logger().info('Publishing: "{}":"{}" "{}"'.format(msg.header.frame_id, msg.pose.position, msg.pose.orientation))
            elif str(name) == "evader":
                self.evader2_publisher.publish(msg)
                self.get_logger().info('Publishing: "{}":"{}" "{}"'.format(msg.header.frame_id, msg.pose.position, msg.pose.orientation))

            q_w, q_x, q_y, q_z = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            self.yaw = math.atan2(2 * (q_y * q_z + q_w * q_x), q_w**2 - q_x**2 - q_y**2 + q_z**2)
            print("yaw:",self.yaw)
            
def main(args=None):
    rclpy.init(args=args)
    mocap = EV3_MocapNode()
    rclpy.spin(mocap)
    mocap.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
