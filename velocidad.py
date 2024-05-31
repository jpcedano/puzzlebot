import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.sub = self.create_subscription(Int32, 'error', self.camera_callback, 10)
        qos_profile = QoSProfile(depth=10,reliability = ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',qos_profile)
        self.robot_vel = Twist()
        self.error = 0
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data
    
    def timer_callback(self, msg):  
        self.robot_vel.angular.z = -float(self.error) / 200  # P-controller for steering
        self.robot_vel.linear.x = 0.15  # Constant forward speed
        self.cmd_vel_pub.publish(self.robot_vel)
        
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
