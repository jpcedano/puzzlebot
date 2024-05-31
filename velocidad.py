import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.error_sub = self.create_subscription(Int32, 'error', self.error_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'contour', self.contour_callback, 10)
        self.signal_sub = self.create_subscription(Float32, '/traffic_light_signal', self.signal_callback, 10)

        qos_profile = QoSProfile(depth=10,reliability = ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',qos_profile)

        self.robot_vel = Twist()
        self.error = 0
        self.contour = False
        self.signal = 1.0

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data

    def signal_callback(self, msg):
        self.signal = msg.data
    
    def timer_callback(self):  
        if self.contour == True:
            self.robot_vel.angular.z = (-float(self.error) / 200) * self.signal # P-controller for steering
            self.robot_vel.linear.x = 0.15 * self.signal  # Constant forward speed
            self.cmd_vel_pub.publish(self.robot_vel)
        else:
            self.robot_vel.angular.z = 0
            self.robot_vel.linear.x = 0
        self.cmd_vel_pub.publish(self.robot_vel)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
