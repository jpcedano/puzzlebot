import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.error_sub = self.create_subscription(Int32, 'error', self.error_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'FindContour', self.contour_callback, 10)
        self.objetos_sub = self.create_subscription(String, 'detected_labels', self.objetos_callback, 10)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        self.robot_vel = Twist()
        self.error = 0
        self.contour = False
        self.objetos = ""
        self.objeto_detectado = ""

        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('angular_speed', 0.1)
        self.declare_parameter('rotate_duration', 0.4)
        self.declare_parameter('forward_duration', 0.5)
        self.declare_parameter('sleep_time', 2.0)

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data

    def objetos_callback(self, msg):
        self.objetos = msg.data.split(", ")
        self.objeto_detectado = self.objetos[0]

    def rotate(self):
        rotate_duration = self.get_parameter('rotate_duration').value
        angular_speed = self.get_parameter('angular_speed').value

        self.robot_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(self.robot_vel)
        self.create_timer(rotate_duration, self.stop_rotate)

    def stop_rotate(self):
        self.robot_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.robot_vel)

    def forward(self):
        forward_duration = self.get_parameter('forward_duration').value
        linear_speed = self.get_parameter('linear_speed').value

        self.robot_vel.linear.x = linear_speed
        self.cmd_vel_pub.publish(self.robot_vel)
        self.create_timer(forward_duration, self.stop_forward)

    def stop_forward(self):
        self.robot_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(self.robot_vel)

    def timer_callback(self):
        self.get_logger().info(f'Detected Object: {self.objeto_detectado}')
        sleep_time = self.get_parameter('sleep_time').value

        if self.contour:
            if self.objeto_detectado == 'workers_sgl':
                self.get_logger().info('Workers signal detected')
                self.robot_vel.angular.z = (-float(self.error) / 400)
                self.robot_vel.linear.x = 0.05
            else:
                self.robot_vel.angular.z = (-float(self.error) / 400)
                self.robot_vel.linear.x = 0.15
        elif not self.contour and self.objeto_detectado == 'turnleft_sgl':
            self.get_logger().info('Turn left signal detected')
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
            self.forward()
            self.create_timer(sleep_time, self.rotate)
            self.create_timer(sleep_time * 2, self.forward)
        else:
            self.get_logger().info('No line detected')
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0

        self.cmd_vel_pub.publish(self.robot_vel)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
