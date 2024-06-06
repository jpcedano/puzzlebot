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
        self.objetos = []
        self.objeto_detectado = ""

        self.turnleft_signal_detected = False
        self.round_signal_detected = False
        self.straight_signal_detected = False
        self.stop_signal_detected = False

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data

    def objetos_callback(self, msg):
        self.objetos = msg.data.split(", ")
        self.objeto_detectado = self.objetos[0] if self.objetos else ""

    def timer_callback(self):
        self.process_signals()
        self.cmd_vel_pub.publish(self.robot_vel)

    def process_signals(self):
        if self.contour:
            self.process_contour()
        else:
            self.process_no_contour()
        
        # Reset flags if no relevant object detected
        self.reset_flags()

    def process_contour(self):
        self.robot_vel.angular.z = -float(self.error) / 400  # P-controller for steering
        self.robot_vel.linear.x = 0.05 if self.objeto_detectado == 'workers_sgl' else 0.1

    def process_no_contour(self):
        if self.objeto_detectado == 'turnleft_sgl' and not self.turnleft_signal_detected:
            self.turnleft_signal_detected = True
            self.perform_turn_left()
        elif self.objeto_detectado == 'round_sgl' and not self.round_signal_detected:
            self.round_signal_detected = True
            self.perform_roundabout()
        elif self.objeto_detectado == 'straight_sgl' and not self.straight_signal_detected:
            self.straight_signal_detected = True
            self.perform_straight()
        elif self.objeto_detectado == 'stop_sgl' and not self.stop_signal_detected:
            self.stop_signal_detected = True
            self.perform_stop()
        else:
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0

    def perform_turn_left(self):
        for _ in range(400):
            self.robot_vel.angular.z = -0.01
            self.robot_vel.linear.x = 0.0
        for _ in range(500):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.1
        for _ in range(1250):
            self.robot_vel.angular.z = 0.05
            self.robot_vel.linear.x = 0.0

    def perform_roundabout(self):
        for _ in range(500):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.1
        for _ in range(1500):
            self.robot_vel.angular.z = -0.05
            self.robot_vel.linear.x = 0.0

    def perform_straight(self):
        for _ in range(200):
            self.robot_vel.angular.z = 0.05
            self.robot_vel.linear.x = 0.0
        for _ in range(500):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
        for _ in range(500):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.1

    def perform_stop(self):
        for _ in range(1000):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
        for _ in range(1000):
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.1

    def reset_flags(self):
        if self.objeto_detectado not in ['turnleft_sgl', 'round_sgl', 'straight_sgl', 'stop_sgl']:
            self.turnleft_signal_detected = False
            self.round_signal_detected = False
            self.straight_signal_detected = False
            self.stop_signal_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
