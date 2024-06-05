import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32,String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.error_sub = self.create_subscription(Int32, 'error', self.error_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'FindContour', self.contour_callback, 10)
        self.objetos_sub = self.create_subscription(String,'detected_labels',self.objetos_callback,10)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        self.robot_vel = Twist()
        self.error = 0
        self.contour = False
        self.objetos = ""
        self.objeto_detectado = ""

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data

    def objetos_callback(self,msg):
        self.objetos = msg.data
        self.objetos = self.objetos.split(", ")
        self.objeto_detectado = self.objetos[0]

    def rotate(self):
        duration = 0.4
        self.robot_vel.angular.z = 0.1
        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(self.robot_vel)
        
        self.robot_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.robot_vel)

    def forward(self):
        duration_forward = 0.5
        self.robot_vel.linear.x = 0.1
        start_time = time.time()

        while time.time() - start_time < duration_forward:
            self.cmd_vel_pub.publish(self.robot_vel)
            
        self.robot_vel.linear.x = 0.0
        self.cmd_vel_pub.publish(self.robot_vel)



    def timer_callback(self):  
        rot_tiempo = 0.1
        duration = 2.0
        # contador = 0 

        print(self.objeto_detectado)
        if self.contour:
            if (self.objeto_detectado == 'workers_sgl'):
                print("workers")
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.05  # Adjusted forward speed   
                self.cmd_vel_pub.publish(self.robot_vel)         
            else:
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.15  # Adjusted forward speed
                self.cmd_vel_pub.publish(self.robot_vel)

        elif(self.contour == False and self.objeto_detectado == 'turnleft_sgl'):
            
            print("turn left signal")
            for i in range(500):
                self.robot_vel.angular.z = 0.0
                self.robot_vel.linear.x = 0.1
                self.cmd_vel_pub.publish(self.robot_vel)

            for j in range(500):
                self.robot_vel.angular.z = 0.05
                self.robot_vel.linear.x = 0.0
                self.cmd_vel_pub.publish(self.robot_vel)

        else:
            print("no line")
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
            self.cmd_vel_pub.publish(self.robot_vel)

        print(self.objeto_detectado)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
