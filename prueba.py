import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.error_sub = self.create_subscription(Int32, 'error', self.error_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'FindContour', self.contour_callback, 10)
        self.objetos_sub = self.create_subscription(String, 'detected_labels', self.objetos_callback, 10)
        self.semaforo_sub = self.create_subscription(String, 'signal_value', self.semaforo_callback, 10)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        
        self.robot_vel = Twist()
        self.error = 0
        self.semaforo_detectado = ""
        self.contour = False
        self.objetos = ""
        self.objeto_detectado = ""
        self.signal_value = 1.0

        self.turnleft_signal_detected = False  # Flag for turn left signal detection
        self.round_signal_detected = False
        self.straight_signal_detected = False
        self.stop_signal_detected = False

        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback)

    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data

    def objetos_callback(self, msg):
        self.objetos = msg.data
        self.objetos = self.objetos.split(", ")
        self.objeto_detectado = self.objetos[0]

    def semaforo_callback(self, msg):
        self.semaforo_detectado = msg.data
        print("Semaforo callback - color:", self.semaforo_detectado)

    def timer_callback(self):

        if self.semaforo_detectado == "red_light":
            self.signal_value = 0.0
        elif self.semaforo_detectado == "yellow_light":
            self.signal_value = 0.5
        else:
            self.signal_value = 1.0

        if self.contour:
            if self.objeto_detectado == 'workers_sgl':
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.05 # Adjusted forward speed   
                self.cmd_vel_pub.publish(self.robot_vel)   
                                
            else:
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.15 * self.signal_value  # Adjusted forward speed
                self.cmd_vel_pub.publish(self.robot_vel)

                print("color: ", self.semaforo_detectado)
                print("valor: ", self.signal_value)

        elif self.objeto_detectado:
            if not self.contour and self.objeto_detectado == 'turnleft_sgl':
                if not self.turnleft_signal_detected:
                    self.turnleft_signal_detected = True  # Set flag to True

                    for _ in range(300):
                        self.robot_vel.angular.z = -0.01
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for _ in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)
                    
                    for _ in range(1350):
                        self.robot_vel.angular.z = 0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)                    

                
                    if self.objeto_detectado != 'turnleft_sgl':
                        self.turnleft_signal_detected = False

            elif not self.contour and self.objeto_detectado == 'round_sgl':
                if not self.round_signal_detected:
                    self.round_signal_detected = True  # Set flag to True
                    for _ in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for _ in range(1500):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                
                    if self.objeto_detectado != 'round_sgl':
                        self.round_signal_detected = False

            elif not self.contour and self.objeto_detectado == 'straight_sgl':
                if not self.straight_signal_detected:
                    self.straight_signal_detected = True

                    for _ in range(300):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for _ in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for _ in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)

                    if self.objeto_detectado != 'straight_sgl':
                        self.straight_signal_detected = False

            elif not self.contour and self.objeto_detectado == 'stop_sgl':
                if not self.stop_signal_detected:
                    self.stop_signal_detected = True

                    for _ in range(1000):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for _ in range(200):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)                

                    for _ in range(1000):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel) 

                    if self.objeto_detectado != 'stop_sgl':
                        self.stop_signal_detected = False   

        else:
            self.turnleft_signal_detected = False  # Flag for turn left signal detection
            self.round_signal_detected = False
            self.straight_signal_detected = False
            self.stop_signal_detected = False
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
