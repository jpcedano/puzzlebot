import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32, String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.error_sub = self.create_subscription(Int32, 'error', self.error_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'FindContour', self.contour_callback, 10)
        self.objetos_sub = self.create_subscription(String, 'detected_labels', self.objetos_callback, 10)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        qos_profile_enc = rclpy.qos.qos_profile_sensor_data
        self.wl_subscripton = self.create_subscription(Float32, 'VelocityEncL',self.wl_callback, qos_profile_enc)
        self.rl_subscripton = self.create_subscription(Float32, 'VelocityEncR',self.wr_callback, qos_profile_enc)

        self.wl = 0.0
        self.wr = 0.0
        self.angulo_actual = 0.0
        self.angulo = 0.0
        
        self.robot_vel = Twist()
        self.error = 0
        self.contour = False
        self.objetos = ""
        self.objeto_detectado = ""

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

    def wl_callback(self,msg):
        self.wl = msg.data

    def wr_callback(self,msg):
        self.wr = msg.data   

    def timer_callback(self): 

        wl = self.wl
        wr = self.wr
        angulo = self.angulo
        radio_llanta = 0.05
        distancia_llantas = 0.19
        diferencial_tiempo = 0.01
        angulo_actual = self.angulo_actual

        
        #print("Round Flag: ",self.round_signal_detected)
        #print("Stop Flag: ",self.stop_signal_detected) 
        #print("Turn Left: ", self.turnleft_signal_detected)
        #print("Straight Flag: ",self.straight_signal_detected)

        print(self.objeto_detectado)
        if self.contour:
            if (self.objeto_detectado == 'workers_sgl'):
                #print("workers")
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.05 # Adjusted forward speed   
                self.cmd_vel_pub.publish(self.robot_vel)         
            else:
                self.robot_vel.angular.z = (-float(self.error) / 400)  # P-controller for steering
                self.robot_vel.linear.x = 0.1  # Adjusted forward speed
                self.cmd_vel_pub.publish(self.robot_vel)

        elif self.objeto_detectado:
            angulo_actual = (radio_llanta*((wl - wr)/distancia_llantas)*diferencial_tiempo)
            self.angulo += angulo_actual
            print("Angulo: ",angulo)
            if self.contour == False and self.objeto_detectado == 'turnleft_sgl':
                if not self.turnleft_signal_detected:
                    self.turnleft_signal_detected = True  # Set flag to True

                    #print("turn left signal")

                    for i in range(500):
                        self.robot_vel.angular.z = -0.01
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for j in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)

                    while self.angulo <= 90.0:
                        self.robot_vel.angular.z = 0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                
                    if (self.objeto_detectado != 'turnleft_sgl'):
                        self.turnleft_signal_detected = False
            

            elif self.contour == False and self.objeto_detectado == 'round_sgl':
                if not self.round_signal_detected:
                    self.round_signal_detected = True  # Set flag to True
                    #print("roundabout signal")
                    for i in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for j in range(1500):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                
                    if (self.objeto_detectado != 'round_sgl'):
                        self.round_signal_detected = False

            elif (self.contour == False and self.objeto_detectado == 'straight_sgl'):
                if not self.straight_signal_detected:
                    self.straight_signal_detected = True

                    #print("Straight Signal")

                    for i in range(300):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for i in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for i in range(500):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel)

                    if (self.objeto_detectado != 'straight_sgl'):
                        self.straight_signal_detected = False

            elif (self.contour == False and self.objeto_detectado == 'stop_sgl'):
                if not self.stop_signal_detected:
                    self.stop_signal_detected = True

                    #print("Stop Signal Detected")
                    for i in range(1000):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)

                    for i in range(200):
                        self.robot_vel.angular.z = -0.05
                        self.robot_vel.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)                

                    for i in range(1000):
                        self.robot_vel.angular.z = 0.0
                        self.robot_vel.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.robot_vel) 

                    if(self.objeto_detectado != 'stop_sgl'):
                        self.stop_signal_detected = False   

        else:
            self.turnleft_signal_detected = False  # Flag for turn left signal detection
            self.round_signal_detected = False
            self.straight_signal_detected = False
            self.stop_signal_detected = False
            #print("no line")
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
            angulo_actual = 0.0
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
