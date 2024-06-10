import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.angle_publisher = self.create_publisher(Float32, 'odom_angle', 10)

        self.objetos_sub = self.create_subscription(String, 'detected_labels', self.objetos_callback, 10)
        self.contour_sub = self.create_subscription(Bool, 'FindContour', self.contour_callback, 10)

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.wl_subscription = self.create_subscription(Float32, '/VelocityEncL', self.wl_callback, qos_profile)
        self.wr_subscription = self.create_subscription(Float32, '/VelocityEncR', self.wr_callback, qos_profile)
        self.wl = 0.0
        self.wr = 0.0
        self.angulo_actual = 0.0
        self.angulo = 0.0
        self.timer_period = 0.01

        self.objetos = ""
        self.objeto_detectado = ""

        self.contour = False

        self.flag = False

        self.timer = self.create_timer(self.timer_period, self.odometry_node)

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def objetos_callback(self, msg):
        self.objetos = msg.data.split(", ")
        if self.objetos:
            self.objeto_detectado = self.objetos[0]
        else:
            self.objeto_detectado = ""

    def contour_callback(self, msg):
        self.contour = msg.data

    def odometry_node(self):
        wl = self.wl
        wr = self.wr
        angulo = self.angulo
        radio_llanta = 0.05
        distancia_llantas = 0.19
        diferencial_tiempo = 0.01
        angulo_actual = self.angulo_actual

        if self.objeto_detectado == 'turnleft_sgl' or self.objeto_detectado == 'round_sgl':
            flag = True
        if flag == True:
            angulo_actual = ((radio_llanta * ((wr - wl) / distancia_llantas) * diferencial_tiempo) * 180 / math.pi)
            self.angulo += angulo_actual
            # Publish the angle
            angle_msg = Float32()
            angle_msg.data = self.angulo
            self.angle_publisher.publish(angle_msg)
        if self.objeto_detectado == 'workers_sgl':
            self.angulo = 0.0
            flag == False

        print("Angulo Actual: ", angulo)

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometryNode()
    try:
        rclpy.spin(odometry_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
