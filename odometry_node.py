import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.angle_publisher = self.create_publisher(Float32, 'odom_angle', 10)
        self.x_publisher = self.create_publisher(Float32, 'odom_x', 10)
        self.y_publisher = self.create_publisher(Float32, 'odom_y', 10)

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.wl_subscription = self.create_subscription(Float32, '/VelocityEncL', self.wl_callback, qos_profile)
        self.wr_subscription = self.create_subscription(Float32, '/VelocityEncR', self.wr_callback, qos_profile)
        self.wl = 0.0
        self.wr = 0.0
        self.angulo_actual = 0.0
        self.angulo = 0.0
        self.x_actual = 0.0
        self.x = 0.0
        self.y_actual = 0.0
        self.y = 0.0
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.odometry_node)

    def wl_callback(self, msg):
        self.wl = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data

    def truncated_remainder(self, dividend, divisor):
        divided_number = dividend / divisor
        divided_number = \
            -int(-divided_number) if divided_number < 0 else int(divided_number)

        remainder = dividend - divisor * divided_number

        return remainder

    def transform_to_pipi(self, input_angle):
        p1 = self.truncated_remainder(input_angle + np.sign(input_angle) * np.pi, 2 * np.pi)
        p2 = (np.sign(np.sign(input_angle)
                    + 2 * (np.sign(np.fabs((self.truncated_remainder(input_angle + np.pi, 2 * np.pi))
                                        / (2 * np.pi))) - 1))) * np.pi

        output_angle = p1 - p2

        return output_angle

    def odometry_node(self):
        wl = self.wl
        wr = self.wr
        angulo = self.angulo
        radio_llanta = 0.05
        distancia_llantas = 0.19
        diferencial_tiempo = 0.01
        angulo_actual = self.angulo_actual
        x_actual = self.x_actual
        y_actual = self.y_actual
        x = self.x
        y = self.y

        angulo_actual = (radio_llanta * ((wr - wl) / distancia_llantas) * diferencial_tiempo)
        self.angulo += angulo_actual
        #self.angulo = self.transform_to_pipi(self.angulo)


        x_actual = ((radio_llanta * ((wr + wl) / 2.0) * diferencial_tiempo) * math.cos(self.angulo))
        self.x += x_actual


        y_actual = ((radio_llanta * ((wr + wl) / 2.0) * diferencial_tiempo) * math.sin(self.angulo))
        self.y += y_actual

        # Publish the angle
        angle_msg = Float32()
        angle_msg.data = self.angulo
        self.angle_publisher.publish(angle_msg)

        # Publish the x value
        x_msg = Float32()
        x_msg.data = self.x
        self.x_publisher.publish(x_msg)

        # Publish the y value
        y_msg = Float32()
        y_msg.data = self.y
        self.y_publisher.publish(y_msg)

        print("Angulo Actual: ", angulo*180/math.pi)
        print("Posición X Actual: ", x)
        print("Posición Y Actual: ", y)



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
