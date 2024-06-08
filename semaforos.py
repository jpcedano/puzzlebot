import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

class TrafficLight(Node):
    def __init__(self):
        super().__init__('TrafficLight_node')

        # Suscribirse al tema de video fuente
        self.image_sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        
        # Publicar la imagen procesada
        self.traffic_light_pub = self.create_publisher(Image, '/traffic_light_image', 10)
        
        # Publicar la señal del semáforo
        self.signal_pub = self.create_publisher(Float32, '/traffic_light_signal', 10)

        self.traffic_light_signal = Float32()
        self.image_received_flag = False
        self.bridge = CvBridge()

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Traffic Light Node started')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))

    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()
            resized_image = cv2.resize(image, (360, 270))
            hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

            # Definir los rangos de color para amarillo, rojo y verde
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([40, 255, 255])

            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])

            lower_green = np.array([40, 100, 100])
            upper_green = np.array([80, 255, 255])

            # Crear máscaras para cada color
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            masks = [mask_yellow, mask_red, mask_green]
            signal_values = [0.5, 0.0, 1.0]  # Amarillo, Rojo, Verde
            detected_signal = None

            for idx, mask in enumerate(masks):
                res = cv2.bitwise_and(resized_image, resized_image, mask=mask)
                img = cv2.medianBlur(res, 5)
                ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
                cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
                circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=22, minRadius=20, maxRadius=80)

                if circles is not None and len(circles[0]) > 0:
                    detected_signal = signal_values[idx]
                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                        cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
                    break

            # Publicar la imagen procesada
            self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))

            # Publicar la señal del semáforo
            if detected_signal is not None:
                self.traffic_light_signal.data = detected_signal
            else:
                self.traffic_light_signal.data = 1.0  # Default to green if no signal detected

            self.signal_pub.publish(self.traffic_light_signal)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
