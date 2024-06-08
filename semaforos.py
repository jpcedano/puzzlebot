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

        self.image_sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        self.traffic_light_pub = self.create_publisher(Image, '/traffic_light_image', 10)
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
            self.get_logger().info(f'Failed to get an image: {str(e)}')

    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()
            resized_image = cv2.resize(image, (360, 270))
            gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.medianBlur(gray, 11)

            color = (0, 0, 0)
            signal_value = Float32()
            signal_value.data = 1.0  # Default to green if no signal detected

            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=30, minRadius=5, maxRadius=30)

            if circles is not None and len(circles[0]) > 0:
                for circle in circles[0]:
                    x, y, r = circle
                    x1, y1, x2, y2 = int(x - r), int(y - r), int(x + r), int(y + r)
                    circle_area = resized_image[y1:y2, x1:x2]

                    hsv_circle = cv2.cvtColor(circle_area, cv2.COLOR_BGR2HSV)
                    mean_hsv = np.mean(hsv_circle, axis=(0, 1))

                    if 0 <= mean_hsv[0] < 30 or 160 <= mean_hsv[0] <= 180:
                        color = (0, 0, 255)
                        signal_value.data = 0.0
                        print("Rojo Detectado")

                    elif 35 <= mean_hsv[0] <= 55:
                        color = (0, 255, 255)
                        signal_value.data = 0.5
                        print("Amarillo Detectado")

                    elif 60 <= mean_hsv[0] <= 85:
                        color = (0, 255, 0)
                        signal_value.data = 1.0
                        print("Verde Detectado")

                    cv2.circle(resized_image, (int(x), int(y)), int(r), color, 2)
                    cv2.rectangle(resized_image, (int(x) - int(r), int(y) - int(r)), (int(x) + int(r), int(y) + int(r)), color, 2)

                    if color in [(0, 0, 255), (0, 255, 255), (0, 255, 0)]:
                        cv2.putText(resized_image, "Semaforo en", (int(x) - int(r), int(y) - int(r) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            else:
                print("Nothing detected")

            self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))
            self.signal_pub.publish(signal_value)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
