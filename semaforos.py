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
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))
    
    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.medianBlur(gray, 11)

            color = (0, 0, 0)
            signal_value = 0

            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                       param1=50, param2=30, minRadius=5, maxRadius=30)

            if circles is not None and len(circles[0]) > 0:
                filtered_circles = []
                for circle in circles[0]:
                    x, y, r = circle
                    circle_area = image[int(y) - int(r):int(y) + int(r), int(x) - int(r):int(x) + int(r)]
                    if np.any(circle_area):
                        hsv_circle = cv2.cvtColor(circle_area, cv2.COLOR_BGR2HSV)
                        mean_hsv = np.mean(hsv_circle, axis=(0, 1))
                        if (0 <= mean_hsv[0] < 30 or 160 <= mean_hsv[0] <= 180) or \
                           (35 <= mean_hsv[0] <= 55) or (60 <= mean_hsv[0] <= 85):
                            filtered_circles.append(circle)

                if len(filtered_circles) > 0:
                    circle = filtered_circles[0]
                    x, y, r = np.round(circle[0]), np.round(circle[1]), np.round(circle[2])

                    circle_area = image[int(y) - int(r):int(y) + int(r), int(x) - int(r):int(x) + int(r)]
                    hsv_circle = cv2.cvtColor(circle_area, cv2.COLOR_BGR2HSV)
                    mean_hsv = np.mean(hsv_circle, axis=(0, 1))

                    if 0 <= mean_hsv[0] < 30 or 160 <= mean_hsv[0] <= 180:
                        color = (0, 0, 255)
                        signal_value = 0.0
                        print("Rojo Detectado")
                    elif 35 <= mean_hsv[0] <= 55:
                        color = (0, 255, 255)
                        signal_value = 0.5
                        print("Amarillo Detectado")
                    elif 60 <= mean_hsv[0] <= 85:
                        color = (0, 255, 0)
                        signal_value = 1.0
                        print("Verde Detectado")

                    cv2.circle(image, (int(x), int(y)), int(r), color, 2)
                    cv2.rectangle(image, (int(x) - int(r), int(y) - int(r)), (int(x) + int(r), int(y) + int(r)), color, 2)

                    if color in [(0, 0, 255), (0, 255, 255), (0, 255, 0)]:
                        cv2.putText(image, f"Semaforo en", (int(x) - int(r), int(y) - int(r) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                print("Nothing detected")

            # Publish processed image
            self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            # Publish signal value
            self.signal_pub.publish(signal_value)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
