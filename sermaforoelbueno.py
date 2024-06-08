import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('TrafficLightDetector_node')

        # Cambiar el nombre del tópico de suscripción a "/videosource/raw"
        self.image_sub = self.create_subscription(Image, '/videosource/raw', self.camera_callback, 10)

        self.image_pub = self.create_publisher(Image, '/traffic_light_image', 10)
        self.signal_pub = self.create_publisher(Float32, '/traffic_light_signal', 10)

        self.traffic_light_signal = Float32()
        self.image_received_flag = False
        self.bridge = CvBridge()

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Traffic Light Detector Node started')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info(f'Failed to get an image: {str(e)}')

    def timer_callback(self):
        if self.image_received_flag:
            # Resize frame to 360x270
            frame = cv2.resize(self.cv_img, (360, 270))
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the ROS Image message
            self.image_pub.publish(ros_image)

            # Detect colored circles
            result_frame, signal_value = self.detect_colored_circles(frame)

            # Publish the detected signal value
            self.signal_pub.publish(signal_value)

    def detect_colored_circles(self, frame):
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for green, red, and yellow
        green_lower = np.array([35, 100, 100])
        green_upper = np.array([85, 255, 255])
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        yellow_lower = np.array([25, 100, 100])
        yellow_upper = np.array([35, 255, 255])

        # Create masks for each color
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Initialize signal value
        signal_value = None

        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(frame, (9, 9), 2)

        # Apply Canny edge detection
        edges = cv2.Canny(cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY), 50, 150)

        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                                   param1=50, param2=30, minRadius=10, maxRadius=30)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Check which color the circle corresponds to
                if mask_green[y, x] > 0:
                    color_label = 'Green'
                    signal_value = 1.0
                    color = (0, 255, 0)
                elif mask_red[y, x] > 0:
                    color_label = 'Red'
                    signal_value = 0.0
                    color = (0, 0, 255)
                elif mask_yellow[y, x] > 0:
                    color_label = 'Yellow'
                    signal_value = 0.5
                    color = (0, 255, 255)
                else:
                    continue

                # Draw the circle and label on the frame
                cv2.circle(frame, (x, y), r, color, 4)
                cv2.putText(frame, color_label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # Only detect one circle at a time
                break

        return frame, signal_value

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
