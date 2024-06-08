import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import numpy as np

class TrafficLightDetector(Node):
    def _init_(self):
        super()._init_('traffic_light_detector')
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image, 
            'video_source/raw', 
            self.camera_callback, 
            10
        )
        self.traffic_light_pub = self.create_publisher(
            Image, 
            '/traffic_light_image', 
            10
        )
        self.signal_pub = self.create_publisher(
            Float32, 
            '/traffic_light_signal', 
            10
        )

        # Define color ranges in HSV
        self.red_range = (np.array([0, 70, 50]), np.array([10, 255, 255]))
        self.yellow_range = (np.array([20, 100, 100]), np.array([30, 255, 255]))
        self.green_range = (np.array([40, 50, 50]), np.array([90, 255, 255]))

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Detect and highlight circles
        frame, signal = self.detect_and_highlight_circles(frame)
        
        # Convert OpenCV image back to ROS Image message
        out_img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.traffic_light_pub.publish(out_img_msg)
        
        # Publish signal
        self.signal_pub.publish(Float32(data=signal))

    def detect_and_highlight_circles(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                   param1=50, param2=30, minRadius=5, maxRadius=30)

        signal = 0.0  # Default signal value

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                circle_area = frame[y - r:y + r, x - r:x + r]
                hsv_circle = cv2.cvtColor(circle_area, cv2.COLOR_BGR2HSV)
                mean_hsv = np.mean(hsv_circle, axis=(0, 1))

                if self.is_color(mean_hsv, self.red_range):
                    color = (0, 0, 255)  # Red
                    color_name = "Red"
                    signal = 1.0
                elif self.is_color(mean_hsv, self.yellow_range):
                    color = (0, 255, 255)  # Yellow
                    color_name = "Yellow"
                    signal = 2.0
                elif self.is_color(mean_hsv, self.green_range):
                    color = (0, 255, 0)  # Green
                    color_name = "Green"
                    signal = 3.0
                else:
                    color = (255, 255, 255)  # White for other colors
                    color_name = "Unknown"

                cv2.circle(frame, (x, y), r, color, 2)
                cv2.rectangle(frame, (x - r, y - r), (x + r, y + r), color, 2)

                if color_name in ["Red", "Yellow", "Green"]:
                    cv2.putText(frame, f"Traffic Light: {color_name}", (x - r, y - r - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return frame, signal

    def is_color(self, hsv_value, color_range):
        return color_range[0][0] <= hsv_value[0] <= color_range[1][0] and \
               color_range[0][1] <= hsv_value[1] <= color_range[1][1] and \
               color_range[0][2] <= hsv_value[2] <= color_range[1][2]


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
