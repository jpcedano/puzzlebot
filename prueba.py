detección de círculos con canny Edge. 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleDetector(Node):
    def _init_(self):
        super()._init_('circle_detector')
        self.subscription = self.create_subscription(Image, 'camera/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, 'camera/image_with_circles', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        green_lower = np.array([35, 100, 100])
        green_upper = np.array([85, 255, 255])
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        yellow_lower = np.array([25, 100, 100])
        yellow_upper = np.array([35, 255, 255])
        
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
        
        edges = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 100, 200)
        
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                   param1=50, param2=30, minRadius=10, maxRadius=50)
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if mask_green[y, x] > 0:
                    color_label = 'Green'
                    color = (0, 255, 0)
                elif mask_red[y, x] > 0:
                    color_label = 'Red'
                    color = (0, 0, 255)
                elif mask_yellow[y, x] > 0:
                    color_label = 'Yellow'
                    color = (0, 255, 255)
                else:
                    continue
                
                cv2.circle(frame, (x, y), r, color, 4)
                cv2.putText(frame, color_label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                break
        
        processed_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(processed_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
