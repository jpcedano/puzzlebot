import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSelector(Node):
    def _init_(self):
        super()._init_('color_selector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'video_source/raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.frame = None

    def listener_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Select Color', self.frame)
        cv2.setMouseCallback('Select Color', self.on_double_click)
        cv2.waitKey(1)

    def on_double_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK and self.frame is not None:
            hsv_pixel = cv2.cvtColor(np.uint8([[self.frame[y, x]]]), cv2.COLOR_BGR2HSV)[0][0]
            print("Selected HSV Color:", hsv_pixel)
            hue_adjustment = 20  # H value adjustment
            lower_color = np.array([max(hsv_pixel[0] - hue_adjustment, 0), 50, 50])
            upper_color = np.array([min(hsv_pixel[0] + hue_adjustment, 179), 255, 255])
            print(f"Lower HSV: {lower_color}, Upper HSV: {upper_color}")

def main(args=None):
    rclpy.init(args=args)
    color_selector = ColorSelector()
    rclpy.spin(color_selector)
    color_selector.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
