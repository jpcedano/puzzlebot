import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.error_pub = self.create_publisher(Int32, 'error',10)
        self.contour_pub = self.create_publisher(Bool,'FindContour',10)
        self.error = Int32()
        self.contour = Bool()
        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Line Follower Node started')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))

    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()

            region_of_interest = image[320:350,70:200]
            gray = cv2.cvtColor(region_of_interest, cv2.COLOR_BGR2GRAY)
            _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                self.contour.data = True
                self.contour_pub.publish(self.contour)
                # Assume the largest contour is the line
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # Draw a circle at the center of the line
                    mark_size = 2  

                    # Dibujar un círculo en el centro de la línea
                    cv2.circle(region_of_interest, (cx, cy), mark_size, (0, 255, 0), -1)

                    # Calculate error from the center of the image
                    img_center = region_of_interest.shape[1]//2
                    error = cx - img_center
                    print('Centro de imagen:', img_center)
                    print('Posicion de linea:', cx)
                    print('Error:', error)
                    
                    self.error.data = error
                    self.error_pub.publish(self.error)

            else:
                self.contour.data = False
                self.contour_pub.publish(self.contour)
                self.get_logger().info('No line detected')

            self.pub.publish(self.bridge.cv2_to_imgmsg(region_of_interest, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
