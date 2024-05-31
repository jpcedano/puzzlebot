import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.sub = self.create_subscription(Image, 'error', self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',qos_profile)
        self.robot_vel = Twist()
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()


            region_of_interest = image[320:360,70:200]
            #blurred = cv2.GaussianBlur(image, (5, 5), 0) # gaussian filter
            gray = cv2.cvtColor(region_of_interest, cv2.COLOR_BGR2GRAY)
            _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            #cv2.imshow("Seguidor de Linea",thresholded)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
                #self.get_logger().info("Exit requested by user.")
                #rclpy.shutdown()
            # Adaptative threshold
            #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2) 
        
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Assume the largest contour is the line
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # Draw a circle at the center of the line
                    mark_size = 2  # Puedes ajustar esto según la estética deseada

                    # Dibujar un círculo en el centro de la línea
                    cv2.circle(region_of_interest, (cx, cy), mark_size, (0, 255, 0), -1)

                    # Calculate error from the center of the image
                    img_center = region_of_interest.shape[1]//2
                    error = cx - img_center
                    print('Centro de imagen:', img_center)
                    print('Posicion de linea:', cx)
                    print('Error:', error)
                    self.robot_vel.angular.z = -float(error) / 200  # P-controller for steering
                    self.robot_vel.linear.x = 0.15  # Constant forward speed
                else:
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
            else:
                self.get_logger().info('No line detected')
                self.robot_vel.angular.z = 0.0
                self.robot_vel.linear.x = 0.0

            self.cmd_vel_pub.publish(self.robot_vel)
            self.pub.publish(self.bridge.cv2_to_imgmsg(region_of_interest, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
