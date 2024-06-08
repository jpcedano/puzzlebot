import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32

# Variables globales para valores HSV seleccionados
lower_color = np.array([110, 50, 50])
upper_color = np.array([130, 255, 255])

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/object_detection/image', 10)
        self.distance_pub = self.create_publisher(Float32, '/object_detection/distance', 10)
        self.face_cascade = cv2.CascadeClassifier('C:/Users/rodri/anaconda3/pkgs/libopencv-4.9.0-qt6_py312hd35d245_612/Library/etc/haarcascades/haarcascade_frontalface_default.xml')

    def detect_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask = np.uint8(mask)
        res = cv2.bitwise_and(frame, frame, mask=mask[:,:,0])
        return res

    def calculate_distance(self, apparent_size, real_size_width, real_size_height, focal_length_width, focal_length_height):
        distance_width = (real_size_width * focal_length_width) / apparent_size[0]
        distance_height = (real_size_height * focal_length_height) / apparent_size[1]
        distance = (distance_width + distance_height) / 2
        return distance

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        color_detected = self.detect_color(frame)
        image_msg = self.bridge.cv2_to_imgmsg(color_detected, "bgr8")
        self.image_pub.publish(image_msg)
        
        # Segunda parte: Deteción de credenciales y caras
        # Detectar objetos del color seleccionado
        gray_frame = cv2.cvtColor(color_detected, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                filtered_contours.append(contour)
        for contour in filtered_contours:
            x,y,w,h = cv2.boundingRect(contour)
            apparent_size = (w, h)
            distance = self.calculate_distance(apparent_size, real_object_width, real_object_height, focal_length_width, focal_length_height)
            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_pub.publish(distance_msg)
            # Detectar caras dentro del rectángulo
            roi_frame = frame[y:y+h, x:x+w]
            gray_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray_roi, 1.3, 5)
            for (fx, fy, fw, fh) in faces:
                cv2.rectangle(roi_frame, (fx, fy), (fx+fw, fy+fh), (0, 255, 0), 2)
                # Verificar si se detecta una cara dentro del rectángulo
                if len(faces) > 0:
                    # Mostrar texto "Estudiante TEC"
                    cv2.putText(frame, "Estudiante TEC", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
