import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolo_v8_node')
        #self.publisher_image = self.create_publisher(Image, 'annotated_frames', 10)
        self.publisher_label = self.create_publisher(String, 'detected_labels', 10)
        self.subscription = self.create_subscription(Image, 'video_source/raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO(r"/home/puzzlebot/proyecto_final/proyecto_final/modelos/best_actualizado.pt")
        self.get_logger().info('YOLO V8 Node started')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

        # Perform inference on the frame
        results = self.model(frame)

        # Annotate the frame with detection results
        #annotated_frame = results[0].plot()

        # Convert OpenCV image to ROS 2 Image Message
        #img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')

        # Publish the annotated frame
        #self.publisher_image.publish(img_msg)
        
        # Extract and publish labels
        labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls]
        labels_str = ', '.join(labels)
        print("Published label", labels_str)
        self.publisher_label.publish(String(data=labels_str))
        
def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
