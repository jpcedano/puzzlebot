class YoloV8Node(Node):
    def __init__(self):
        super().__init__('yolo_v8_node')
        self.publisher_image = self.create_publisher(Image, 'annotated_frames', 10)
        self.publisher_label = self.create_publisher(String, 'detected_labels', 10)
        self.subscription = self.create_subscription(Image, 'video_source/raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO(r"/home/puzzlebot/proyecto_final/proyecto_final/modelos/best_actualizado.pt")
        self.signal_sequence = ["turnleft_sgl", "workers_sgl", "round_sgl", "stop_sgl", "workers_sgl", "straight_sgl"]
        self.current_sequence_index = 0
        self.get_logger().info('YOLO V8 Node started')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

        # Perform inference on the frame
        results = self.model(frame)

        # Extract labels
        labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls]

        # Check if the current sequence is detected
        sequence_length = len(self.signal_sequence)
        if labels[:sequence_length] == self.signal_sequence:
            # Publish the detected sequence
            labels_str = ', '.join(self.signal_sequence)
            self.publisher_label.publish(String(data=labels_str))

            # Move to the next sequence
            self.current_sequence_index += 1

            # If all sequences have been detected, reset the index
            if self.current_sequence_index >= len(labels) - sequence_length + 1:
                self.current_sequence_index = 0

        # Annotate the frame with detection results
        annotated_frame = results[0].plot()

        # Convert OpenCV image to ROS 2 Image Message
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')

        # Publish the annotated frame
        self.publisher_image.publish(img_msg)
