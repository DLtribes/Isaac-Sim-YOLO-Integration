import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class Yolov8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',  # The topic where Isaac Sim publishes camera frames
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/camera/detections', 10)
        self.bridge = CvBridge()
        self.model = YOLO('~/your_path/yolov8n.pt')  # Path to your YOLOv8 model

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform object detection
        results = self.model(cv_image)

        # Draw bounding boxes on the image
        annotated_frame = results[0].plot()

        # Convert back to ROS Image message
        detection_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        detection_msg.header = msg.header  # Retain original message header
        self.publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    yolov8_detector = Yolov8Detector()
    rclpy.spin(yolov8_detector)
    yolov8_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
