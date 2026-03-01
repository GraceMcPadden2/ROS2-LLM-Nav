import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

from rclpy.qos import qos_profile_sensor_data



class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # lightweight model
        
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            String,
            "/detections",
            10
        )

        self.get_logger().info("Object detector ready")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(frame)

        detected_labels = []

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]
                detected_labels.append(label)

        if detected_labels:
            msg = String()
            msg.data = str(detected_labels)
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()
