#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Initialize publishers
        self.centroid_pub = self.create_publisher(Point, '/qr_code_centroid', 10)
        self.detection_pub = self.create_publisher(Bool, '/qr_code_detection', 10)

        self.qr_code_detector = cv2.QRCodeDetector()

        self.get_logger().info('QRCodeDetector node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_width = frame.shape[1]  

            # Detect and decode the QR code
            data, bbox, _ = self.qr_code_detector.detectAndDecode(frame)
            # print(bbox)
            detected = False
            if bbox is not None and len(bbox) > 0:
                # If a QR code was detected and successfully decoded
                detected = True
                # self.get_logger().info(f"QR Code detected: {data}")

                # Calculate and publish the centroid of the QR code
                centroid = self.calculate_centroid(bbox, frame_width)
                self.publish_centroid(centroid)
                self.draw_bbox(frame, bbox)
            else:
                # self.get_logger().info("No QR Code detected.")
                pass

            # Publish detection status
            self.publish_detection_status(detected)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def draw_bbox(self, frame, bbox):
        if bbox is not None:
            # Draw bounding box around the detected QR code
            n = len(bbox[0])
            for j in range(n):
                # Convert points to integer tuples
                point1 = tuple(int(v) for v in bbox[0][j])
                point2 = tuple(int(v) for v in bbox[0][(j + 1) % n])
                cv2.line(frame, point1, point2, (255, 0, 0), 3)

    def calculate_centroid(self, bbox, frame_width):
        # Calculate the centroid of the QR code
        if bbox is not None:
            points = np.array(bbox).reshape(-1, 2)
            x_center =  float(np.mean(points[:, 0]))
            y_center =  float(np.mean(points[:, 1]))
            center_x =  float(frame_width / 2)
            return (x_center, y_center, center_x)
        return (0.0, 0.0, 0.0)

    def publish_centroid(self, centroid):
        centroid_msg = Point()
        centroid_msg.x = centroid[0]
        centroid_msg.y = centroid[1]
        centroid_msg.z = centroid[2]  
        self.centroid_pub.publish(centroid_msg)
        # self.get_logger().info(f"Published QR code centroid: ({centroid[0]}, {centroid[1]}), with center_x: {centroid[2]}")

    def publish_detection_status(self, detected):
        detection_status = Bool()
        detection_status.data = detected
        self.detection_pub.publish(detection_status)
        # self.get_logger().info(f"Published QR code detection status: {'Detected' if detected else 'Not Detected'}")

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


