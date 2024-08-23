#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np

class BlackBandDetector(Node):
    def __init__(self):
        super().__init__('blackband_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # Publisher for black band detection
        self.band_pub = self.create_publisher(Bool, '/black_band_detected', 10)
        self.get_logger().info('BlackBandDetector node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect black band in the image
            detected = self.detect_black_band(frame)

            if detected:
                self.get_logger().info("Black band detected!")
            else:
                self.get_logger().info("No black band detected.")

            # Publish detection status
            self.publish_detection(detected)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_black_band(self, frame):
        # Convert the image to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range for black color in HSV
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        # Create a mask for black color
        mask = cv2.inRange(hsv_frame, lower_black, upper_black)

        # Find contours of the detected black regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False

        if contours:
            # Check if any contours have a significant area
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100: 
                    detected = True
                    break

        return detected

    def publish_detection(self, detected):
        detection_msg = Bool()
        detection_msg.data = detected
        self.band_pub.publish(detection_msg)
        self.get_logger().debug(f"Published detection status: {detected}")

def main(args=None):
    rclpy.init(args=args)
    node = BlackBandDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
