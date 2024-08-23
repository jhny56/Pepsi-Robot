#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np

class WhiteColorDetector(Node):
    def __init__(self):
        super().__init__('white_color_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Initialize publishers
        self.centroid_pub = self.create_publisher(Point, '/white_color_centroid', 10)
        self.detection_pub = self.create_publisher(Bool, '/white_color_detection', 10)

        self.get_logger().info('WhiteColorDetector node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect white color in the image
            detected, centroid = self.detect_white_color(frame)

            if detected:
                self.get_logger().info("White color detected!")
                # Calculate and publish the centroid of the white color
                self.publish_centroid(centroid)
            else:
                self.get_logger().info("No white color detected.")

            # Publish detection status
            self.publish_detection_status(detected)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_white_color(self, frame):
        # Convert the image to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range for white color in HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        # Create a mask for white color
        mask = cv2.inRange(hsv_frame, lower_white, upper_white)

        # Find contours of the detected white regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        centroid = (0.0, 0.0)

        if contours:
            detected = True
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # Compute the centroid of the largest contour
            moments = cv2.moments(largest_contour)
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                centroid = (cx, cy)

        return detected, centroid

    def publish_centroid(self, centroid):
        centroid_msg = Point()
        centroid_msg.x = centroid[0]
        centroid_msg.y = centroid[1]
        centroid_msg.z = 0.0
        self.centroid_pub.publish(centroid_msg)
        self.get_logger().info(f"Published white color centroid: ({centroid[0]}, {centroid[1]})")

    def publish_detection_status(self, detected):
        detection_status = Bool()
        detection_status.data = detected
        self.detection_pub.publish(detection_status)
        self.get_logger().info(f"Published white color detection status: {'Detected' if detected else 'Not Detected'}")

def main(args=None):
    rclpy.init(args=args)
    node = WhiteColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
