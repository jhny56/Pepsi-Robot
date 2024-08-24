#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('blue_color_detector')

class BlueColorDetector(Node):
    def __init__(self):
        super().__init__('Blue_detector')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.centroid_pub = self.create_publisher(Point, '/blue_color_centroid', 10)
        self.detection_pub = self.create_publisher(Bool, '/blue_color_detection', 10)

        logger.info('BlueColorDetector node has been started.')

    def image_callback(self, msg):
        logger.debug('Received image message.')
        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            logger.debug('Converted image to OpenCV format.')

            # Convert image to HSV color space for easier color detection
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the range for the blue color in HSV
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

            # Create a mask for the blue color
            mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

            # Find contours of the blue areas
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            center_x = float(mask.shape[1])/ 2 

            # If any blue contours are found, process the largest one
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                centroid_x = x + w / 2
                centroid_y = y + h / 2
                self.publish_centroid(centroid_x, centroid_y,center_x)
                self.publish_detection_status(True)
                logger.info("Blue color detected!")
            else:
                self.publish_detection_status(False)
                logger.info("Blue color not detected.")

        except Exception as e:
            logger.error(f'Failed to process image: {e}')

    def publish_centroid(self, x, y,center_x):
        centroid = Point()
        centroid.x = x
        centroid.y = y
        centroid.z = center_x
        self.centroid_pub.publish(centroid)
        logger.info(f"Published blue color centroid: ({x}, {y})")

    def publish_detection_status(self, detected):
        detection_status = Bool()
        detection_status.data = detected
        self.detection_pub.publish(detection_status)
        logger.info(f"Published blue color detection status: {'Detected' if detected else 'Not Detected'}")

def main(args=None):
    rclpy.init(args=args)
    node = BlueColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
