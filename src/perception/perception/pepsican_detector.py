#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('pepsi_can_detector')

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.centroid_pub = self.create_publisher(Point, '/pepsi_can_centroid', 10)
        self.detection_pub = self.create_publisher(Bool, '/pepsi_can_detection', 10)

        # Initialize the InferenceHTTPClient
        self.client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com",
            api_key="bwZdRPnoYYRFnrcscwxp"  
        )
        self.model_id = "pepsi-can-detection-krjyn/1"

        logger.info('PepsiCanDetector node has been started.')

    def image_callback(self, msg):
        logger.debug('Received image message.')
        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            logger.debug('Converted image to OpenCV format.')

            # Convert image to JPEG format
            _, img_encoded = cv2.imencode('.jpg', frame)
            img_bytes = img_encoded.tobytes()

            # Use the inference client to perform detection
            try:
                result = self.client.infer(img_bytes, model_id=self.model_id)
                self.process_detections(result, frame.shape[1])  
            except Exception as e:
                logger.error(f"Error communicating with inference server: {e}")

        except Exception as e:
            logger.error(f'Failed to convert image: {e}')

    def process_detections(self, result, frame_width):
        logger.debug('Processing detections.')
        # Check if Pepsi can is detected and publish centroid and boolean
        detected = False
        for detection in result.get('predictions', []):
            if detection['class'] == 'pepsi':
                logger.info("Pepsi can detected!")
                detected = True

                # Calculate the centroid of the detected Pepsi can
                x_min = detection['x'] - detection['width'] / 2
                y_min = detection['y'] - detection['height'] / 2
                x_max = detection['x'] + detection['width'] / 2
                y_max = detection['y'] + detection['height'] / 2

                centroid_x = (x_min + x_max) / 2
                centroid_y = (y_min + y_max) / 2
                center_x = frame_width / 2  
                self.publish_centroid(centroid_x, centroid_y, center_x)
                break

        self.publish_detection_status(detected)

    def publish_centroid(self, x, y, center_x):
        centroid = Point()
        centroid.x = x
        centroid.y = y
        centroid.z = center_x  
        self.centroid_pub.publish(centroid)
        logger.info(f"Published Pepsi can centroid: ({x}, {y}), with center_x: {center_x}")

    def publish_detection_status(self, detected):
        detection_status = Bool()
        detection_status.data = detected
        self.detection_pub.publish(detection_status)
        logger.info(f"Published Pepsi can detection status: {'Detected' if detected else 'Not Detected'}")

def main(args=None):
    rclpy.init(args=args)
    node = PepsiCanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


