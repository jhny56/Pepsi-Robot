#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import torch
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

        # Load the trained model from the same directory as the script
        try:
            weights_path = 'weightspepsican.pt'
            self.model = torch.load(weights_path)
            self.model.eval()
            logger.info('Model loaded successfully from weightspepsican.pt')
        except FileNotFoundError as e:
            logger.error(f"Failed to load weights: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error loading model: {e}")
            raise

        # Initialize frame counter
        self.frame_counter = 0
        
        # Set the frame skip interval
        self.frame_skip_interval = 7  # Adjust this value to control frequency

        logger.info('PepsiCanDetector node has been started.')

    def image_callback(self, msg):
        logger.debug('Received image message.')
        
        # Update frame counter
        self.frame_counter += 1

        # Process only every Nth frame
        if self.frame_counter % self.frame_skip_interval != 0:
            logger.debug('Skipping inference for this frame.')
            return

        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            logger.debug('Converted image to OpenCV format.')

            # Preprocess the image for the model
            input_image = self.preprocess_image(frame)

            # Perform detection using the loaded model
            result = self.model(input_image)

            # Post-process the detections
            self.process_detections(result, frame.shape[1])

        except Exception as e:
            logger.error(f'Failed to process image: {e}')

    def preprocess_image(self, frame):
        # Resize, normalize, and convert image to the format your model expects
        input_image = cv2.resize(frame, (640, 640))  # Example resize, adjust as needed
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        input_image = input_image.transpose(2, 0, 1)  # Convert HxWxC to CxHxW
        input_image = input_image / 255.0  # Normalize pixel values to [0, 1]
        input_image = torch.tensor(input_image, dtype=torch.float32).unsqueeze(0)  # Add batch dimension
        return input_image

    def process_detections(self, result, frame_width):
        logger.debug('Processing detections.')
        # Check if Pepsi can is detected and publish centroid and boolean
        detected = False

        # Assuming result contains bounding box and class data
        if result is not None and len(result.xyxy[0]) > 0:
            for detection in result.xyxy[0]:  # Adjust indexing as per your model's output format
                if detection[-1] == "pepsi":  # Assuming last element is the class label
                    logger.info("Pepsi can detected!")
                    detected = True

                    # Calculate the centroid of the detected Pepsi can
                    x_min = detection[0].item()
                    y_min = detection[1].item()
                    x_max = detection[2].item()
                    y_max = detection[3].item()

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





