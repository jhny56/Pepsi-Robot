#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO 
import logging
import numpy as np

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('pepsi_can_detector')

class PepsiCanDetector(Node):
    def __init__(self):
        super().__init__('pepsi_can_detector')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(CompressedImage, '/robot_interfaces/compressed', self.image_callback, 10)

        # Publishers
        self.centroid_pub = self.create_publisher(Point, '/pepsi_can_centroid', 10)
        self.detection_pub = self.create_publisher(Bool, '/pepsi_can_detection', 10)

        self.detectionImage = self.create_publisher(Image, '/pepsi_image_detection', 10)


        # Load the trained model from the specified path
        self.model = YOLO("/ros2_ws/weights.pt", task='detect')

        logger.info('PepsiCanDetector node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS CompressedImage message to a raw OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Convert BGR image to RGB for the model
            # cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Perform detection using the loaded model
            # print("CV IMAGE : ",cv_image)
            results = self.model.predict(source=cv_image, conf=0.25, iou=0.45)
            result = results[0]  
            annotated_image = result.plot() 


            centroid_y = None
            center_x = None

            if result.boxes.xyxy.shape[0] > 0:
                
                box = result.boxes.xyxy[0]
                xmin, ymin, xmax, ymax = box

                # Calculate centroid coordinates
                centroid_x = (xmin + xmax) / 2
                centroid_y = (ymin + ymax) / 2

                frame_width, frame_height = cv_image.shape[1], cv_image.shape[0]
                center_x = frame_width / 2

                detected = True
            
            annotated_ros_image = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")

            self.detectionImage.publish(annotated_ros_image)


            
            if detected:
                self.publish_centroid(centroid_x, centroid_y, center_x)
            self.publish_detection_status(detected)

            print("DETECTION : " ,detected)
            
            # cv2.imwrite("/ros2_ws/detected_pepsi.jpg", annotated_image)

        except Exception as e:
            logger.error(f'Failed to process image: {e}')

    def publish_centroid(self, x, y, center_x):
        centroid = Point()
        centroid.x = float(x)
        centroid.y = float(y)
        centroid.z = float(center_x)
        self.centroid_pub.publish(centroid)
        logger.info(f"Published Pepsi can centroid: ({x}, {y}) with center_x: {center_x}")

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




