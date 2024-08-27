#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_publisher', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_image)  

    def publish_image(self):
        # Read the image from the file
        img = cv2.imread('image.jpg')

        if img is not None:
            self.get_logger().info('Publishing image...')
            # Convert the image to a ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            # Publish the image message
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().error('Failed to read image.jpg')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
