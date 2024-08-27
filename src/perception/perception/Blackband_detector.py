#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class BlackBandDetector(Node):
    def __init__(self):
        super().__init__('black_band_detector')

        # Subscriber to the light sensor topic
        self.sensor_sub = self.create_subscription(Float32, '/light_sensor', self.sensor_callback, 10)
        
        # Publisher for black band detection
        self.band_pub = self.create_publisher(Bool, '/black_band_detected', 10)
        
        self.black_band_detected = False
        self.threshold = 0.2  # Threshold for detecting a black band

    def sensor_callback(self, msg):
        light_intensity = msg.data

        # Check if the detected light intensity corresponds to a black band
        if light_intensity < self.threshold:
            if not self.black_band_detected:
                self.get_logger().info("Black band detected!")
                self.black_band_detected = True
                self.publish_detection(True)
        else:
            if self.black_band_detected:
                self.get_logger().info("Black band no longer detected!")
                self.black_band_detected = False
                self.publish_detection(False)

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

