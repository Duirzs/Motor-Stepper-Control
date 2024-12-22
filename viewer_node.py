#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoViewer(Node):
    def __init__(self):
        super().__init__('video_viewer')
        self.subscription = self.create_subscription(
            Image,
            'yolo_detected_frame',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('YOLO Detection Output', frame)
        if cv2.waitKey(1) == 27:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VideoViewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()