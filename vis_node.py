#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.image_publisher = self.create_publisher(Image, 'yolo_detected_frame', 10)
        self.coordinate_publisher = self.create_publisher(Point, 'object_coordinates', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.video_path = "/home/girvan/Downloads/vid2.mp4"
        self.cap = cv2.VideoCapture(2)
        self.bridge = CvBridge()

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/girvan/yolov5/runs/train/exp4/weights/best.pt', trust_repo=True)
        self.get_logger().info('YOLO model loaded')

    def process_frame(self, frame):
        results = self.model(frame)

        annotated_frame = results.render()[0]

        detections = results.xyxy[0]
        return annotated_frame, detections

    def publish_coordinates(self, detections):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        center_offset_x = frame_width / 2
        center_offset_y = frame_height / 2

        for det in detections:
            x_min, y_min, x_max, y_max, conf, cls = det.cpu().numpy()
            center_x = (x_min + x_max) / 2 - center_offset_x
            center_y = (y_min + y_max) / 2 - center_offset_y

            point = Point()
            point.x = center_x
            point.y = center_y
            point.z = 0.0

            self.coordinate_publisher.publish(point)
            self.get_logger().info(f"Published object at ({center_x:.0f}, {center_y:.0f})")


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video")
            self.cap.release()
            self.destroy_node()
            rclpy.shutdown()
            return
        
        processed_frame, detections = self.process_frame(frame)

        ros_image = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
        self.image_publisher.publish(ros_image)
        self.get_logger().info('Published YOLO-detected frame')

        if len(detections) > 0:
            self.publish_coordinates(detections)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)

    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()