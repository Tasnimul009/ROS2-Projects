#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

# YOLOv8
from ultralytics import YOLO


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')          # nano = fastest
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('show_window', True)

        model_path  = self.get_parameter('model').get_parameter_value().string_value
        self.conf   = self.get_parameter('confidence').get_parameter_value().double_value
        cam_topic   = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.show   = self.get_parameter('show_window').get_parameter_value().bool_value

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model loaded successfully!')

        self.bridge = CvBridge()

        # Subscriber — Gazebo camera
        self.sub = self.create_subscription(
            Image,
            cam_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.det_pub  = self.create_publisher(String, '/detections', 10)
        self.img_pub  = self.create_publisher(Image,  '/detections/image', 10)

        self.get_logger().info(f'Subscribed to: {cam_topic}')
        self.get_logger().info('Object Detection Node is READY.')

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        results = self.model(frame, conf=self.conf, verbose=False)

        detections = []
        for result in results:
            for box in result.boxes:
                class_id   = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                detections.append({
                    'class': class_name,
                    'confidence': round(confidence, 2),
                    'bbox': [x1, y1, x2, y2]
                })

                # Draw bounding box
                color = self._get_color(class_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                label = f'{class_name} {confidence:.0%}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
                cv2.putText(frame, label, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # HUD — object count
        count_text = f'Detected: {len(detections)} object(s)'
        cv2.putText(frame, count_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Publish detections as JSON string
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.det_pub.publish(det_msg)

        # Publish annotated image back to ROS
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.img_pub.publish(annotated_msg)

        # Optional live window (disable in headless environments)
        if self.show:
            cv2.imshow('YOLOv8 Object Detection', frame)
            cv2.waitKey(1)

        if detections:
            names = [d['class'] for d in detections]
            self.get_logger().info(f'Detected: {names}')

    def _get_color(self, class_id: int):
        """Return a consistent BGR color per class."""
        colors = [
            (255, 56,  56),  (255, 157, 151), (255, 112, 31),
            (255, 178, 29),  (207, 210, 49),  (72,  249, 10),
            (146, 204, 23),  (61,  219, 134), (26,  147, 52),
            (0,   212, 187), (44,  153, 168), (0,   194, 255),
            (52,  69,  147), (100, 115, 255), (0,   24,  236),
            (132, 56,  255), (82,  0,   133), (203, 56,  255),
            (255, 149, 200), (255, 55,  199),
        ]
        return colors[class_id % len(colors)]

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
