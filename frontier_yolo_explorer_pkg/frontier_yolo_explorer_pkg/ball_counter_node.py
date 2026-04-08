#!/usr/bin/env python3

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class BallCounterNode(Node):
    def __init__(self):
        super().__init__('ball_counter_node')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_class', 'sports ball')

        detections_topic = (
            self.get_parameter('detections_topic')
            .get_parameter_value()
            .string_value
        )
        self.target_class = (
            self.get_parameter('target_class')
            .get_parameter_value()
            .string_value
        )

        self.pub = self.create_publisher(Int32, '/sports_ball/count', 10)
        self.sub = self.create_subscription(
            String,
            detections_topic,
            self._on_detections,
            10,
        )

        self._last_count: int | None = None
        self.get_logger().info(
            f"Ball counter started. Listening on '{detections_topic}', "
            f"target_class='{self.target_class}'."
        )

    def _on_detections(self, msg: String) -> None:
        count = 0
        try:
            payload: Any = json.loads(msg.data)
            if isinstance(payload, list):
                count = sum(
                    1
                    for d in payload
                    if isinstance(d, dict) and d.get('class') == self.target_class
                )
        except json.JSONDecodeError:
            # Ignore malformed messages.
            return

        out = Int32()
        out.data = int(count)
        self.pub.publish(out)

        if self._last_count != count:
            self.get_logger().info(f"Sports balls detected (current frame): {count}")
            self._last_count = count


def main(args=None):
    rclpy.init(args=args)
    node = BallCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
