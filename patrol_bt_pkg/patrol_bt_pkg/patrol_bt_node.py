#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class PatrolBTNode(Node):

    def __init__(self):
        super().__init__('patrol_bt_node')
        self.get_logger().info("Starting Patrol BT Manager")

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Get the absolute path to our custom BT XML
        pkg_share = get_package_share_directory('patrol_bt_pkg')
        self.bt_xml = os.path.join(pkg_share, 'behavior_trees', 'patrol_navigation.xml')
        self.get_logger().info(f"Using BT: {self.bt_xml}")

        self.waypoints = self.create_waypoints()
        self.current_index = 0
        self.is_navigating = False
        self.retry_count = 0
        self.max_retries = 3

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Patrol BT Manager Ready")

    def create_waypoints(self):
        coords = [
            (0.5, 0.0),
            (3.5, 0.5),
            (0.0, 0.5),
            (0.0, 0.0)
        ]
        points = []
        for x, y in coords:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            points.append(pose)
        return points

    def control_loop(self):
        if not self.is_navigating:
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✓ Reached waypoint {self.current_index}")
            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.retry_count = 0  # reset retries on success

        elif result == TaskResult.FAILED:
            self.retry_count += 1
            self.get_logger().warn(
                f"✗ Navigation failed at waypoint {self.current_index} "
                f"(attempt {self.retry_count}/{self.max_retries})"
            )
            if self.retry_count >= self.max_retries:
                self.get_logger().error(
                    f"Max retries hit, skipping waypoint {self.current_index}"
                )
                self.current_index = (self.current_index + 1) % len(self.waypoints)
                self.retry_count = 0

        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Navigation was canceled, moving on")
            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.retry_count = 0

        self.send_next_goal()

    def send_next_goal(self):
        goal = self.waypoints[self.current_index]
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"→ Going to waypoint {self.current_index}: "
            f"({goal.pose.position.x}, {goal.pose.position.y})"
        )

        self.is_navigating = True

        # This is where we actually USE the BT
        self.navigator.goToPose(goal, behavior_tree=self.bt_xml)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolBTNode()
    node.send_next_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()