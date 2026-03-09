#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.get_logger().info("Starting Patrol Manager...")

        # -------------------------
        # Navigator
        # -------------------------
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # -------------------------
        # Waypoints
        # -------------------------
        self.waypoints = self.create_waypoints()
        self.current_index = 0

        # -------------------------
        # State Machine
        # -------------------------
        self.state = "IDLE"
        self.retry_count = 0
        self.max_retries = 2

        # -------------------------
        # Main Control Loop
        # -------------------------
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info("Patrol Manager Ready.")

    # --------------------------------------------------
    # Create Waypoints
    # --------------------------------------------------
    def create_waypoints(self):
        points = []

        coords = [
            (3.5, -0.5),
            (3.5, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]

        for x, y in coords:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y

            # Facing forward (no rotation)
            pose.pose.orientation.w = 1.0

            points.append(pose)

        return points

    # --------------------------------------------------
    # State Machine Control Loop
    # --------------------------------------------------
    def control_loop(self):

        if self.state == "IDLE":
            self.start_next_waypoint()
            return

        if self.state == "MOVING":

            # If still navigating → do nothing
            if not self.navigator.isTaskComplete():
                return

            # Navigation finished → get result
            result = self.navigator.getResult()
            self.get_logger().info(f"Navigation result: {result}")

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Reached waypoint!")
                self.current_index = (self.current_index + 1) % len(self.waypoints)
                self.retry_count = 0
                self.state = "IDLE"

            elif result == TaskResult.FAILED:
                self.get_logger().warn("Navigation failed!")
                self.state = "FAILED"

            elif result == TaskResult.CANCELED: 
                self.get_logger().warn("Navigation canceled!")
                self.state = "FAILED"

            return

        if self.state == "FAILED":

            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f"Retry attempt {self.retry_count}")
                self.navigator.goToPose(self.waypoints[self.current_index])
                self.state = "MOVING"
            else:
                self.get_logger().error("Max retries reached. Skipping waypoint.")
                self.retry_count = 0
                self.current_index = (self.current_index + 1) % len(self.waypoints)
                self.state = "IDLE"

    # --------------------------------------------------
    # Start Moving to Next Waypoint
    # --------------------------------------------------
    def start_next_waypoint(self):
        goal = self.waypoints[self.current_index]

        # Update timestamp before sending goal
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Moving to waypoint {self.current_index}")
        self.navigator.goToPose(goal)
        self.state = "MOVING"


# ------------------------------------------------------
# Main
# ------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Patrol Manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()