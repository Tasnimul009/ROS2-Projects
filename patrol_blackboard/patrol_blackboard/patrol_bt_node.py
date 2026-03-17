import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import py_trees
import json

#check check 
class PatrolBTNode(Node):
    def __init__(self):
        super().__init__('patrol_bt_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.bt_xml = "/home/tasz/ros2_ws/src/patrol_blackboard/behavior_trees/patrol_blackboard.xml"

        # ── Blackboard setup ──────────────────────────────────────────────
        # Create a blackboard client for this node
        # Think of it as: this node gets a "pen" to write on the shared whiteboard
        self.blackboard = py_trees.blackboard.Client(name="PatrolNode")
        self.blackboard.register_key("waypoints", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("waypoints", access=py_trees.common.Access.READ)

        # Write the default hardcoded waypoints onto the blackboard at startup
        # After this, the node always reads from the blackboard — never from a hardcoded list
        self.blackboard.waypoints = self.create_waypoints()
        self.get_logger().info("Default waypoints written to blackboard")

        # ── State ─────────────────────────────────────────────────────────
        self.current_index = 0
        self.is_navigating = False
        self.retry_count = 0
        self.max_retries = 3

        # ── Subscriber ────────────────────────────────────────────────────
        # This listens on /patrol_waypoints topic for new waypoint lists
        # Message format (JSON string): "[[x1,y1],[x2,y2],[x3,y3]]"
        # Example: ros2 topic pub /patrol_waypoints std_msgs/String 'data: "[[1.0,0.0],[2.0,1.0]]"'
        self.waypoint_sub = self.create_subscription(
            String,
            '/patrol_waypoints',
            self.waypoint_callback,
            10
        )
        self.get_logger().info("Listening for new waypoints on /patrol_waypoints")

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Patrol BT Manager Ready")

    # ── Default waypoints (used only at startup) ──────────────────────────
    def create_waypoints(self):
        coords = [
            (0.5, 0.0),
            (3.5, 0.5),
            (0.0, 0.5),
            (0.0, 0.0)
        ]
        return self._coords_to_poses(coords)

    # ── Waypoint topic callback ───────────────────────────────────────────
    def waypoint_callback(self, msg: String):
        """
        Called whenever someone publishes new waypoints on /patrol_waypoints.
        Parses the JSON, converts to PoseStamped list, writes to blackboard.
        The robot will use these AFTER finishing its current waypoint.
        """
        try:
            coords = json.loads(msg.data)  # expects [[x1,y1],[x2,y2],...]

            if not isinstance(coords, list) or len(coords) == 0:
                self.get_logger().warn("Received empty or invalid waypoint list, ignoring")
                return

            new_waypoints = self._coords_to_poses(coords)

            # Write new waypoints to the blackboard
            self.blackboard.waypoints = new_waypoints

            # Reset index so patrol starts from waypoint 0 of the new list
            # (after current navigation finishes — control_loop handles that)
            self.current_index = 0
            self.retry_count = 0

            self.get_logger().info(
                f"Blackboard updated: {len(new_waypoints)} new waypoints received. "
                f"Will use after current waypoint completes."
            )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse waypoint message: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in waypoint_callback: {e}")

    # ── Helper: convert (x,y) list → PoseStamped list ─────────────────────
    def _coords_to_poses(self, coords):
        points = []
        for item in coords:
            x, y = item[0], item[1]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            points.append(pose)
        return points

    # ── Control loop (runs every 1 second) ────────────────────────────────
    def control_loop(self):
        if not self.is_navigating:
            return

        if not self.navigator.isTaskComplete():
            return  # still navigating, wait

        result = self.navigator.getResult()

        # Read current waypoints FROM the blackboard (not a hardcoded variable)
        waypoints = self.blackboard.waypoints

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✓ Reached waypoint {self.current_index}")
            self.current_index = (self.current_index + 1) % len(waypoints)
            self.retry_count = 0

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
                self.current_index = (self.current_index + 1) % len(waypoints)
                self.retry_count = 0

        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Navigation was canceled, moving on")
            self.current_index = (self.current_index + 1) % len(waypoints)
            self.retry_count = 0

        self.send_next_goal()

    # ── Send next goal using waypoints from blackboard ────────────────────
    def send_next_goal(self):
        # Always read from blackboard — this is the key change from before
        waypoints = self.blackboard.waypoints

        goal = waypoints[self.current_index]
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"→ Going to waypoint {self.current_index}: "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )

        self.is_navigating = True
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
