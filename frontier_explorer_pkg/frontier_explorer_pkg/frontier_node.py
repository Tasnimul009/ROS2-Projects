import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FrontierMonitor(Node):
    def __init__(self):
        super().__init__('frontier_monitor')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info('Frontier monitor node started')

    def map_callback(self, msg):
        total_cells = len(msg.data)
        explored = sum(1 for c in msg.data if c == 0)
        unknown = sum(1 for c in msg.data if c == -1)
        occupied = sum(1 for c in msg.data if c == 100)

        pct = (explored / total_cells) * 100 if total_cells > 0 else 0
        self.get_logger().info(
            f'Map: {pct:.1f}% explored | '
            f'Free: {explored} | Unknown: {unknown} | Occupied: {occupied}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()