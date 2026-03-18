from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('frontier_explorer_pkg'),
        'config', 'slam_toolbox_params.yaml'
    )
    explore_params = os.path.join(
        get_package_share_directory('frontier_explorer_pkg'),
        'config', 'explore_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen'
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params, {'use_sim_time': True}],
        output='screen'
    )

    monitor_node = Node(
        package='frontier_explorer_pkg',
        executable='frontier_node',
        name='frontier_monitor',
        output='screen'
    )

    return LaunchDescription([
        slam_node,
        explore_node,
        monitor_node,
    ])