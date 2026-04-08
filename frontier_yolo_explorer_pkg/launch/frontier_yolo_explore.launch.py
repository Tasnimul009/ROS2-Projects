from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('frontier_yolo_explorer_pkg')

    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    explore_params = os.path.join(pkg_share, 'config', 'explore_params.yaml')

    # Keep this path explicit so the model is found even when launched from elsewhere.
    yolo_model_path = os.path.expanduser('~/ros2_ws/yolov8n.pt')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params, {'use_sim_time': True}],
        output='screen',
    )

    detection_node = Node(
        package='object_detection',
        executable='detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[{
            'model': yolo_model_path,
            'confidence': 0.5,
            'camera_topic': '/camera/image_raw',
            'show_window': True,
        }],
    )

    return LaunchDescription([
        slam_node,
        explore_node,
        detection_node,
    ])
