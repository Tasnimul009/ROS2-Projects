import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Gazebo world with a camera-equipped robot ──────────────────────────
    # Swap this for your own world/robot launch if you already have one.
    # This example uses the standard turtlebot3_world.
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch', 'turtlebot3_world.launch.py'
            )
        ])
    )

    # ── Object Detection Node ──────────────────────────────────────────────
    detection_node = Node(
        package='object_detection',
        executable='detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[{
            'model': 'yolov8n.pt',       # nano — fastest, CPU-friendly
            'confidence': 0.50,
            'camera_topic': '/camera/image_raw',
            'show_window': True,
        }]
    )

    # ── RViz2 — view annotated image ───────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('object_detection'),
            'config', 'detection.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        turtlebot3_gazebo,
        detection_node,
        # rviz_node,   # uncomment if you want RViz2 alongside
    ])
