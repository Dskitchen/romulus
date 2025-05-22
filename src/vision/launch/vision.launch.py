from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'max_surface_angle': 95.0,
                'min_surface_area': 0.00005,
                'max_surface_area': 0.3,  # Before we adjusted to 0.05
                'plane_distance_threshold': 0.1,  # Before we adjusted to 0.02
                'camera_resolution': 'SVGA',
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
