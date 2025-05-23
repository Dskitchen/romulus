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
                'max_surface_area': 0.05,  # Updated to match pattern.launch.py
                'plane_distance_threshold': 0.02,  # Updated to match pattern.launch.py
                'camera_resolution': 'SVGA',
                # Updated for current vision_node.py (cake detection)
                'cake_hue_min': 10,
                'cake_hue_max': 30,
                'cake_sat_min': 50,
                'cake_sat_max': 255,
                'cake_val_min': 50,
                'cake_val_max': 255
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
