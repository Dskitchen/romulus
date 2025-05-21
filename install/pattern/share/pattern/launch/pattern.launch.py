from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='vision_node.py',
            name='vision_node',
            output='screen',
            parameters=[{
                'max_surface_angle': 45.0,
                'min_surface_area': 0.005,
                'max_surface_area': 1.0,
                'plane_distance_threshold': 0.02
            }]
        ),
        Node(
            package='pattern',
            executable='pattern_node.py',
            name='pattern_node',
            output='screen',
            parameters=[{
                'svg_directory': '/root/Documents/Cake_Drawing',
                'pattern_scale': 0.8
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
