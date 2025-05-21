from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'zed'
        }.items()
    )

    return LaunchDescription([
        zed_wrapper_launch,
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'max_surface_angle': 60.0,
                'min_surface_area': 0.002,
                'max_surface_area': 1.5,
                'plane_distance_threshold': 0.03,
                'camera_resolution': 'SVGA'
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
