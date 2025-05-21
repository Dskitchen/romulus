from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_robot_driver',
            executable='ur_control',
            name='ur_control',
            output='screen',
            parameters=[{'ur_type': 'ur5e', 'robot_ip': '192.168.56.101'}]
        ),
        Node(
            package='control',
            executable='control_node.py',
            name='control_node',
            output='screen'
        )
    ])
