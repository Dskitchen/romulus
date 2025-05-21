from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cake_decorator')
    urdf_file = os.path.join(pkg_share, 'urdf', 'romulus.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='zed_wrapper',
            executable='zed_wrapper',
            name='zed_wrapper',
            output='screen',
            arguments=['--ros-args', '--params-file', '/workspaces/isaac_ros-dev/ros_ws/src/zed-ros2-wrapper/zed_wrapper/config/zedx.yaml']
        ),
        Node(
            package='ur_robot_driver',
            executable='ur_control',
            name='ur_control',
            output='screen',
            parameters=[{'ur_type': 'ur5e', 'robot_ip': '192.168.56.101'}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool0_to_zedx_transform',
            output='screen',
            arguments=['--frame-id', 'tool0', '--child-frame-id', 'zed2i_left_camera_optical_frame',
                       '--x', '-0.065', '--y', '-0.016', '--z', '0.0175',
                       '--roll', '3.14159', '--pitch', '-1.5708', '--yaw', '3.14159']
        ),
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
                'svg_directory': os.path.expanduser('~/decoration_patterns'),
                'pattern_scale': 0.8
            }]
        ),
        Node(
            package='control',
            executable='control_node.py',
            name='control_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
