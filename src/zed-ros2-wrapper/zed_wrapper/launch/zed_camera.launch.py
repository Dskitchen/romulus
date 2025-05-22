# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common'
)

# FFMPEG Configuration to be loaded by ZED Node
default_config_ffmpeg = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'ffmpeg.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def parse_array_param(param):
    str_val = param.replace('[', '')
    str_val = str_val.replace(']', '')
    arr = str_val.split(',')
    return arr


def launch_setup(context, *args, **kwargs):
    return_array = []

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    sim_address = LaunchConfiguration('sim_address')
    sim_port = LaunchConfiguration('sim_port')

    stream_address = LaunchConfiguration('stream_address')
    stream_port = LaunchConfiguration('stream_port')

    container_name_lc = LaunchConfiguration('container_name') # Renamed to avoid conflict
    namespace_lc = LaunchConfiguration('namespace') # Renamed
    camera_name_lc = LaunchConfiguration('camera_name') # Renamed
    camera_model_lc = LaunchConfiguration('camera_model') # Renamed

    node_name_lc = LaunchConfiguration('node_name') # Renamed

    config_common_path_cfg = LaunchConfiguration('config_path')
    ffmpeg_config_path_lc = LaunchConfiguration('ffmpeg_config_path') # Renamed

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path_cfg = LaunchConfiguration('xacro_path')

    custom_baseline = LaunchConfiguration('custom_baseline')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    enable_gnss = LaunchConfiguration('enable_gnss')
    gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')

    resolution_launch_arg = LaunchConfiguration('resolution')

    # Perform context evaluation for launch configurations
    container_name_val = container_name_lc.perform(context)
    namespace_val = namespace_lc.perform(context)
    camera_name_val = camera_name_lc.perform(context)
    camera_model_val = camera_model_lc.perform(context)
    node_name_val = node_name_lc.perform(context)
    enable_gnss_val = enable_gnss.perform(context)
    gnss_coords_str = gnss_antenna_offset.perform(context)
    gnss_coords = parse_array_param(gnss_coords_str) if gnss_coords_str and gnss_coords_str != '[]' else []
    
    custom_baseline_val = custom_baseline.perform(context)
    
    resolution_input_str = resolution_launch_arg.perform(context)
    resolution_sdk_string = ''
    if resolution_input_str == '0':
        resolution_sdk_string = 'VGA'
    elif resolution_input_str == '1':
        resolution_sdk_string = 'HD720'
    elif resolution_input_str == '2':
        resolution_sdk_string = 'HD1080'
    else:
        return_array.append(LogInfo(msg=f"Warning: Unexpected 'resolution' value '{resolution_input_str}'. Defaulting to 'VGA' for SDK."))
        resolution_sdk_string = 'VGA' 

    if camera_name_val == '':
        camera_name_val = 'zed'

    if camera_model_val == 'virtual' and float(custom_baseline_val) <= 0:
        return [
            LogInfo(msg="Please set a positive value for the 'custom_baseline' argument when using a 'virtual' Stereo Camera with two ZED X One devices."),
        ]
    
    if namespace_val == '': # If namespace is not provided, use camera_name as namespace
        namespace_val = camera_name_val
    # If namespace IS provided, node_name_val (default 'zed_node') is used as is within that namespace.
    # The original logic for node_name_val potentially being overridden by camera_name_val if namespace_val was set
    # has been removed to align with typical ROS conventions where node_name is distinct within a namespace.
    # The original file's logic:
    # if namespace_val == '':
    #   namespace_val = camera_name_val
    # else:
    #   node_name_val = camera_name_val <--- This part changed; node_name is now independent if namespace is set.

    config_common_path_val = config_common_path_cfg.perform(context)
    if config_common_path_val == '':
        if (camera_model_val == 'zed' or 
            camera_model_val == 'zedm' or 
            camera_model_val == 'zed2' or 
            camera_model_val == 'zed2i' or 
            camera_model_val == 'zedx' or 
            camera_model_val == 'zedxm' or
            camera_model_val == 'virtual'):
            config_common_path_val = default_config_common + '_stereo.yaml'
        else: 
            config_common_path_val = default_config_common + '_mono.yaml'

    print('Using common configuration file: ' + config_common_path_val)

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )
    
    ffmpeg_config_path_val = ffmpeg_config_path_lc.perform(context)


    xacro_command_list = ['xacro', ' ']
    xacro_command_list.append(xacro_path_cfg.perform(context))
    xacro_command_list.append(' ')
    xacro_command_list.append('camera_name:=')
    xacro_command_list.append(camera_name_val)
    xacro_command_list.append(' ')
    xacro_command_list.append('camera_model:=')
    xacro_command_list.append(camera_model_val)
    xacro_command_list.append(' ')
    xacro_command_list.append('custom_baseline:=')
    xacro_command_list.append(custom_baseline_val)   
    if enable_gnss_val == 'true':
        xacro_command_list.append(' ')
        xacro_command_list.append('enable_gnss:=true')
        xacro_command_list.append(' ')
        if len(gnss_coords) == 3:
            xacro_command_list.append('gnss_x:=')
            xacro_command_list.append(gnss_coords[0])
            xacro_command_list.append(' ')
            xacro_command_list.append('gnss_y:=')
            xacro_command_list.append(gnss_coords[1])
            xacro_command_list.append(' ')
            xacro_command_list.append('gnss_z:=')
            xacro_command_list.append(gnss_coords[2])
            xacro_command_list.append(' ')
    
    robot_description_command = Command(xacro_command_list)

    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description_command
        }]
    )
    return_array.append(rsp_node)

    if container_name_val == '':
        container_name_val = 'zed_container' 
        distro = os.environ.get('ROS_DISTRO', 'unknown')
        if distro == 'foxy':
            container_exec = 'component_container'
        else:
            container_exec = 'component_container_isolated'
        
        zed_container = ComposableNodeContainer(
            name=container_name_val,
            namespace=namespace_val,
            package='rclcpp_components',
            executable=container_exec,
            arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'], # Corrected
            output='screen',
        )
        return_array.append(zed_container)

    node_parameters = [
        config_common_path_val,
        config_camera_path,
        ffmpeg_config_path_val,
        {
            'use_sim_time': use_sim_time,
            'simulation.sim_enabled': sim_mode,
            'simulation.sim_address': sim_address,
            'simulation.sim_port': sim_port,
            'stream.stream_address': stream_address,
            'stream.stream_port': stream_port,
            'general.camera_name': camera_name_val, # This sets the camera_name for the node
            'general.camera_model': camera_model_val,
            'svo.svo_path': svo_path,
            'general.serial_number': serial_number,
            'general.grab_resolution': resolution_sdk_string,
            'pos_tracking.publish_tf': publish_tf,
            'pos_tracking.publish_map_tf': publish_map_tf,
            'sensors.publish_imu_tf': publish_imu_tf,
            'gnss_fusion.gnss_fusion_enabled': enable_gnss,
        }
    ]
    
    ros_params_override_path_val = ros_params_override_path.perform(context)
    if ros_params_override_path_val != '':
        node_parameters.append(ros_params_override_path_val)

    return_array.append(LogInfo(msg=f"Launch argument 'resolution' set to: '{resolution_input_str}', mapped to SDK value: '{resolution_sdk_string}' for general.grab_resolution"))

    zed_wrapper_plugin_name = ''
    if (camera_model_val == 'zed' or
        camera_model_val == 'zedm' or
        camera_model_val == 'zed2' or
        camera_model_val == 'zed2i' or
        camera_model_val == 'zedx' or
        camera_model_val == 'zedxm' or
        camera_model_val == 'virtual'):
        zed_wrapper_plugin_name = 'stereolabs::ZedCamera'
    else: 
        zed_wrapper_plugin_name = 'stereolabs::ZedCameraOne'

    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=namespace_val,
        plugin=zed_wrapper_plugin_name,
        name=node_name_val,
        parameters=node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}] # This is correct for ComposableNode
    )
    
    # Correct full_target_container_name construction
    if namespace_val: # If namespace is like "zed"
      if container_name_val.startswith('/'):
          full_target_container_name = container_name_val # Already fully qualified
      elif namespace_val.startswith('/'):
          full_target_container_name = namespace_val + '/' + container_name_val
      else:
          full_target_container_name = '/' + namespace_val + '/' + container_name_val
    else: # If namespace is empty (global)
      if container_name_val.startswith('/'):
          full_target_container_name = container_name_val
      else:
          full_target_container_name = '/' + container_name_val


    info_msg = f"* Loading ZED node '{node_name_val}' (plugin: {zed_wrapper_plugin_name}) in container '{full_target_container_name}'"
    return_array.append(LogInfo(msg=TextSubstitution(text=info_msg)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_target_container_name, 
        composable_node_descriptions=[zed_wrapper_component]
    )
    return_array.append(load_composable_node)

    return return_array


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model. If `namespace` is not set, this will be used as the namespace.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'container_name',
                default_value='', 
                description='The name of the container for the ZED component. If empty, a new container "zed_container" is created within the effective namespace.'),
            DeclareLaunchArgument(
                'namespace',
                default_value='', 
                description='The namespace for all nodes and the container. If empty, it defaults to `camera_name`.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the ZED component node itself (e.g., zed_node).'),
            DeclareLaunchArgument(
                'config_path',
                default_value='', 
                description='Path to common YAML configuration. Defaults to `common_stereo.yaml` or `common_mono.yaml` based on model.'),
            DeclareLaunchArgument(
                'ffmpeg_config_path',
                default_value=TextSubstitution(text=default_config_ffmpeg),
                description='Path to FFMPEG parameters YAML for FFMPEG image transport.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='Camera serial number. "0" for first available. Integer expected by SDK for non-zero SNs.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and Robot State Publisher for static TFs.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable `odom -> <camera_link_frame>` TF publication.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='true',
                description='Enable `map -> odom` TF publication. (Requires `publish_tf`=true).',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='true',
                description='Enable IMU TF publication. (Requires `publish_tf`=true).',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF xacro file.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='Path to an additional parameters file to override defaults.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'), 
                description='Path to SVO file or "live" for live camera stream.'),
            DeclareLaunchArgument(
                'enable_gnss',
                default_value='false',
                description='Enable GNSS fusion with `sensor_msgs::msg::NavSatFix` messages.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'gnss_antenna_offset',
                default_value='[]', 
                description='GNSS antenna position relative to ZED camera. Format: "[x,y,z]" as string.'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated clock from `/clock` topic.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_mode',
                default_value='false',
                description='Enable simulation mode. Configure `sim_address` and `sim_port`.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'sim_address',
                default_value='127.0.0.1',
                description='Simulation server connection address.'),
            DeclareLaunchArgument(
                'sim_port',
                default_value='30000',
                description='Simulation server connection port.'),
            DeclareLaunchArgument(
                'stream_address',
                default_value='',
                description='Input streaming server connection address.'),
            DeclareLaunchArgument(
                'stream_port',
                default_value='30000',
                description='Input streaming server connection port.'),
            DeclareLaunchArgument(
                'custom_baseline',
                default_value='0.0',
                description='For ZED X One custom stereo rig: distance between camera centers (meters).'),
            DeclareLaunchArgument(
                'resolution',
                default_value='0',
                description="Resolution: '0' (VGA), '1' (HD720), '2' (HD1080). Mapped to SDK string."),
            OpaqueFunction(function=launch_setup)
        ]
    )
