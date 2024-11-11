import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    package_name = 'insta360_ros2_driver'
    config_file = os.path.join(get_package_share_directory(package_name), 'config', 'config.yaml')
    
    # Declare launch configurations
    launch_args = {
        'compress': ('true', 'Enable compression'),
        'undistort': ('false', 'Enable undistortion'),
        'debug': ('false', 'Enable debug mode'),
        'get_images': ('false', 'Get image data'),
        'config': ('config.yaml', 'Configuration file')
    }
    
    # Create LaunchConfiguration objects
    launch_configs = {
        key: LaunchConfiguration(key, default=value[0])
        for key, value in launch_args.items()
    }

    def create_debug_node(package, executable, name, args=None, prefix=None):
        """Helper function to create debug nodes with consistent configuration"""
        node_args = {
            'package': package,
            'executable': executable,
            'name': name,
            'output': 'screen',
            'condition': IfCondition(launch_configs['debug'])
        }
        if args:
            node_args['arguments'] = [args]
        if prefix:
            node_args['prefix'] = prefix
        return Node(**node_args)

    # Core driver node
    bringup_node = Node(
        package='insta360_ros2_driver',
        executable='insta360_ros2_driver_node',
        name='bringup',
        parameters=[{
            'config_file': launch_configs['config'],
            'undistort': launch_configs['undistort']
        }]
    )

    # Compression enabled group
    compress_group = GroupAction([
        # Live processing node
        Node(
            package='insta360_ros2_driver',
            executable='live_processing.py',
            name='live_processing',
            output='screen',
            condition=IfCondition(launch_configs['compress']),
            parameters=[config_file],
        ),

        # Debug nodes
        create_debug_node(
            'insta360_ros2_driver',
            'live_processing.py',
            'live_processing_debug',
            prefix="xterm -e"
        ),

        create_debug_node(
            'rqt_image_view',
            'rqt_image_view',
            'rqt_image_view',
            args='/back_camera_image/compressed'
        ),

        create_debug_node(
            'insta360_ros2_driver',
            'rostopic_hz.sh',
            'rostopic_hz',
            prefix="xterm -e"
        ),

        # Image capture node
        Node(
            package='insta360_ros2_driver',
            executable='get_images.py',
            name='get_images',
            output='screen',
            parameters=[{'topic': '/back_camera_image/compressed'}],
            condition=IfCondition(launch_configs['get_images'])
        )
    ])

    # No compression group
    no_compress_group = GroupAction([
        create_debug_node(
            'rqt_image_view',
            'rqt_image_view',
            'rqt_image_view_raw',
            args='/insta_image_yuv'
        ),
        
        create_debug_node(
            'insta360_ros2_driver',
            'rostopic_hz_raw.sh',
            'rostopic_hz_raw',
            prefix="xterm -e"
        )
    ])

    # Create launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=value[0],
            description=value[1]
        )
        for name, value in launch_args.items()
    ]

    # Return launch description with all components
    return LaunchDescription(
        launch_arguments +
        [
            LogInfo(msg='Starting Insta360 ROS2 Driver...'),
            bringup_node,
            compress_group,
            no_compress_group
        ]
    )