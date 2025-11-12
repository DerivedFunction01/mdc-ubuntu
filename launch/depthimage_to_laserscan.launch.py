import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments (defaults match Azure Kinect ROS Driver topic names)
    ld.add_action(DeclareLaunchArgument(
        'depth_image_topic', default_value='depth/image_raw',
        description='Depth image topic (raw or rectified)'))
    ld.add_action(DeclareLaunchArgument(
        'camera_info_topic', default_value='depth/camera_info',
        description='Camera info topic for depth camera'))
    ld.add_action(DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='Output LaserScan topic'))
    ld.add_action(DeclareLaunchArgument(
        'output_frame', default_value='camera_base',
        description='Frame id for published LaserScan (should match nav2 robot_base_frame)'))
    ld.add_action(DeclareLaunchArgument(
        'range_min', default_value='0.35',
        description='Minimum valid range for LaserScan (m)'))
    ld.add_action(DeclareLaunchArgument(
        'range_max', default_value='8.0',
        description='Maximum valid range for LaserScan (m)'))
    ld.add_action(DeclareLaunchArgument(
        'scan_height', default_value='10',
        description='Scan row (in pixels) or height index to sample from depth image'))
    ld.add_action(DeclareLaunchArgument(
        'output_window_step', default_value='4',
        description='Downsample step in horizontal image pixels to reduce CPU'))

    params = {
        # The parameter names below are commonly used by depthimage_to_laserscan ports.
        # If your installed version differs, adjust names accordingly.
        'depth_image_topic': LaunchConfiguration('depth_image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'scan_topic': LaunchConfiguration('scan_topic'),
        'output_frame': LaunchConfiguration('output_frame'),
        'range_min': LaunchConfiguration('range_min'),
        'range_max': LaunchConfiguration('range_max'),
        'scan_height': LaunchConfiguration('scan_height'),
        'output_window_step': LaunchConfiguration('output_window_step'),
        # angular defaults (full FOV by default)
        'min_angle': -3.14159,
        'max_angle': 3.14159,
        # optional: tune these if your node supports them
        'use_inf': False,
    }

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[params],
        remappings=[
            # remap the typical internal topic names to your actual topics
            ('depth/image', LaunchConfiguration('depth_image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )

    ld.add_action(depth_to_scan_node)

    return ld
