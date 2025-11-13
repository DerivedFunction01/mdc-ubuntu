import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Create the depthimage_to_laserscan node
    # This node converts depth images from sensors (like Azure Kinect) into laser scan data
    # for use with navigation stacks like Nav2
    depth_to_scan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",  # Show all node output in terminal for debugging
        # Node parameters - tune these based on your sensor and environment
        parameters=[
            {
                "range_min": 0.15,  # Minimum valid range in meters (closer than this is ignored)
                "range_max": 8.0,  # Maximum valid range in meters (further than this is ignored)
                "scan_height": 240,  # Which row of the depth image to extract (0=top, increase for lower rows)
                "output_window_step": 8,  # Downsample horizontal pixels (higher = fewer points, less CPU)
            }
        ],
        # Topic remappings - CRITICAL: these must match the node's internal topic names
        # The node internally looks for /depth and /depth_camera_info by default
        # We remap these to match where your sensor publishes (e.g., Azure Kinect publishes to /depth/image_raw)
        remappings=[
            # Depth image: node expects /depth, Kinect publishes to /depth/image_raw
            ("/depth", "/depth/image_raw"),
            # Camera calibration info: node expects /depth_camera_info, Kinect publishes to /depth/camera_info
            ("/depth_camera_info", "/depth/camera_info"),
            # Output scan topic: node publishes here
            ("/scan", "/scan"),
        ],
    )

    ld.add_action(depth_to_scan_node)

    return ld
