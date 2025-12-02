import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # --- RTAB-Map Parameters for Visual Odometry Only ---
    # We strip out all the mapping/database parameters
    rtabmap_odom_parameters = {
        "frame_id": "camera_base",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "use_sim_time": False,  # Set to false for real hardware
        "approx_sync": True,
        "approx_sync_max_interval": 0.04,
        "qos_image": 2,
        "qos_imu": 2,
        # ODOMETRY TUNING
        "Odom/Strategy": "0",  # 0=Frame-to-Frame (Fast), 1=Frame-to-Map (Robust)
        "Odom/ResetCountdown": "1",  # Auto-reset odom if lost (critical for AMCL recovery)
        "Vis/MinInliers": "15",  # Min feature points to track movement
        "Vis/FeatureType": "8",  # 8=ORB (Fast), 6=GFTT/BRIEF (Robust)
        # Publish the critical TF: odom -> camera_base
        "publish_tf": True,
    }

    # Remappings for Azure Kinect
    rtabmap_remapping = [
        ("rgb/image", "/rgb/image_raw"),
        ("rgb/camera_info", "/rgb/camera_info"),
        ("depth/image", "/depth_to_rgb/image_raw"),
        # Output odometry topic
        ("odom", "/odom"),
    ]

    # --- Node: Only Visual Odometry ---
    # This provides the "odom" -> "camera_base" transform AMCL is waiting for
    visual_odometry_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        output="screen",
        parameters=[rtabmap_odom_parameters],
        remappings=rtabmap_remapping,
    )

    ld.add_action(visual_odometry_node)

    return ld
