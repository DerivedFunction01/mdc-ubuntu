import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# --- Paths ---
azure_kinect_ros_driver_pkg_dir = get_package_share_directory("azure_kinect_ros_driver")
urdf_file_path = os.path.join(
    azure_kinect_ros_driver_pkg_dir, "urdf", "azure_kinect.urdf.xacro"
)


def generate_launch_description():
    ld = LaunchDescription()

    # Common launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "log_level", default_value="info", description="log level"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "FPS",
            default_value="15",  # CHANGED: Increased from 5 to 15 for better Odometry tracking
            description="FPS set to 5, (default) 15, or 30",
        )
    )

    # --- Robot State Publisher ---
    # Publishes the TF tree (base_link -> camera_base -> sensors)
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",  # Added name for clarity
            output="screen",
            parameters=[
                {
                    "robot_description": Command(["xacro ", urdf_file_path]),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    )

    # --- Azure Kinect ROS Driver ---
    ld.add_action(
        Node(
            package="azure_kinect_ros_driver",
            executable="node",
            output="screen",
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                {
                    "depth_enabled": True,
                    "depth_mode": "NFOV_UNBINNED",
                    "color_enabled": True,  # CHANGED: Must be True for Visual Odometry!
                    "color_resolution": "720P",
                    "fps": LaunchConfiguration("FPS"),
                    "point_cloud": False,  # Keep False (saves CPU, we use depthimage_to_laserscan)
                    "rgb_point_cloud": False,
                    "point_cloud_in_depth_frame": False,
                    "synchronized_images_only": True,  # CHANGED: Ensure RGB and Depth match for Odom
                    "imu_rate_target": 100,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
        )
    )

    return ld
