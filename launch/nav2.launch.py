import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default nav2 params YAML inside this package's config folder
default_nav2_params = os.path.join(os.getcwd(), "config", "nav2_params_2d_scan.yaml")

# --- LIFECYCLE NODE MANAGEMENT ---
# The order here matters!
# 1. map_server (must load map first)
# 2. amcl (needs map to localize)
# 3. servers (planner/controller need to know where robot is)
lifecycle_nodes = [
    "map_server",
    "amcl",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]

# --- Configured params substitution for Nav2 nodes ---
configured_params = [LaunchConfiguration("params_file")]


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
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
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
            "params_file",
            default_value=default_nav2_params,
            description="Full path to the nav2 params yaml file to use",
        )
    )

    # --- Nav2 nodes ---
    nav_nodes = [
        # Map Server (Loads the static map)
        {
            "package": "nav2_map_server",
            "executable": "map_server",
            "name": "map_server",
        },
        # AMCL (Localization)
        {
            "package": "nav2_amcl",
            "executable": "amcl",
            "name": "amcl",
        },
        # Controller (Follows the path)
        {
            "package": "nav2_controller",
            "executable": "controller_server",
            "name": "controller_server",
        },
        # Smoother (Optional, smooths path)
        {
            "package": "nav2_smoother",
            "executable": "smoother_server",
            "name": "smoother_server",
        },
        # Planner (Calculates the path)
        {
            "package": "nav2_planner",
            "executable": "planner_server",
            "name": "planner_server",
        },
        # Behaviors (Recovery actions like backing up/spinning)
        {
            "package": "nav2_behaviors",
            "executable": "behavior_server",
            "name": "behavior_server",
        },
        # BT Navigator (Orchestrates the navigation logic)
        {
            "package": "nav2_bt_navigator",
            "executable": "bt_navigator",
            "name": "bt_navigator",
        },
        {
            "package": "nav2_waypoint_follower",
            "executable": "waypoint_follower",
            "name": "waypoint_follower",
        },
    ]

    for nd in nav_nodes:
        ld.add_action(
            Node(
                package=nd["package"],
                executable=nd["executable"],
                name=nd["name"],
                output="screen",
                respawn=LaunchConfiguration("use_respawn"),
                respawn_delay=1.0,
                parameters=configured_params,
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                # If you need to remap cmd_vel to something else (e.g. /robot/cmd_vel), do it here
                remappings=[("cmd_vel", "cmd_vel")],
            )
        )

    # lifecycle manager for nav2
    ld.add_action(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"autostart": LaunchConfiguration("autostart")},
                {"node_names": lifecycle_nodes},
            ],
        )
    )

    return ld
