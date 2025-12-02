import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# --- Interactive DB selection ---
dbname = input("Enter map name: ").strip()
if not dbname:
    dbname = "rtabmap.db"
if not dbname.endswith(".db"):
    dbname += ".db"

# --- Paths ---
azure_kinect_ros_driver_pkg_dir = get_package_share_directory('azure_kinect_ros_driver')
nav2_ros_pkg_dir = get_package_share_directory('nav2_bringup')

urdf_file_path = os.path.join(azure_kinect_ros_driver_pkg_dir, 'urdf', 'azure_kinect.urdf.xacro')
rviz_config_file = os.path.join(os.getcwd(), "config", "nav2_slam.rviz")

save_location = os.path.join(os.getcwd(), "rtabmap_data")
default_db_path = os.path.join(save_location, dbname)
default_nav2_params = os.path.join(os.getcwd(), "config", "nav2_params_2d_scan.yaml")

# Nodes that lifecycle manager will manage
lifecycle_nodes = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower'
]

# --- OPTIMIZED 2D SLAM PARAMETERS ---
rtabmap_parameters = {
    "frame_id": "camera_base",
    "subscribe_depth": True,
    "subscribe_rgb": True,
    "use_sim_time": LaunchConfiguration("use_sim_time"),
    "approx_sync": True,
    "approx_sync_max_interval": 0.05, # Tightened sync for better data quality
    "qos_image": 2,
    "qos_imu": 2,

    # --- 1. ODOMETRY (Visual Tracking) ---
    "Reg/Strategy": "0",       # Visual Odometry
    "Odom/Strategy": "0",      # Frame-to-Frame (Fastest)
    "Vis/MinInliers": "15",    # Minimum points to hold a lock
    "Vis/FeatureType": "8",    # ORB (Fast feature detection)
    "Odom/ResetCountdown": "1", # Auto-reset quickly if lost
    
    # --- 2. FORCING 2D MODE (CPU SAVERS) ---
    "Reg/Force3DoF": "true",   # CRITICAL: Ignores Z, Roll, Pitch. Robot is "flat".
    "Grid/3D": "false",        # CRITICAL: Do not build a 3D Octomap.
    "Map/Cloud2DOnly": "true", # CRITICAL: Only generate 2D map cells, not 3D clouds.
    "Vis/EstimationType": "0", # 0=2D, 1=3D. Hints visualizer to stay 2D.

    # --- 3. MAPPING & GRID GENERATION ---
    "Grid/FromDepth": "true",  # Create 2D grid from depth camera
    "Grid/RangeMax": "4.0",    # Don't map things too far away (noise)
    "Grid/MaxObstacleHeight": "1.0", # Points above this are ignored (ceiling lights, etc)
    "Grid/RayTracing": "true", # Clears empty space (important for navigation)
    "Grid/MapFrameProjection": "true",
    
    # --- 4. PERFORMANCE THROTTLING ---
    "Rtabmap/DetectionRate": "1", # Update map only 1 time per second (Odom runs faster, Map runs slow)
    "Vis/MaxFeatures": "600",     # Cap features to 600 to save CPU (default is 1000)

    # Database
    "database_path": default_db_path,
    "Mem/IncrementalMemory": "true", # "true" = SLAM (Mapping), "false" = Localization only
}

# --- Remapping Kinect topics to RTAB-Map ---
rtabmap_remapping = [
    ('rgb/image', '/rgb/image_raw'),
    ('rgb/camera_info', '/rgb/camera_info'),
    ('depth/image', '/depth_to_rgb/image_raw')
]

configured_params = [LaunchConfiguration('params_file')]

def generate_launch_description():
    ld = LaunchDescription()

    # Common launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_respawn', default_value='False'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=default_nav2_params))

    # --- Robot State Publisher ---
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # --- Azure Kinect ROS Driver ---
    ld.add_action(Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'depth_enabled': True,
            'depth_mode': 'NFOV_UNBINNED',
            'color_enabled': True,
            'color_resolution': '720P',
            # INCREASED FPS: 5 is too slow for Odometry to hold lock. 
            # We increased this to 15, but throttled "Rtabmap/DetectionRate" to 1 above.
            'fps': 15, 
            'point_cloud': False,
            'rgb_point_cloud': False,
            'point_cloud_in_depth_frame': False,
            'synchronized_images_only': True,
            'imu_rate_target': 100,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # --- RTAB-Map Visual Odometry (Calculates Movement) ---
    ld.add_action(Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remapping
    ))

    # --- RTAB-Map SLAM (Builds Map) ---
    ld.add_action(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remapping,
    ))

    # --- RViz2 ---
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    ))

    # --- Nav2 nodes ---
    nav_nodes = [
        {'package': 'nav2_controller', 'executable': 'controller_server', 'name': 'controller_server'},
        {'package': 'nav2_smoother', 'executable': 'smoother_server', 'name': 'smoother_server'},
        {'package': 'nav2_planner', 'executable': 'planner_server', 'name': 'planner_server'},
        {'package': 'nav2_behaviors', 'executable': 'behavior_server', 'name': 'behavior_server'},
        {'package': 'nav2_bt_navigator', 'executable': 'bt_navigator', 'name': 'bt_navigator'},
        {'package': 'nav2_waypoint_follower', 'executable': 'waypoint_follower', 'name': 'waypoint_follower'},
    ]

    for nd in nav_nodes:
        ld.add_action(
            Node(
                package=nd["package"],
                executable=nd["executable"],
                name=nd["name"],
                output="screen",
                respawn=LaunchConfiguration("use_respawn"),
                respawn_delay=2.0,
                parameters=configured_params,
                arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
                remappings=rtabmap_remapping,
            )
        )

    # Lifecycle manager for nav2
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': lifecycle_nodes}
        ]
    ))
    # --- Depth to LaserScan (REQUIRED for Nav2 Local Costmap) ---
    # This converts the depth image into the /scan topic Nav2 is looking for
    ld.add_action(
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            output="screen",
            parameters=[
                {
                    "range_min": 0.35,  # Ignore self-hits
                    "range_max": 8.0,  # Max range
                    "scan_height": 360,  # Middle of image (720p / 2)
                    "output_frame": "camera_base",
                }
            ],
            remappings=[
                ("depth", "/depth_to_rgb/image_raw"),  # Use the aligned depth image
                ("depth_camera_info", "/rgb/camera_info"),
                ("scan", "/scan"),
            ],
        )
    )

    return ld
