# Using the 2D LaserScan Nav2 Configuration

This guide explains how to use the new 2D LaserScan-based Nav2 parameters instead of the RGB-D approach.

## Files

- **`config/nav2_params_2d_scan.yaml`** — Nav2 parameters optimized for 2D LaserScan localization with AMCL.
- **`config/nav2_params.yaml`** — Original RGB-D/RTAB-Map based configuration (kept for reference).

## Key Changes from RGB-D to 2D LaserScan

### What's New
1. **AMCL Section** — Added full Adaptive Monte Carlo Localization configuration:
   - Uses `/scan` topic for 2D particle filter localization
   - Configured with reasonable defaults for Kinect depth-derived scans
   - `max_beams: 60` (tunable for CPU vs accuracy)
   - `laser_model_type: "likelihood_field"` (robust to noise)

2. **LaserScan Obstacle Layer** — Replaced pointcloud with LaserScan:
   - Much lighter computationally
   - Uses `data_type: LaserScan` instead of `PointCloud2`
   - Configured in both `local_costmap` and `global_costmap`

### What's Removed
- RGB-D visual odometry / SLAM components
- Pointcloud (`/cloud_obstacles`) dependency

### What's Kept
- All behavior server, planner, and controller configs
- Velocity smoother, waypoint follower, etc.

## Usage

### Option 1: Use the new config with depthimage_to_laserscan
```bash
# Terminal 1: Start Kinect driver
ros2 launch azure_kinect_ros_driver driver.launch.py

# Terminal 2: Convert depth to 2D scan
ros2 launch launch/depthimage_to_laserscan.launch.py

# Terminal 3: Launch Nav2 with 2D config
ros2 launch launch/nav2.launch.py params_file:=$(pwd)/config/nav2_params_2d_scan.yaml
```

### Option 2: Update nav2.launch.py to use 2d config by default
Edit `launch/nav2.launch.py` and change the default:
```python
default_nav2_params = os.path.join(os.getcwd(), "config", "nav2_params_2d_scan.yaml")
```

## AMCL Tuning Parameters

If localization is noisy or drifts:

### Too many particles diverging?
- Increase `sigma_hit` (less trust in scans): `0.3` to `0.5`
- Decrease `z_hit` (reduce good measurement weight): `0.9` to `0.8`
- Reduce `max_beams` to `30` to filter noise

### Particles not converging?
- Decrease `sigma_hit` (more trust): `0.1` to `0.15`
- Increase `z_hit` (more weight on good measurements): `0.95` to `0.99`
- Increase `max_particles` to `3000`

### Drifting over time?
- Tune odometry model parameters (`alpha1-5`) to match your robot's motion model
- Decrease `update_min_d` / `update_min_a` to update AMCL more frequently

## Scan-Specific Tuning

The AMCL config assumes:
- Scan min range: `0.35 m` (matches depth_to_scan_params.yaml)
- Scan max range: `8.0 m` (matches depth_to_scan_params.yaml)
- Scan height: middle of Kinect image (adjust in depth_to_scan_params.yaml)

If you adjust `scan_height`, `range_min`, or `range_max` in `depth_to_scan_params.yaml`, ensure consistency with AMCL's `laser_min_range`, `laser_max_range`, etc.

## Testing Localization

1. **RViz2 Visualization**:
   ```bash
   ros2 run rviz2 rviz2
   ```
   - Add `PoseWithCovarianceStamped` for AMCL pose estimate
   - Add `LaserScan` for /scan display
   - Watch particles track your motion

2. **Publish initial pose**:
   ```bash
   # Use RViz2 "2D Pose Estimate" button, or:
   ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
     "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}}}}"
   ```

3. **Check /amcl_pose topic**:
   ```bash
   ros2 topic echo /amcl_pose
   ```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| AMCL particles diverge immediately | Check `/scan` is publishing and in correct frame (`camera_base`); verify TF between camera_base and odom |
| Localization drifts over time | Tune `sigma_hit` down (more trust in scan); adjust odometry alphas |
| No map or costmap appears | Ensure `/map` topic is published (from map_server or SLAM backend) |
| Slow performance | Reduce `max_beams`, `max_particles`, or `update_frequency` in costmaps |

## Migrating Back to RGB-D

If you want to revert:
```bash
ros2 launch launch/nav2.launch.py params_file:=$(pwd)/config/nav2_params.yaml
```

This uses the original pointcloud-based approach but is more computationally expensive.

## References

- [Nav2 AMCL documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [depthimage_to_laserscan wiki](https://wiki.ros.org/depthimage_to_laserscan)
- [Azure Kinect ROS Driver docs](../Azure_Kinect_ROS_Driver/docs/)
