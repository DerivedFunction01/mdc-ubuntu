# Azure Kinect Depth-to-LaserScan for Nav2 Localization

This directory contains launch and configuration files to convert the Azure Kinect depth stream into a 2D laser scan for lightweight localization with AMCL and Nav2.

## Why?

RGB-D localization (visual SLAM using full 6-DOF pose estimation from color + depth) is computationally expensive. A 2D laser scan approach is lighter, more stable, and integrates seamlessly with Nav2's standard AMCL localization stack.

## Files

- **`launch/depthimage_to_laserscan.launch.py`** — ROS2 launch file that starts the `depthimage_to_laserscan_node`.
- **`config/depth_to_scan_params.yaml`** — Parameter file with tuning guidance for the Azure Kinect (range, scan height, downsampling).

## Quick Start

### 1. Ensure the Azure Kinect driver is running
```bash
ros2 launch azure_kinect_ros_driver driver.launch.py
```

Verify the driver publishes depth topics:
```bash
ros2 topic list | grep depth
```

You should see:
- `depth/image_raw`
- `depth/camera_info`
- `points2` (pointcloud, optional)

### 2. Launch the depth-to-scan converter
```bash
ros2 launch launch/depthimage_to_laserscan.launch.py
```

Or with custom parameters:
```bash
ros2 launch launch/depthimage_to_laserscan.launch.py \
  depth_image_topic:=depth/image_raw \
  camera_info_topic:=depth/camera_info \
  range_min:=0.35 \
  range_max:=8.0 \
  scan_height:=512
```

### 3. Verify the scan topic is publishing
```bash
ros2 topic echo /scan | head -20
```

You should see `LaserScan` messages with range arrays.

### 4. Launch Nav2 with AMCL
```bash
ros2 launch launch/nav2.launch.py
```

Nav2's AMCL localization will use the `/scan` topic for particle filter localization.

## Tuning

### `scan_height` (most important)
- The Azure Kinect depth sensor outputs a 1024×1024 image (NFOV_UNBINNED mode).
- `scan_height` selects which pixel row is converted to the laser scan (0=top, 1024=bottom).
- **Choose a height that corresponds to a physical plane where obstacles are likely** (e.g., wall height, chair legs).
- If your Kinect is mounted at ~1m height looking forward, try `scan_height=512` (middle) or adjust based on test runs.
- If obstacles are consistently missed or false positives appear, adjust ±50-100 pixels.

### `range_min` / `range_max`
- **`range_min`:** Kinect has near-field noise (reflections, multi-path). Default 0.35m filters most noise. Can reduce to 0.25m if needed but noisier.
- **`range_max`:** Default 8.0m is conservative. Can increase to 10m for larger spaces, but quality drops at distance.

### `output_window_step`
- Downsampling factor to reduce CPU load. 
- `4` means use every 4th column from the depth image (reduces ~75% of pixels).
- Increase to `8` if CPU is high; decrease to `2` if scan resolution is poor.

## Troubleshooting

### No `/scan` topic published
- Check that the Kinect driver is running and publishing `depth/image_raw` and `depth/camera_info`.
- Check node logs: `ros2 node info /depthimage_to_laserscan` and `ros2 node list`.

### AMCL particles diverge or don't localize
- Reduce `scan_height` uncertainty: adjust it to hit a consistent feature (wall, doorway).
- Tune AMCL's laser model parameters in `config/nav2_params.yaml`:
  - `z_hit`: increase if scan matches observations well.
  - `sigma_hit`: decrease (lower values = more trust in scan) if AMCL is unstable.

### Scan looks noisy in RViz
- Increase `range_min` to filter edge noise (Kinect edges are reflective).
- Increase `output_window_step` to reduce spurious pixels.
- Verify `scan_height` isn't sampling the ceiling or floor.

## Next Steps

1. Test with a recorded ROS bag or live Kinect stream.
2. Compare localization results (AMCL with 2D scan) vs. your previous method.
3. If better, migrate from RGB-D SLAM to this 2D AMCL approach.
4. Consider adding AprilTag fiducials for occasional global localization corrections.

## References

- [depthimage_to_laserscan wiki](https://wiki.ros.org/depthimage_to_laserscan)
- [Azure Kinect ROS Driver docs](../Azure_Kinect_ROS_Driver/docs/)
- [Nav2 AMCL documentation](https://navigation.ros.org/)
