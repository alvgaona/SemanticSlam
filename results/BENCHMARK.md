# Benchmark Instructions

## Prerequisites

- Pixi environment installed (`pixi install`)
- OpenVINS built and configured
- TII-RATM dataset flights in `dataset/piloted/`

## Pipeline

### 1. Generate rosbag from raw dataset

```bash
pixi run python3 scripts/dataset_to_rosbag.py \
    --dataset-dir dataset/piloted/flight-XXp-ellipse --compress
```

Output: `dataset/piloted/flight-XXp-ellipse/rosbag_vio/`

For image-only bag (separate):
```bash
pixi run python3 scripts/dataset_images_to_rosbag.py \
    --dataset-dir dataset/piloted/flight-XXp-ellipse --compress
```

### 2. Run OpenVINS and record

```bash
# Terminal 1 — play raw bag
pixi run ros2 bag play dataset/piloted/flight-XXp-ellipse/rosbag_vio/ --clock

# Terminal 2 — OpenVINS (ellipse config)
ros2 launch ov_msckf subscribe.launch.py \
    config_path:=$HOME/workspaces/ov_ws/src/open_vins/config/tii_ellipse/estimator_config.yaml \
    rviz_enable:=false

# Terminal 3 — record all topics
pixi run ros2 bag record -a -s mcap \
    -o dataset/piloted/flight-XXp-ellipse/rosbag2_openvins
```

For lemniscate flights, use `tii_lemniscate` config instead of `tii_ellipse`.

### 3. Calibrate earth_to_map

```bash
pixi run python3 scripts/calibrate_earth_to_map.py \
    --bag dataset/piloted/flight-XXp-ellipse/rosbag2_openvins --skip 1000
```

Update `dual_pose_graph/config/tii_config.yaml` with the output values, then rebuild:

```bash
pixi reinstall ros-humble-dual-pose-graph
```

### 4. Run SLAM

```bash
# Terminal 1 — play OpenVINS bag (contains /clock, /tf, VIO, detections, GT)
pixi run ros2 bag play dataset/piloted/flight-XXp-ellipse/rosbag2_openvins/

# Terminal 2 — launch SLAM
pixi run ros2 launch dual_pose_graph dual_pose_graph.launch.py use_sim_time:=true
```

### 5. Evaluate ATE

```bash
# Extract ground truth (once per flight)
pixi run python3 scripts/extract_ground_truth.py \
    --bag dataset/piloted/flight-XXp-ellipse/rosbag2_openvins

# Compute ATE (use calibration values from step 3)
pixi run python3 scripts/evaluate_slam.py \
    --e2m-xyz X Y Z --e2m-rpy R P Y
```

### 6. Analyze graph stats and optimization latency

```bash
pixi run python3 scripts/analyze_keyframes.py
```

## Configuration Parameters

Key parameters in `dual_pose_graph/config/tii_config.yaml`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `use_dual_graph` | `true` = dual-graph, `false` = single-graph | `true` |
| `main_graph_odometry_distance_threshold` | Meters between main keyframes | `2.0` |
| `temp_graph_odometry_distance_threshold` | Meters between temp keyframes (dual only) | `0.5` |
| `temp_graph_odometry_orientation_threshold` | Radians between temp keyframes (dual only) | `1.0` |
| `throttle_detections` | Limit 1 detection per gate per keyframe (single only) | `true` |
| `detection_covariance_factor` | Detection position covariance scale | `0.1` |
| `map_odom_transform_alpha` | Low-pass filter for map→odom TF update | `0.3` |

## Sweep Example

To compare configurations on a single flight:

```bash
# Dual-graph 2.0m main / 0.5m temp
# Set use_dual_graph: true, main: 2.0, temp: 0.5
pixi reinstall ros-humble-dual-pose-graph
# Run SLAM (step 4), evaluate (step 5-6)
mkdir -p results/run-XXp-dual-2.0m-0.5m && cp slam_*.csv results/run-XXp-dual-2.0m-0.5m/

# Single-graph 2.0m
# Set use_dual_graph: false, main: 2.0
pixi reinstall ros-humble-dual-pose-graph
# Run SLAM, evaluate
mkdir -p results/run-XXp-single-2.0m && cp slam_*.csv results/run-XXp-single-2.0m/
```

## Cleanup

Remove intermediate bags to save disk:

```bash
rm -rf dataset/piloted/flight-XXp-ellipse/rosbag_vio
rm -rf dataset/piloted/flight-XXp-ellipse/rosbag2_openvins
```

Only `rosbag2_openvins` is needed for re-running SLAM evaluations.
