# Evaluation Results

## Setup

- **VIO**: OpenVINS with calibrated equidistant fisheye model (averaged across 5 flights)
- **SLAM**: G2O graph optimization with known gate positions as fixed landmarks
- **Detections**: Synthetic gate detections from VIO poses (recomputed with correct timestamps)
- **Evaluation**: `evo_ape` with SE(3) Umeyama alignment, and custom `evaluate_slam.py` with per-flight
  earth_to_map SVD calibration (skip=1000 VIO samples)

## Configuration

```yaml
main_graph_odometry_distance_threshold: 5.0
main_graph_odometry_distance_threshold_if_detections: 2.0
main_graph_odometry_orientation_threshold: 1.0
temp_graph_odometry_distance_threshold: 0.5
temp_graph_odometry_orientation_threshold: 1.0
throttle_detections: true
force_object_type: "aruco"
detection_covariance_factor: 0.1
detection_orientation_covariance_factor: 1.0
map_odom_security_threshold: 5.0
map_odom_transform_alpha: 0.3
calculate_odom_covariance: true
fixed_objects:
  gate_1: { type: "aruco", pose: [4.0, 1.3, 1.13, 3.14] }
  gate_2: { type: "aruco", pose: [4.0, -1.34, 1.16, 0.0] }
  gate_3: { type: "aruco", pose: [-4.0, -1.29, 1.16, 0.0] }
  gate_4: { type: "aruco", pose: [-3.97, 1.28, 1.17, 3.14] }
```

Per-flight `earth_to_map` calibration (SVD, skip=1000):

| Flight | x | y | z | roll | pitch | yaw |
|--------|-------|--------|--------|--------|--------|---------|
| 01p    | -0.783 | -1.599 | -0.114 | 0.041 | -0.043 | -3.129 |
| 02p    | -0.478 | -1.650 | 0.028 | 0.007 | -0.016 | 3.114 |
| 03p    | -0.793 | -1.122 | -0.513 | 0.021 | -0.060 | -3.102 |

## Single-Graph Results

Detections routed directly to main graph. Throttled to 1 detection per gate per main keyframe.

### evo_ape (SE(3) Umeyama alignment)

| Flight | VIO RMSE (m) | Corrected RMSE (m) | Improvement |
|--------|-------------|--------------------|-----------:|
| 01p    | 0.984       | 0.973              |      +1.1% |
| 02p    | 0.564       | 0.577              |      -2.3% |
| 03p    | 0.729       | 0.666              |      +8.6% |

### evaluate_slam.py (per-flight earth_to_map calibration)

| Flight | Metric | VIO RMSE (m) | Corrected RMSE (m) | Improvement |
|--------|--------|-------------|--------------------|-----------:|
| 01p    | 3D     | 1.635       | 1.286              |     +21.4% |
| 01p    | XY     | 1.331       | 1.085              |     +18.5% |
| 01p    | Z      | 0.950       | 0.691              |     +27.3% |
| 02p    | 3D     | 0.718       | 0.624              |     +13.1% |
| 02p    | XY     | 0.638       | 0.574              |     +10.0% |
| 02p    | Z      | 0.330       | 0.244              |     +26.1% |
| 03p    | 3D     | 1.010       | 0.788              |     +22.0% |
| 03p    | XY     | 0.886       | 0.664              |     +25.1% |
| 03p    | Z      | 0.486       | 0.425              |     +12.5% |

### Summary (evaluate_slam.py, 3D RMSE)

| Flight | VIO (m) | Corrected (m) | Improvement |
|--------|---------|---------------|-----------:|
| 01p    | 1.635   | 1.286         |     +21.4% |
| 02p    | 0.718   | 0.624         |     +13.1% |
| 03p    | 1.010   | 0.788         |     +22.0% |

## Notes

- **evo vs evaluate_slam.py**: evo uses optimal SE(3) Umeyama alignment (best possible alignment for both
  trajectories). evaluate_slam.py uses a fixed earth_to_map calibration per flight. evo gives a stricter,
  more conservative improvement number.
- **Per-flight calibration**: earth_to_map must be computed per flight because OpenVINS initializes with
  an arbitrary heading. The calibration script (`scripts/calibrate_earth_to_map.py`) must skip the first
  ~1000 VIO samples to avoid unconverged estimates.
- **Throttling**: Single-graph throttles to 1 detection per gate per main keyframe. Without throttling,
  the graph is overwhelmed with redundant constraints and performance degrades.
- **Detections are synthetic**: Generated from VIO poses and known gate positions (zero noise). Real
  vision-based detections will have noise, missed detections, and false positives.
