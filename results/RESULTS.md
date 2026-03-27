# Evaluation Results

## Setup

- **VIO**: OpenVINS with calibrated equidistant fisheye model (averaged across 5 flights)
- **SLAM**: G2O graph optimization with known gate positions as fixed landmarks
- **Detections**: Synthetic gate detections from VIO poses (recomputed with correct timestamps)
- **Evaluation**: `evaluate_slam.py` with per-flight earth_to_map SVD calibration (skip=1000 VIO samples)

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

| Flight | x      | y      | z      | roll   | pitch  | yaw     |
|--------|------- |--------|--------|--------|--------|---------|
| 01p    | -0.806 | -1.683 | -0.010 | 0.048  | -0.035 | -3.140  |
| 02p    | -0.478 | -1.650 | 0.028  | 0.007  | -0.016 | 3.114   |
| 03p    | -0.793 | -1.122 | -0.513 | 0.021  | -0.060 | -3.102  |
| 04p    | -1.236 | -0.692 | -0.379 | 0.044  | -0.067 | -2.967  |
| 05p    | -0.649 | -1.394 | 0.351  | 0.149  | 0.035  | -3.070  |

## Single-Graph Results

Detections routed directly to main graph. Throttled to 1 detection per gate per main keyframe.

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

## Dual-Graph Results

Detections routed to temp graph (gates as free nodes), optimized, then merged to main graph as absolute
measurements with covariance from graph marginals. Key fix: `setFixedObjects` removed from temp graph so
gate positions are freely estimated from observations rather than fixed.

### evaluate_slam.py (per-flight earth_to_map calibration, fixed detections)

Previous results (01p-05p) used buggy detection computation (rotation matrix was transposed,
introducing ~0.55m position error per detection). Results below are with the corrected detections.

| Flight | Metric | VIO RMSE | Corrected RMSE | Improvement |
|--------|--------|----------|----------------|------------|
| 01p    | 3D     | 1.627m   | 0.657m         |     +59.6% |
| 01p    | XY     | 1.172m   | 0.557m         |     +52.5% |
| 01p    | Z      | 1.128m   | 0.350m         |     +69.0% |
| 01p    | ROT    | 8.77deg  | 4.77deg        |     +45.5% |
| 07p    | 3D     | 2.179m   | 0.503m         |     +76.9% |
| 07p    | XY     | 2.066m   | 0.485m         |     +76.5% |
| 07p    | Z      | 0.690m   | 0.134m         |     +80.6% |
| 07p    | ROT    | 5.79deg  | 5.03deg        |     +13.1% |

## Single-Graph vs Dual-Graph Comparison (corrected detections)

Flight 07p lemniscate:

| Mode | Threshold | 3D | XY | Z | ROT | Opt (mean) |
|------|-----------|------|------|------|------|------------|
| Dual-graph | 2.0m | 0.481m (+77.5%) | 0.468m (+77.0%) | 0.111m (+83.3%) | 5.04deg (+11.9%) | — |
| Single-graph | 5.0m | 0.682m (+68.1%) | 0.647m (+68.2%) | 0.216m (+67.1%) | 3.52deg (+38.3%) | — |
| Single-graph | 2.0m | 0.546m (+74.8%) | 0.527m (+74.4%) | 0.143m (+78.9%) | 4.21deg (+27.0%) | 2.7ms |
| Single-graph | 0.5m | 0.449m (+78.6%) | 0.426m (+78.6%) | 0.142m (+78.8%) | 3.80deg (+33.3%) | 13.5ms |

Single-graph at 0.5m matches dual-graph accuracy but with 4x more nodes/edges (895 nodes, 4422 edges,
~3527 gate edges). Optimization grows to ~25ms at end of flight. The 2.0m threshold offers the best
trade-off: near-dual-graph accuracy at 5x lower optimization cost.

## Notes

- **Per-flight calibration**: earth_to_map must be computed per flight because OpenVINS initializes with
  an arbitrary heading. The calibration script (`scripts/calibrate_earth_to_map.py`) must skip the first
  ~1000 VIO samples to avoid unconverged estimates.
- **Throttling**: Single-graph throttles to 1 detection per gate per main keyframe. Without throttling,
  the graph is overwhelmed with redundant constraints and performance degrades.
- **Detections are synthetic**: Generated from VIO poses and known gate positions (zero noise). Real
  vision-based detections will have noise, missed detections, and false positives.
