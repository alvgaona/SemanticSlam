# Evaluation Results

## Setup

- **VIO**: OpenVINS with calibrated equidistant fisheye model (averaged across 5 ellipse flights)
- **SLAM**: G2O pose graph optimization with known gate positions as fixed landmarks
- **Detections**: Synthetic gate detections from GT (mocap) poses, body-relative
- **Evaluation**: `evaluate_slam.py` with per-flight earth_to_map SVD calibration (skip=1000)
- **Dataset**: TII-RATM piloted flights, ArduCam IMX219 fisheye 640×480 ~106Hz, SBG Ellipse IMU 500Hz

## Configuration

Dual-graph mode, `main=2.0m`, `temp=0.5m` unless noted:

```yaml
use_dual_graph: true
main_graph_odometry_distance_threshold: 2.0
temp_graph_odometry_distance_threshold: 0.5
force_object_type: "aruco"
detection_covariance_factor: 0.1
detection_orientation_covariance_factor: 1.0
map_odom_transform_alpha: 0.3
calculate_odom_covariance: true
fixed_objects:
  gate_1: { type: "aruco", pose: [4.0, 1.3, 1.13, 0.0] }
  gate_2: { type: "aruco", pose: [4.0, -1.34, 1.16, 0.0] }
  gate_3: { type: "aruco", pose: [-4.0, -1.29, 1.16, 0.0] }
  gate_4: { type: "aruco", pose: [-3.97, 1.28, 1.17, 0.0] }
```

## Results — Dual-graph 2.0m/0.5m (default config)

All results with E2M.inverse() fix applied. Works for any VIO initialization angle.

| Flight | Track | E2M yaw | ATE 3D | 3D % | ATE XY | XY % | ATE Z | Z % | ATE ROT | ROT % | Nodes | Edges | Opt P95 |
|--------|-------|---------|--------|------|--------|------|-------|-----|---------|-------|-------|-------|---------|
| 01p | ellipse | +3.13 | 0.555m | +65.6 | 0.439m | +62.2 | 0.339m | +69.7 | 4.81° | +50.5 | 251 | 1218 | 7ms |
| 03p | ellipse | +3.14 | 0.417m | +49.9 | 0.393m | +51.3 | 0.140m | +31.3 | 6.07° | -25.7 | 193 | 932 | 6ms |
| 07p | lemniscate | -3.11 | 0.503m | +77.0 | 0.485m | +76.6 | 0.133m | +80.7 | 5.04° | +13.1 | 226 | 1097 | 5ms |
| 08p | lemniscate | -1.57 | 0.357m | +66.4 | 0.331m | +61.2 | 0.134m | +78.8 | 5.81° | +4.7 | 188 | 907 | 6ms |

Raw VIO baselines:

| Flight | VIO 3D | VIO XY | VIO Z | VIO ROT |
|--------|--------|--------|-------|---------|
| 01p | 1.613m | 1.160m | 1.120m | 9.73° |
| 03p | 0.833m | 0.807m | 0.204m | 4.83° |
| 07p | 2.180m | 2.070m | 0.690m | 5.77° |
| 08p | 1.063m | 0.854m | 0.632m | 6.10° |

## Results — Flight 01p Ellipse (parameter sweep)

| # | Mode | Main | Temp | ATE 3D | 3D % | ATE XY | XY % | ATE Z | Z % | ATE ROT | ROT % | Nodes | Edges | Opt P95 |
|---|--------|------|------|--------|-------|--------|-------|-------|------|---------|--------|-------|-------|---------|
| 0 | VIO only | — | — | 1.611m | — | 1.158m | — | 1.119m | — | 9.71° | — | — | — | — |
| 1 | Dual | 2.0m | 0.5m | 0.563m | +65.0 | 0.443m | +61.8 | 0.348m | +68.9 | 4.80° | +50.5 | 250 | 1213 | 8ms |
| 2 | Dual | 2.0m | 0.1m | 0.548m | +66.0 | 0.431m | +62.7 | 0.338m | +69.8 | 4.84° | +50.2 | 249 | 1188 | 8ms |
| 2b | Dual | 2.0m | 0.1m* | 0.432m | +72.8 | 0.419m | +62.8 | 0.107m | +90.4 | 4.88° | +49.4 | 239 | 1142 | 8ms |
| 3 | Dual | 0.5m | 0.1m | 0.502m | +67.8 | 0.317m | +72.7 | 0.389m | +62.6 | 3.82° | +59.1 | 992 | 4518 | 32ms |
| 4 | Single | 2.0m | — | 0.635m | +60.5 | 0.528m | +54.3 | 0.352m | +68.4 | 3.28° | +66.2 | 248 | 1203 | 7ms |
| 5 | Single | 0.5m | — | 0.537m | +65.1 | 0.390m | +66.1 | 0.368m | +63.8 | 2.91° | +68.5 | 991 | 4890 | 37ms |
| 6 | Single | 0.1m | — | 0.346m | +70.8 | 0.232m | +74.2 | 0.257m | +66.9 | 2.42° | +65.9 | 1822 | 8617 | 65ms |

## Results — Flight 07p Lemniscate (parameter sweep)

| # | Mode | Main | Temp | ATE 3D | 3D % | ATE XY | XY % | ATE Z | Z % | ATE ROT | ROT % | Nodes | Edges | Opt P95 |
|---|--------|------|------|--------|-------|--------|-------|-------|------|---------|--------|-------|-------|---------|
| 0 | VIO only | — | — | 2.180m | — | 2.070m | — | 0.690m | — | 5.77° | — | — | — | — |
| 1 | Dual | 2.0m | 0.5m | 0.503m | +77.0 | 0.485m | +76.6 | 0.133m | +80.7 | 5.04° | +13.1 | 226 | 1097 | 5ms |
| 2 | Dual | 2.0m | 0.1m | 0.498m | +77.1 | 0.479m | +76.9 | 0.138m | +80.0 | 4.98° | +14.1 | 226 | 1085 | 5ms |
| 3 | Dual | 0.5m | 0.1m | 0.415m | +80.1 | 0.390m | +80.3 | 0.142m | +78.8 | 4.55° | +19.7 | 895 | 4158 | 30ms |
| 4 | Single | 5.0m | — | 0.682m | +68.1 | 0.647m | +68.2 | 0.216m | +67.1 | 3.52° | +38.3 | 56 | 252 | <1ms |
| 5 | Single | 2.0m | — | 0.546m | +74.8 | 0.527m | +74.4 | 0.143m | +78.9 | 4.21° | +27.0 | 209 | 1012 | 5ms |
| 6 | Single | 0.5m | — | 0.449m | +78.6 | 0.426m | +78.6 | 0.142m | +78.8 | 3.80° | +33.3 | 895 | 4422 | 27ms |
| 7 | Single | 0.1m | — | 0.294m | +84.2 | 0.266m | +85.1 | 0.127m | +77.5 | 3.69° | +35.2 | 1781 | 8688 | 50ms |

## Key takeaways

- **Dual 2.0m/0.5m is the best real-time trade-off**: +65-77% 3D improvement with ~200 nodes and
  P95 5-8ms optimization latency
- **Temp graph threshold has minimal effect** on accuracy when main threshold is fixed (0.5m vs 0.1m
  temp: +77.0% vs +77.1%). The main graph size drives performance.
- **Main graph threshold drives both accuracy and latency**: halving it roughly doubles nodes/edges
- **Dual-graph produces fewer edges** than single-graph at the same main threshold
- **Single 0.1m achieves best absolute accuracy** (+84%, ATE=0.29m) but P95 ~50ms exceeds 30Hz budget
- **Single-graph has better rotation correction** (+33-68% vs +5-50% for dual)
- **E2M.inverse() fix**: critical for 90° VIO initialization flights. Previous code double-applied E2M
  rotation, accidentally working for 180° but failing for 90°.

## Notes

- **\* temp=0.1m***: Row 2b uses `temp_graph_odometry_orientation_threshold: 0.1` (vs 1.0 default),
  which triggers more temp keyframes on rotation changes, dramatically improving Z correction (+90%).
- **Detection fix**: Early results used a buggy detection computation (rotation matrix was transposed,
  introducing ~0.55m position error per detection). All results above use the corrected computation.
- **E2M.inverse() fix**: `map_ref = E2M.inverse() * odom` instead of `E2M * odom`. The previous code
  double-applied the E2M rotation to odom nodes. For 180° E2M (E2M²=I), this was invisible. For 90°
  E2M (E2M²=R(180°)), it caused 180° rotation error in every graph node, destroying rotation ATE.
- **Per-flight calibration**: earth_to_map must be computed per flight because OpenVINS initializes
  with an arbitrary heading. The calibration script must skip the first ~1000 VIO samples.
- **Throttling**: Single-graph throttles to 1 detection per gate per main keyframe.
- **Detections are synthetic**: Generated from GT (mocap) poses and known gate positions (zero noise).
  Real vision-based detections will have noise, missed detections, and false positives.
