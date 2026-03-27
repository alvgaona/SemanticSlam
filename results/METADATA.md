# Data Frames and Coordinate Systems

Analysis from flight-01p-ellipse rosbag2_vio_detections bag.

## Frames

| Data | Frame ID | Description |
|------|----------|-------------|
| GT pose | `world` | Mocap world frame |
| GT gates | `world` | Same mocap frame as GT pose |
| VIO odometry | `odom` → `base_link` | OpenVINS arbitrary frame |
| SLAM output | `earth` | Corrected localization in earth frame |
| Gate config | `earth`/`map` | Known gate positions used by SLAM |

## Ground Truth (world frame)

Drone initial pose: (0.002, -1.485, 0.070), rpy=(-1.2, 1.9, -2.2) deg

Gate positions from `/ground_truth/gates`:

| Gate | x | y | z | yaw (deg) |
|------|-------|--------|-------|-----------|
| 1 | 4.009 | 1.308 | 1.164 | 0.0 |
| 2 | 3.966 | -1.286 | 1.169 | -1.1 |
| 3 | -3.943 | -1.322 | 1.167 | 0.3 |
| 4 | -3.981 | 1.312 | 1.144 | 0.3 |

## OpenVINS VIO (odom frame)

Drone initial pose: (-0.117, -0.012, 0.603), rpy=(-0.1, 2.9, -179.9) deg

OpenVINS initializes with arbitrary heading. For ellipse flights 01p-05p, yaw ≈ ±180° (drone's +X in
world appears as -X in odom). Flight 06p initialized at yaw ≈ -89° (anomalous).

## Gate Config (earth/map frame)

Positions used in `tii_config.yaml`:

| Gate | x | y | z | yaw (rad) |
|------|------|-------|------|-----------|
| 1 | 4.0 | 1.3 | 1.13 | 3.14 |
| 2 | 4.0 | -1.34 | 1.16 | 0.0 |
| 3 | -4.0 | -1.29 | 1.16 | 0.0 |
| 4 | -3.97 | 1.28 | 1.17 | 3.14 |

Note: config gate yaw for gates 1 & 4 is 3.14 rad (180 deg), but bag GT shows yaw ≈ 0 deg for all gates.
Position values are close but not identical to bag GT (rounded/estimated vs mocap-measured).

## earth_to_map Transform

Maps from world/earth frame to VIO/odom frame. Per-flight because OpenVINS heading is arbitrary.

Typical values for ellipse flights: yaw ≈ -3.14 rad (≈ -180 deg), accounting for the 180 deg VIO heading
offset. Translation varies per flight due to different VIO initialization positions.

## Timestamp Epochs

- Raw dataset sensor timestamps: Aug 2023 (1691231xxx)
- Bag receive timestamps: Mar 2026 (1774xxx)
- `/clock` topic contains 2023 sim-time timestamps
- VIO messages use 2023 header.stamp
- Must play bags without `--clock` if `/clock` is already recorded
