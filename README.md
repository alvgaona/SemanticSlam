# DPS-SLAM: Dual Pose-graph Semantic SLAM

Semantic SLAM system for the [AeroStack2](https://github.com/aerostack2) UAV framework. It fuses odometry with semantic object detections using [g2o](https://github.com/RainerKuemmerle/g2o) pose-graph optimization.

## Features

- **Dual pose-graph optimization**: maintains a persistent global graph for long-term state estimation and a temporary graph that compresses multiple observations into a single optimized constraint, limiting graph growth while preserving detection information.
- **Semantic landmarks**: supports landmarks represented as full poses (6-DOF) or 3D points.
- **Fixed landmark anchoring**: allows known landmark positions to be configured as fixed references to reduce long-term drift.
- **Covariance propagation**: extracts per-node covariance estimates from graph marginals when promoting detections into the global graph.
- **ROS 2 multi-distro support**: compatible with ROS 2 Humble and Jazzy through `pixi-build-ros`.
- **TF integration**: publishes the corrected `map → odom` transform and localization estimates with covariance information.

## Installation

Requires [pixi](https://pixi.sh).

```bash
git clone https://github.com/aerostack2/SemanticSlam.git
cd SemanticSlam
pixi install
```

This resolves all dependencies (ROS2, G2O, Eigen3, AeroStack2) through conda channels.

To use a specific ROS2 distribution:

```bash
pixi install -e humble
pixi install -e jazzy    # default
```

## Usage

```bash
# Launch with default config
pixi run ros2 launch as2_semantic_slam semantic_slam_launch.py

# Launch with a specific config and namespace
pixi run ros2 launch as2_semantic_slam semantic_slam_launch.py \
    config_file:=config/tii_config.yaml namespace:=drone1 use_sim_time:=false

# Run the node directly
pixi run ros2 run as2_semantic_slam as2_semantic_slam_node \
    --ros-args --params-file config/config.yaml

# Rebuild after code changes
pixi reinstall ros-jazzy-as2-semantic-slam
```

## Configuration

Parameters are loaded from YAML config files. Three scenarios are provided in `config/`:

| Config | Motion input | Fixed objects | Use case |
|---|---|---|---|
| `config.yaml` | Template (empty) | 4 gates (commented) | Base/template |
| `fronton_config.yaml` | Vision pose | 2 gates | Camera-based UAV |
| `tii_config.yaml` | Odometry | 4 gates | Odometry-based platform |

### Key Parameters

```yaml
odometry_topic: "odometry"                        # nav_msgs/Odometry input
pose_topic: "none"                                 # geometry_msgs/PoseStamped alternative
detections_topic: "processed_gate_poses_array"     # as2_msgs/PoseStampedWithIDArray
map_frame: "drone0/map"
odom_frame: "drone0/odom"
robot_frame: "drone0/base_link"
main_graph_odometry_distance_threshold: 2.0        # meters between keyframes
generate_odom_map_transform: True
```

### Fixed Objects

Known landmark positions can be added to anchor the pose graph:

```yaml
fixed_objects:
  gate_1:
    type: "aruco"        # or "gate"
    pose: [4.0, 1.3, 1.13, 3.14]   # [x, y, z, yaw]
```

## Architecture

```mermaid
graph TD
    odom[Odometry / Pose] --> slam[SemanticSlam<br/>ROS2 Node]
    det[Detections] --> slam
    slam --> optimizer[OptimizerG2O]

    subgraph Dual-Graph Optimization
        optimizer --> main[Main Graph<br/>keyframes + merged detections]
        optimizer --> temp[Temp Graph<br/>detection staging & validation]
        temp -- promote with<br/>covariance --> main
    end

    main --> loc[Corrected Localization]
    main --> tf[TF: map → odom]
    main --> viz[RViz Markers]
```

The optimizer uses G2O's Levenberg-Marquardt algorithm with a CHOLMOD sparse linear solver.
Keyframes are inserted into the main graph when the robot moves beyond a configurable distance
threshold. Detections accumulate in the temporary graph and are promoted (with covariance) to
the main graph at each new keyframe, then the temporary graph resets.

## License

BSD-3-Clause. See [LICENSE](LICENSE).
