#!/usr/bin/env python3
import argparse
import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from as2_msgs.msg import PoseStampedWithID, PoseStampedWithIDArray


@dataclass
class DatasetPaths:
    flight_id: str
    dataset_dir: Path
    imu_csv: Path
    drone_state_csv: Path
    camera_csv: Path
    mocap_csv: Path
    image_dir: Path
    metadata_yaml: Path
    original_bag: Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert TII drone racing dataset to ROS2 bag"
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        required=True,
        help="Path to dataset directory (e.g., dataset/piloted/flight-01p-ellipse)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output bag directory (default: <dataset-dir>/rosbag_vio)",
    )
    parser.add_argument(
        "--storage",
        choices=["mcap", "sqlite3"],
        default="mcap",
        help="Bag storage format (default: mcap)",
    )
    parser.add_argument(
        "--no-images",
        action="store_true",
        help="Skip camera images for fast debug",
    )
    parser.add_argument(
        "--image-encoding",
        choices=["bgr8", "mono8"],
        default="bgr8",
        help="Image encoding (default: bgr8)",
    )
    parser.add_argument(
        "--compress",
        action="store_true",
        help="Enable zstd compression (mcap only)",
    )
    parser.add_argument(
        "--undistort",
        type=Path,
        default=None,
        help="Path to calibration JSON to rectify fisheye images to pinhole",
    )
    parser.add_argument(
        "--detection-range",
        type=float,
        default=15.0,
        help="Max range (m) for synthesized gate detections (default: 15.0)",
    )
    parser.add_argument(
        "--prepend-static",
        type=float,
        default=0.0,
        help="Prepend N seconds of static data (repeat first frame/IMU) before real data",
    )
    return parser.parse_args()


def resolve_paths(dataset_dir: Path) -> DatasetPaths:
    flight_id = dataset_dir.name
    imu_csv = dataset_dir / "csv_raw" / "ros2bag_dump" / f"imu_{flight_id}.csv"
    drone_state_csv = (
        dataset_dir / "csv_raw" / "ros2bag_dump" / f"drone_state_{flight_id}.csv"
    )
    camera_csv = dataset_dir / "csv_raw" / f"camera_{flight_id}.csv"
    mocap_csv = dataset_dir / "csv_raw" / f"mocap_{flight_id}.csv"
    image_dir = dataset_dir / f"camera_{flight_id}"
    metadata_yaml = dataset_dir / f"metadata_{flight_id}.yaml"
    original_bag = dataset_dir / f"ros2bag_{flight_id}"

    paths = DatasetPaths(
        flight_id=flight_id,
        dataset_dir=dataset_dir,
        imu_csv=imu_csv,
        drone_state_csv=drone_state_csv,
        camera_csv=camera_csv,
        mocap_csv=mocap_csv,
        image_dir=image_dir,
        metadata_yaml=metadata_yaml,
        original_bag=original_bag,
    )

    for field_name in [
        "imu_csv",
        "drone_state_csv",
        "camera_csv",
        "mocap_csv",
        "image_dir",
        "metadata_yaml",
    ]:
        p = getattr(paths, field_name)
        if not p.exists():
            print(f"WARNING: {field_name} not found: {p}")

    return paths


def ts_us_to_ros(timestamp_us: int) -> Time:
    t = Time()
    t.sec = int(timestamp_us // 1_000_000)
    t.nanosec = int((timestamp_us % 1_000_000) * 1000)
    return t


def ts_us_to_ns(timestamp_us: int) -> int:
    return int(timestamp_us) * 1000


def make_header(timestamp_us: int, frame_id: str) -> Header:
    h = Header()
    h.stamp = ts_us_to_ros(timestamp_us)
    h.frame_id = frame_id
    return h


def create_writer(
    output_dir: Path, storage_id: str, compress: bool = False
) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()

    storage_config_uri = ""
    if compress and storage_id == "mcap":
        config_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        config_file.write("output:\n  compression: zstd\n  chunk_size: 4194304\n")
        config_file.close()
        storage_config_uri = config_file.name

    storage_options = rosbag2_py.StorageOptions(
        uri=str(output_dir),
        storage_id=storage_id,
        storage_config_uri=storage_config_uri,
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    topics = [
        ("/imu", "sensor_msgs/msg/Imu"),
        ("/camera/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/ground_truth/pose", "geometry_msgs/msg/PoseStamped"),
        ("/ground_truth/odometry", "nav_msgs/msg/Odometry"),
        ("/ground_truth/gates", "visualization_msgs/msg/MarkerArray"),
        ("/detections/gates", "as2_msgs/msg/PoseStampedWithIDArray"),
    ]

    for topic_name, topic_type in topics:
        topic = rosbag2_py.TopicMetadata(
            name=topic_name,
            type=topic_type,
            serialization_format="cdr",
        )
        writer.create_topic(topic)

    return writer


def write_imu_from_bag(writer: rosbag2_py.SequentialWriter, original_bag: Path) -> int:
    from rosidl_runtime_py.utilities import get_message

    print(f"Writing IMU from original bag {original_bag.name}...")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(original_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/sensors/imu"]))

    imu_msg_type = get_message(type_map["/sensors/imu"])
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        src = deserialize_message(data, imu_msg_type)
        msg = Imu()
        msg.header = make_header(int(src.timestamp), "imu_link")
        msg.linear_acceleration = Vector3(
            x=float(src.accel_x), y=float(src.accel_y), z=float(src.accel_z)
        )
        msg.angular_velocity = Vector3(
            x=float(src.gyro_x), y=float(src.gyro_y), z=float(src.gyro_z)
        )
        msg.orientation_covariance[0] = -1.0
        writer.write("/imu", serialize_message(msg), ts_us_to_ns(int(src.timestamp)))
        count += 1

    del reader
    print(f"  IMU: {count} messages (from original bag, includes pre-flight data)")
    return count


def write_imu_messages(
    writer: rosbag2_py.SequentialWriter, imu_csv: Path, original_bag: Path = None
) -> int:
    if original_bag is not None and original_bag.exists():
        return write_imu_from_bag(writer, original_bag)
    print(f"Writing IMU from {imu_csv.name}...")
    df = pd.read_csv(imu_csv)
    count = 0

    for _, row in df.iterrows():
        msg = Imu()
        msg.header = make_header(int(row["timestamp"]), "imu_link")
        msg.linear_acceleration = Vector3(
            x=float(row["accel_x"]),
            y=float(row["accel_y"]),
            z=float(row["accel_z"]),
        )
        msg.angular_velocity = Vector3(
            x=float(row["gyro_x"]),
            y=float(row["gyro_y"]),
            z=float(row["gyro_z"]),
        )
        msg.orientation_covariance[0] = -1.0

        writer.write(
            "/imu",
            serialize_message(msg),
            ts_us_to_ns(int(row["timestamp"])),
        )
        count += 1

    print(f"  IMU: {count} messages")
    return count


def write_ground_truth_from_bag(
    writer: rosbag2_py.SequentialWriter, original_bag: Path
) -> int:
    from rosidl_runtime_py.utilities import get_message

    print(f"Writing ground truth from original bag {original_bag.name}...")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(original_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=["/perception/drone_state"])
    )

    msg_type = get_message(type_map["/perception/drone_state"])
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        src = deserialize_message(data, msg_type)
        t = int(src.timestamp)

        pose_msg = PoseStamped()
        pose_msg.header = make_header(t, "world")
        pose_msg.pose = src.pose

        odom_msg = Odometry()
        odom_msg.header = make_header(t, "world")
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = PoseWithCovariance(pose=src.pose)
        odom_msg.twist = TwistWithCovariance(twist=src.velocity)

        ts_ns = ts_us_to_ns(t)
        writer.write("/ground_truth/pose", serialize_message(pose_msg), ts_ns)
        writer.write("/ground_truth/odometry", serialize_message(odom_msg), ts_ns)
        count += 1

    del reader
    print(f"  Ground truth: {count} messages (from original bag)")
    return count


def write_ground_truth(
    writer: rosbag2_py.SequentialWriter,
    drone_state_csv: Path,
    original_bag: Path = None,
) -> int:
    if original_bag is not None and original_bag.exists():
        return write_ground_truth_from_bag(writer, original_bag)
    print(f"Writing ground truth from {drone_state_csv.name}...")
    df = pd.read_csv(drone_state_csv)
    count = 0

    for _, row in df.iterrows():
        ts = int(row["timestamp"])

        pose_msg = PoseStamped()
        pose_msg.header = make_header(ts, "world")
        pose_msg.pose = Pose(
            position=Point(
                x=float(row["pose_position_x"]),
                y=float(row["pose_position_y"]),
                z=float(row["pose_position_z"]),
            ),
            orientation=Quaternion(
                x=float(row["pose_orientation_x"]),
                y=float(row["pose_orientation_y"]),
                z=float(row["pose_orientation_z"]),
                w=float(row["pose_orientation_w"]),
            ),
        )

        odom_msg = Odometry()
        odom_msg.header = make_header(ts, "world")
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = PoseWithCovariance(pose=pose_msg.pose)
        odom_msg.twist = TwistWithCovariance(
            twist=Twist(
                linear=Vector3(
                    x=float(row["velocity_linear_x"]),
                    y=float(row["velocity_linear_y"]),
                    z=float(row["velocity_linear_z"]),
                ),
                angular=Vector3(
                    x=float(row["velocity_angular_x"]),
                    y=float(row["velocity_angular_y"]),
                    z=float(row["velocity_angular_z"]),
                ),
            )
        )

        ts_ns = ts_us_to_ns(ts)
        writer.write("/ground_truth/pose", serialize_message(pose_msg), ts_ns)
        writer.write("/ground_truth/odometry", serialize_message(odom_msg), ts_ns)
        count += 1

    print(f"  Ground truth: {count} messages (pose + odometry)")
    return count


def load_undistort_maps(calib_path: Path, width: int, height: int):
    with open(calib_path) as f:
        calib = json.load(f)

    if calib.get("model") == "fisheye" or calib.get("fisheye"):
        K = np.array(calib["mtx"], dtype=np.float64)
        D = np.array(calib.get("dist_fisheye", [0.0, 0.0, 0.0, 0.0]),
                      dtype=np.float64).reshape(4, 1)
    else:
        metadata_path = calib_path.parent.parent / "piloted"
        fov_h = calib.get("fov_horizontal", 155)
        fov_v = calib.get("fov_vertical", 115)
        import math
        fx = (width / 2.0) / math.radians(fov_h / 2.0)
        fy = (height / 2.0) / math.radians(fov_v / 2.0)
        K = np.array([
            [fx, 0, width / 2.0],
            [0, fy, height / 2.0],
            [0, 0, 1],
        ], dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (width, height), np.eye(3), balance=0.0,
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, (width, height), cv2.CV_16SC2,
    )
    print(f"  Undistort (fisheye): {calib_path.name}")
    print(f"  Equidistant intrinsics: fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print(f"  New pinhole intrinsics: fx={new_K[0,0]:.2f} fy={new_K[1,1]:.2f} cx={new_K[0,2]:.2f} cy={new_K[1,2]:.2f}")
    return map1, map2, new_K


def write_static_camera_preamble(
    writer: rosbag2_py.SequentialWriter,
    paths: DatasetPaths,
    metadata: dict,
    encoding: str,
    undistort_maps=None,
) -> int:
    if not paths.original_bag.exists():
        return 0

    imu_csv = pd.read_csv(paths.imu_csv, nrows=1)
    cam_csv = pd.read_csv(paths.camera_csv, nrows=1)
    imu_start_us = int(imu_csv["timestamp"].iloc[0])
    cam_start_us = int(cam_csv["timestamp"].iloc[0])

    gap_us = cam_start_us - imu_start_us
    if gap_us < 500_000:
        return 0

    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(paths.original_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=["/sensors/imu"])
    )
    msg_type = get_message(type_map["/sensors/imu"])
    first_msg = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        first_msg = deserialize_message(data, msg_type)
        break
    del reader

    if first_msg is None:
        return 0

    bag_imu_start_us = int(first_msg.timestamp)

    first_img_path = paths.image_dir / cam_csv["filename"].iloc[0]
    first_img = cv2.imread(str(first_img_path))
    if first_img is None:
        return 0

    if undistort_maps is not None:
        map1, map2, _ = undistort_maps
        first_img = cv2.remap(first_img, map1, map2, cv2.INTER_LINEAR)

    if encoding == "mono8":
        first_img = cv2.cvtColor(first_img, cv2.COLOR_BGR2GRAY)

    width = metadata.get("camera", {}).get("image_width", 640)
    height = metadata.get("camera", {}).get("image_height", 480)
    camera_info = CameraInfo()
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = "plumb_bob"
    camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.k = [0.0] * 9
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.p = [0.0] * 12

    img_msg_template = Image()
    img_msg_template.height = first_img.shape[0]
    img_msg_template.width = first_img.shape[1]
    img_msg_template.encoding = encoding
    img_msg_template.is_bigendian = False
    if encoding == "mono8":
        img_msg_template.step = first_img.shape[1]
    else:
        img_msg_template.step = first_img.shape[1] * 3
    img_msg_template.data = first_img.tobytes()

    cam_dt_us = 33333
    t = bag_imu_start_us
    count = 0
    print(f"Writing static camera preamble ({(cam_start_us - bag_imu_start_us) / 1e6:.1f}s)...")
    while t < cam_start_us:
        header = make_header(t, "camera")
        img_msg_template.header = header
        camera_info.header = header
        writer.write(
            "/camera/image_raw", serialize_message(img_msg_template), ts_us_to_ns(t)
        )
        writer.write(
            "/camera/camera_info", serialize_message(camera_info), ts_us_to_ns(t)
        )
        count += 1
        t += cam_dt_us

    print(f"  Static camera preamble: {count} frames at ~30Hz")
    return count


def write_camera_messages(
    writer: rosbag2_py.SequentialWriter,
    camera_csv: Path,
    image_dir: Path,
    metadata: dict,
    encoding: str,
    undistort_maps=None,
) -> int:
    print(f"Writing camera images from {image_dir.name}/...")
    df = pd.read_csv(camera_csv)
    width = metadata.get("camera", {}).get("image_width", 640)
    height = metadata.get("camera", {}).get("image_height", 480)

    map1, map2, new_mtx = None, None, None
    if undistort_maps is not None:
        map1, map2, new_mtx = undistort_maps

    camera_info = CameraInfo()
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = "plumb_bob"
    camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    if new_mtx is not None:
        camera_info.k = new_mtx.flatten().tolist()
        camera_info.p = [
            new_mtx[0, 0], 0.0, new_mtx[0, 2], 0.0,
            0.0, new_mtx[1, 1], new_mtx[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
    else:
        camera_info.k = [0.0] * 9
        camera_info.p = [0.0] * 12

    count = 0
    for _, row in df.iterrows():
        ts = int(row["timestamp"])
        filename = row["filename"]
        img_path = image_dir / filename

        if not img_path.exists():
            continue

        img = cv2.imread(str(img_path))
        if img is None:
            continue

        if map1 is not None:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        if encoding == "mono8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        header = make_header(ts, "camera")

        img_msg = Image()
        img_msg.header = header
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = False
        if encoding == "mono8":
            img_msg.step = img.shape[1]
        else:
            img_msg.step = img.shape[1] * 3
        img_msg.data = img.tobytes()

        camera_info.header = header

        ts_ns = ts_us_to_ns(ts)
        writer.write("/camera/image_raw", serialize_message(img_msg), ts_ns)
        writer.write("/camera/camera_info", serialize_message(camera_info), ts_ns)
        count += 1

        if count % 1000 == 0:
            print(f"  Camera: {count}/{len(df)} images...")

    print(f"  Camera: {count} images")
    return count


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    r = Rotation.from_euler("xyz", [roll, pitch, yaw])
    q = r.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def rotmat_to_quaternion(rot: np.ndarray) -> Quaternion:
    r = Rotation.from_matrix(rot)
    q = r.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def write_gate_ground_truth(
    writer: rosbag2_py.SequentialWriter, mocap_csv: Path
) -> int:
    print(f"Writing gate ground truth from {mocap_csv.name}...")
    df = pd.read_csv(mocap_csv, nrows=1)
    row = df.iloc[0]
    ts = int(row["timestamp"])

    marker_array = MarkerArray()
    for i in range(1, 5):
        prefix = f"gate{i}_int_"
        marker = Marker()
        marker.header = make_header(ts, "world")
        marker.ns = "gates"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = Pose(
            position=Point(
                x=float(row[f"{prefix}x"]),
                y=float(row[f"{prefix}y"]),
                z=float(row[f"{prefix}z"]),
            ),
            orientation=euler_to_quaternion(
                float(row[f"{prefix}roll"]),
                float(row[f"{prefix}pitch"]),
                float(row[f"{prefix}yaw"]),
            ),
        )
        marker.scale = Vector3(x=0.1, y=1.4, z=1.4)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        marker_array.markers.append(marker)

    writer.write(
        "/ground_truth/gates",
        serialize_message(marker_array),
        ts_us_to_ns(ts),
    )
    print(f"  Gate ground truth: 4 gates published")
    return 1


def write_gate_detections(
    writer: rosbag2_py.SequentialWriter,
    camera_csv: Path,
    mocap_csv: Path,
    detection_range: float,
) -> int:
    print(f"Synthesizing gate detections (range < {detection_range}m)...")
    cam_df = pd.read_csv(camera_csv)
    mocap_df = pd.read_csv(mocap_csv)
    mocap_timestamps = mocap_df["timestamp"].values
    count = 0

    for _, cam_row in cam_df.iterrows():
        cam_ts = int(cam_row["timestamp"])
        idx = np.argmin(np.abs(mocap_timestamps - cam_ts))
        mrow = mocap_df.iloc[idx]

        drone_rot = np.array(
            [
                [mrow["drone_rot[0]"], mrow["drone_rot[1]"], mrow["drone_rot[2]"]],
                [mrow["drone_rot[3]"], mrow["drone_rot[4]"], mrow["drone_rot[5]"]],
                [mrow["drone_rot[6]"], mrow["drone_rot[7]"], mrow["drone_rot[8]"]],
            ]
        )
        drone_pos = np.array([mrow["drone_x"], mrow["drone_y"], mrow["drone_z"]])

        det_msg = PoseStampedWithIDArray()
        header = make_header(cam_ts, "base_link")

        for i in range(1, 5):
            prefix = f"gate{i}_int_"
            gate_pos = np.array(
                [mrow[f"{prefix}x"], mrow[f"{prefix}y"], mrow[f"{prefix}z"]]
            )
            gate_rot = np.array(
                [
                    [
                        mrow[f"{prefix}rot[0]"],
                        mrow[f"{prefix}rot[1]"],
                        mrow[f"{prefix}rot[2]"],
                    ],
                    [
                        mrow[f"{prefix}rot[3]"],
                        mrow[f"{prefix}rot[4]"],
                        mrow[f"{prefix}rot[5]"],
                    ],
                    [
                        mrow[f"{prefix}rot[6]"],
                        mrow[f"{prefix}rot[7]"],
                        mrow[f"{prefix}rot[8]"],
                    ],
                ]
            )

            rel_pos = drone_rot.T @ (gate_pos - drone_pos)
            dist = np.linalg.norm(rel_pos)

            if dist > detection_range:
                continue

            rel_rot = drone_rot.T @ gate_rot
            q = rotmat_to_quaternion(rel_rot)

            det = PoseStampedWithID()
            det.id = f"gate_{i}"
            det.pose = PoseStamped()
            det.pose.header = header
            det.pose.pose = Pose(
                position=Point(x=float(rel_pos[0]), y=float(rel_pos[1]), z=float(rel_pos[2])),
                orientation=q,
            )
            det_msg.poses.append(det)

        if len(det_msg.poses) > 0:
            writer.write(
                "/detections/gates",
                serialize_message(det_msg),
                ts_us_to_ns(cam_ts),
            )
            count += 1

    print(f"  Gate detections: {count} messages")
    return count


def main():
    args = parse_args()
    paths = resolve_paths(args.dataset_dir.resolve())

    if args.output_dir is None:
        output_dir = paths.dataset_dir / "rosbag_vio"
    else:
        output_dir = args.output_dir.resolve()

    if output_dir.exists():
        print(f"ERROR: Output directory already exists: {output_dir}")
        sys.exit(1)

    metadata = {}
    if paths.metadata_yaml.exists():
        with open(paths.metadata_yaml) as f:
            metadata = yaml.safe_load(f)

    print(f"Dataset: {paths.flight_id}")
    print(f"Output:  {output_dir}")
    print(f"Storage: {args.storage}")
    print()

    writer = create_writer(output_dir, args.storage, args.compress)

    counts = {}
    counts["imu"] = write_imu_messages(writer, paths.imu_csv, paths.original_bag)
    counts["ground_truth"] = write_ground_truth(
        writer, paths.drone_state_csv, paths.original_bag
    )

    undistort_maps = None
    if args.undistort is not None:
        width = metadata.get("camera", {}).get("image_width", 640)
        height = metadata.get("camera", {}).get("image_height", 480)
        undistort_maps = load_undistort_maps(args.undistort.resolve(), width, height)

    if not args.no_images:
        counts["static_cam"] = write_static_camera_preamble(
            writer, paths, metadata, args.image_encoding, undistort_maps
        )
        counts["camera"] = write_camera_messages(
            writer, paths.camera_csv, paths.image_dir, metadata, args.image_encoding,
            undistort_maps=undistort_maps,
        )
    else:
        print("Skipping camera images (--no-images)")
        counts["camera"] = 0

    counts["gate_gt"] = write_gate_ground_truth(writer, paths.mocap_csv)
    counts["gate_detections"] = write_gate_detections(
        writer, paths.camera_csv, paths.mocap_csv, args.detection_range
    )

    del writer

    print()
    print("=== Summary ===")
    for k, v in counts.items():
        print(f"  {k}: {v}")
    print(f"  Output: {output_dir}")


if __name__ == "__main__":
    main()
