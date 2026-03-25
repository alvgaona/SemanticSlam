#!/usr/bin/env python3
"""Compute earth_to_map calibration from a bag with VIO and ground truth.

Matches the first N VIO poses to ground truth by timestamp and computes
the rigid transform (SVD) from VIO odom frame to the GT (world/earth) frame.

Usage:
    pixi run python3 scripts/calibrate_earth_to_map.py \
        --bag dataset/piloted/flight-02p-ellipse/rosbag2_vio_detections \
        --n-samples 200
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--n-samples", type=int, default=1000,
                        help="Number of samples for calibration")
    parser.add_argument("--skip", type=int, default=1000,
                        help="Skip first N VIO samples (wait for convergence)")
    parser.add_argument("--vio-topic", default="/ov_msckf/odomimu")
    parser.add_argument("--gt-topic", default="/ground_truth/odometry")
    parser.add_argument("--storage", default="mcap")
    return parser.parse_args()


def main():
    args = parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))

    gt_pts, vio_pts = [], []
    gt_ts, vio_ts = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == args.gt_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gt_pts.append([p.x, p.y, p.z])
            gt_ts.append(t)
        elif topic == args.vio_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            vio_pts.append([p.x, p.y, p.z])
            vio_ts.append(t)

    gt_pts = np.array(gt_pts)
    vio_pts = np.array(vio_pts)
    gt_ts = np.array(gt_ts)
    vio_ts = np.array(vio_ts)

    start = min(args.skip, len(vio_pts) - 1)
    n_calib = min(args.n_samples, len(vio_pts) - start)
    matched_gt, matched_vio = [], []
    for i in range(start, start + n_calib):
        idx = np.argmin(np.abs(gt_ts - vio_ts[i]))
        matched_gt.append(gt_pts[idx])
        matched_vio.append(vio_pts[i])

    matched_gt = np.array(matched_gt)
    matched_vio = np.array(matched_vio)

    vio_c = matched_vio.mean(axis=0)
    gt_c = matched_gt.mean(axis=0)
    H = (matched_vio - vio_c).T @ (matched_gt - gt_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = gt_c - R @ vio_c

    R_e2m = R.T
    t_e2m = -R.T @ t
    rpy = Rotation.from_matrix(R_e2m).as_euler("xyz")

    vio_in_earth = (R @ matched_vio.T).T + t
    residual = np.sqrt(np.mean(np.sum((vio_in_earth - matched_gt) ** 2, axis=1)))

    print(f"Calibration from {n_calib} samples, residual: {residual:.4f}m")
    print()
    print("    earth_to_map:")
    print(f"      x: {t_e2m[0]:.6f}")
    print(f"      y: {t_e2m[1]:.6f}")
    print(f"      z: {t_e2m[2]:.6f}")
    print(f"      roll: {rpy[0]:.6f}")
    print(f"      pitch: {rpy[1]:.6f}")
    print(f"      yaw: {rpy[2]:.6f}")


if __name__ == "__main__":
    main()
