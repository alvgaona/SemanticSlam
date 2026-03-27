#!/usr/bin/env python3
"""Plot trajectory comparison: VIO vs SLAM-corrected vs Ground Truth.

Generates a publication-quality figure with XY, X, Y, Z, Roll, Pitch, Yaw subplots.

Usage:
    pixi run python3 scripts/plot_trajectory.py \
        --e2m-xyz 0.408 -0.311 -0.271 --e2m-rpy -0.000 -0.033 -1.527

    pixi run python3 scripts/plot_trajectory.py \
        --log-dir results/run-09p-dual-2.0m-0.5m \
        --e2m-xyz 0.408 -0.311 -0.271 --e2m-rpy -0.000 -0.033 -1.527 \
        --output results/traj_09p.pdf
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 10,
    "axes.titlesize": 11,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "axes.grid": True,
    "grid.alpha": 0.3,
})


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log-dir", type=Path, default=Path("."))
    parser.add_argument("--e2m-xyz", type=float, nargs=3, required=True)
    parser.add_argument("--e2m-rpy", type=float, nargs=3, required=True)
    parser.add_argument("--output", type=Path, default=Path("trajectory_comparison.pdf"))
    parser.add_argument("--title", type=str, default=None)
    return parser.parse_args()


def main():
    args = parse_args()

    odom = pd.read_csv(args.log_dir / "slam_odom.csv").apply(pd.to_numeric, errors="coerce").dropna()
    gt = pd.read_csv(args.log_dir / "slam_ground_truth.csv")

    R_e2m = Rotation.from_euler("xyz", args.e2m_rpy).as_matrix()
    t_e2m = np.array(args.e2m_xyz)

    gt_ts = (gt["sec"] + gt["nsec"] * 1e-9).values
    odom_ts = (odom["sec"] + odom["nsec"] * 1e-9).values
    t0 = odom_ts[0]
    odom_t = odom_ts - t0
    gt_t = gt_ts - t0

    gt_xyz = gt[["x", "y", "z"]].values
    gt_quat = gt[["qx", "qy", "qz", "qw"]].values
    gt_rpy = np.degrees(Rotation.from_quat(gt_quat).as_euler("xyz"))

    cor_xyz = odom[["cor_x", "cor_y", "cor_z"]].values
    cor_quat = odom[["cor_qx", "cor_qy", "cor_qz", "cor_qw"]].values
    cor_rpy = np.degrees(Rotation.from_quat(cor_quat).as_euler("xyz"))

    raw_pos = odom[["raw_x", "raw_y", "raw_z"]].values
    raw_quat = odom[["raw_qx", "raw_qy", "raw_qz", "raw_qw"]].values
    raw_xyz = (R_e2m.T @ (raw_pos - t_e2m).T).T
    raw_rpy = np.degrees(np.array([
        (Rotation.from_matrix(R_e2m.T) * Rotation.from_quat(q)).as_euler("xyz")
        for q in raw_quat
    ]))

    c_gt = "#2c3e50"
    c_vio = "#e74c3c"
    c_cor = "#27ae60"
    lw_gt = 1.2
    lw = 0.8
    alpha_vio = 0.7

    fig = plt.figure(figsize=(12, 14))
    gs = fig.add_gridspec(4, 2, hspace=0.35, wspace=0.3)

    # XY trajectory
    ax_xy = fig.add_subplot(gs[0, :])
    ax_xy.plot(gt_xyz[:, 0], gt_xyz[:, 1], color=c_gt, lw=lw_gt, label="Ground Truth")
    ax_xy.plot(raw_xyz[:, 0], raw_xyz[:, 1], color=c_vio, lw=lw, alpha=alpha_vio, label="VIO (raw)")
    ax_xy.plot(cor_xyz[:, 0], cor_xyz[:, 1], color=c_cor, lw=lw, label="SLAM (corrected)")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("XY Trajectory")
    ax_xy.set_aspect("equal")
    ax_xy.legend(loc="upper right")

    # X, Y, Z vs time
    labels_pos = ["X (m)", "Y (m)", "Z (m)"]
    for i, label in enumerate(labels_pos):
        ax = fig.add_subplot(gs[1, 0] if i == 0 else (gs[1, 1] if i == 1 else gs[2, 0]))
        ax.plot(gt_t, gt_xyz[:, i], color=c_gt, lw=lw_gt, label="GT")
        ax.plot(odom_t, raw_xyz[:, i], color=c_vio, lw=lw, alpha=alpha_vio, label="VIO")
        ax.plot(odom_t, cor_xyz[:, i], color=c_cor, lw=lw, label="SLAM")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(label)
        ax.set_title(label.split(" ")[0] + " vs Time")
        if i == 0:
            ax.legend(loc="upper right", fontsize=7)

    # Roll, Pitch, Yaw vs time
    labels_rot = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
    positions = [gs[2, 1], gs[3, 0], gs[3, 1]]
    for i, (label, pos) in enumerate(zip(labels_rot, positions)):
        ax = fig.add_subplot(pos)
        ax.plot(gt_t, gt_rpy[:, i], color=c_gt, lw=lw_gt, label="GT")
        ax.plot(odom_t, raw_rpy[:, i], color=c_vio, lw=lw, alpha=alpha_vio, label="VIO")
        ax.plot(odom_t, cor_rpy[:, i], color=c_cor, lw=lw, label="SLAM")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(label)
        ax.set_title(label.split(" ")[0] + " vs Time")
        if i == 0:
            ax.legend(loc="upper right", fontsize=7)

    if args.title:
        fig.suptitle(args.title, fontsize=13, fontweight="bold", y=0.995)

    plt.savefig(args.output)
    print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
