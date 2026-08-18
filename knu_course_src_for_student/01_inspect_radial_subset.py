#!/usr/bin/env python3
"""Lab 01: inspect the distributed RADIal subset and radar point-cloud contract."""

from __future__ import annotations

import json

import matplotlib.pyplot as plt
import numpy as np

from _course_common import (
    load_labels,
    load_radial_pcl,
    radial_root,
    sample_id_from_path,
)


def main() -> None:
    root = radial_root()

    # ------------------------------------------------------------------
    # 1. Dataset structure
    # ------------------------------------------------------------------
    manifest = json.loads(
        (root / "subset_manifest.json").read_text(encoding="utf-8")
    )
    labels = load_labels(root / "labels.csv")

    pcl_files = sorted((root / "radar_PCL").glob("pcl_*.npy"))
    camera_files = sorted((root / "camera").glob("image_*.jpg"))
    fft_files = sorted((root / "radar_FFT").glob("fft_*.npy"))
    freespace_files = sorted(
        (root / "radar_Freespace").glob("freespace_*.png")
    )

    if not pcl_files:
        raise FileNotFoundError(f"no PCL files under {root}")

    split_count = {
        item["split"]: item["selected_samples"]
        for item in manifest["splits"]
    }

    modalities = {
        "camera": len(camera_files),
        "radar_PCL": len(pcl_files),
        "radar_FFT": len(fft_files),
        "freespace": len(freespace_files),
    }

    # ------------------------------------------------------------------
    # 2. Load one radar frame
    # ------------------------------------------------------------------
    sample_id = sample_id_from_path(pcl_files[0])
    points = load_radial_pcl(pcl_files[0])

    if points.ndim != 2 or points.shape[1] < 4:
        raise ValueError(
            f"unexpected radar PCL shape: {points.shape}; expected Nx4"
        )

    # RADIal course coordinate contract
    x_forward = points[:, 0]
    y_left = points[:, 1]
    z_up = points[:, 2]
    relative_speed = points[:, 3]

    # ------------------------------------------------------------------
    # 3. Derived radar features
    # ------------------------------------------------------------------
    range_xy = np.hypot(x_forward, y_left)

    # Coarse automotive ROI used only for visualization / data inspection.
    roi_mask = (
        np.isfinite(x_forward)
        & np.isfinite(y_left)
        & np.isfinite(relative_speed)
        & (x_forward > 0.0)
        & (x_forward <= 80.0)
        & (np.abs(y_left) <= 20.0)
    )

    # ------------------------------------------------------------------
    # 4. Console inspection
    # ------------------------------------------------------------------
    print("RADIal root :", root)
    print("profile     :", manifest.get("profile"))
    print("samples     :", manifest.get("sample_count"))
    print("split count :", split_count)
    print("modalities  :", modalities)
    print(
        "first sample:",
        sample_id,
        "PCL shape",
        points.shape,
        "label rows",
        len(labels.get(sample_id, [])),
    )
    print(
        "contract    : columns = "
        "x_forward_m, y_left_m, z_up_m, relative_speed_mps"
    )
    print("ROI points  :", int(np.count_nonzero(roi_mask)))

    if np.any(roi_mask):
        print(
            "range [m]   : "
            f"min={range_xy[roi_mask].min():.2f}, "
            f"median={np.median(range_xy[roi_mask]):.2f}, "
            f"max={range_xy[roi_mask].max():.2f}"
        )
        print(
            "rel vel [m/s]: "
            f"min={relative_speed[roi_mask].min():.2f}, "
            f"median={np.median(relative_speed[roi_mask]):.2f}, "
            f"max={relative_speed[roi_mask].max():.2f}"
        )

    # ------------------------------------------------------------------
    # 5. Save academic-style plots
    # ------------------------------------------------------------------
    output_dir = root.parent / "results" / "lab01"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Figure 1: Radar BEV
    fig, ax = plt.subplots(figsize=(7.2, 6.2))

    scatter = ax.scatter(
        y_left[roi_mask],
        x_forward[roi_mask],
        c=relative_speed[roi_mask],
        s=18,
        alpha=0.85,
    )

    ax.scatter(
        [0.0],
        [0.0],
        marker="^",
        s=100,
        label="Ego vehicle",
    )

    ax.set_xlabel("Lateral position, y_left [m]")
    ax.set_ylabel("Longitudinal position, x_forward [m]")
    ax.set_title(f"RADIal Radar Point Cloud — Sample {sample_id}")
    ax.set_xlim(-20.0, 20.0)
    ax.set_ylim(0.0, 80.0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Relative speed [m/s]")

    fig.tight_layout()

    bev_path = output_dir / f"sample_{sample_id}_radar_bev.png"
    fig.savefig(bev_path, dpi=180)
    plt.close(fig)

    # Figure 2: Range vs relative velocity
    fig, ax = plt.subplots(figsize=(7.2, 5.0))

    ax.scatter(
        range_xy[roi_mask],
        relative_speed[roi_mask],
        s=18,
        alpha=0.8,
    )

    ax.axhline(0.0, linewidth=1.0)
    ax.set_xlabel("Planar range [m]")
    ax.set_ylabel("Relative speed [m/s]")
    ax.set_title(f"Radar Range–Velocity Distribution — Sample {sample_id}")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    rv_path = output_dir / f"sample_{sample_id}_range_velocity.png"
    fig.savefig(rv_path, dpi=180)
    plt.close(fig)

    print("BEV plot    :", bev_path)
    print("R-V plot    :", rv_path)
    print("[PASS] subset, coordinate contract, and radar QA plots generated")


if __name__ == "__main__":
    main()
