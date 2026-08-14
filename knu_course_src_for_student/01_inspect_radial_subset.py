#!/usr/bin/env python3
"""
Practice 01: RADIal Dataset & Radar Point-Cloud QA

목표
- RADIal subset의 데이터 구성을 확인합니다.
- Radar PCL의 좌표계와 Relative Velocity를 이해합니다.
- Forward ADAS용 ROI를 적용합니다.
- Radar BEV 및 Range–Velocity Plot을 생성합니다.
"""

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


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


def main() -> None:
    root = radial_root()

    # ------------------------------------------------------------
    # Example. Dataset 정보 및 Sensor Modality 검색
    # ------------------------------------------------------------
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
        raise FileNotFoundError(f"Radar PCL 데이터를 찾을 수 없습니다: {root}")

    # ------------------------------------------------------------
    # Practice 1. Dataset Split 확인
    # Dictionary: {key: value for item in iterable}
    # 사용할 key: "split", "selected_samples"
    # ------------------------------------------------------------

    ## TODO 1
    split_name_key = None
    sample_count_key = None

    check_completed("Practice 1: split_name_key", split_name_key)
    check_completed("Practice 1: sample_count_key", sample_count_key)

    split_count = {
        item[split_name_key]: item[sample_count_key]
        for item in manifest["splits"]
    }

    # ------------------------------------------------------------
    # Practice 2. Sensor Modality 개수 확인
    # len(files): 파일 목록에 포함된 원소 개수
    # ------------------------------------------------------------

    ## TODO 2
    camera_source = None
    radar_pcl_source = None
    radar_fft_source = None
    freespace_source = None

    check_completed("Practice 2: camera_source", camera_source)
    check_completed("Practice 2: radar_pcl_source", radar_pcl_source)
    check_completed("Practice 2: radar_fft_source", radar_fft_source)
    check_completed("Practice 2: freespace_source", freespace_source)

    modalities = {
        "camera": len(camera_source),
        "radar_PCL": len(radar_pcl_source),
        "radar_FFT": len(radar_fft_source),
        "freespace": len(freespace_source),
    }

    # ------------------------------------------------------------
    # Example. 첫 번째 Radar Point Cloud 불러오기
    # ------------------------------------------------------------
    sample_id = sample_id_from_path(pcl_files[0])
    points = load_radial_pcl(pcl_files[0])

    if points.ndim != 2 or points.shape[1] < 4:
        raise ValueError(f"Unexpected Radar PCL shape: {points.shape}")

    # ------------------------------------------------------------
    # Practice 3. Radar PCL Coordinate Contract
    #
    # points[:, i] : 모든 Point의 i번째 Feature 선택
    #
    # column 0 : x_forward_m
    # column 1 : y_left_m
    # column 2 : z_up_m
    # column 3 : relative_speed_mps
    # ------------------------------------------------------------

    ## TODO 3
    x_forward_col = None
    y_left_col = None
    z_up_col = None
    relative_speed_col = None

    check_completed("Practice 3: x_forward_col", x_forward_col)
    check_completed("Practice 3: y_left_col", y_left_col)
    check_completed("Practice 3: z_up_col", z_up_col)
    check_completed("Practice 3: relative_speed_col", relative_speed_col)

    x_forward = points[:, x_forward_col]
    y_left = points[:, y_left_col]
    z_up = points[:, z_up_col]
    relative_speed = points[:, relative_speed_col]

    # ------------------------------------------------------------
    # Practice 4. Radar Range 계산
    #
    # np.hypot(x, y)
    # → sqrt(x² + y²), 피타고라스 정리를 이용한 평면 거리
    # ------------------------------------------------------------

    ## TODO 4
    range_x = None
    range_y = None

    check_completed("Practice 4: range_x", range_x)
    check_completed("Practice 4: range_y", range_y)

    range_xy = np.hypot(range_x, range_y)

    # ------------------------------------------------------------
    # Practice 5. Forward ADAS ROI
    #
    # np.isfinite(x) : NaN / Inf 제거
    # np.abs(x)      : 절댓값
    #
    # 조건:
    #   0 < x_forward <= 80 m
    #   |y_left| <= 20 m
    # ------------------------------------------------------------

    ## TODO 5
    roi_forward = None
    roi_lateral = None
    roi_velocity = None

    forward_min_m = None
    forward_max_m = None
    lateral_max_m = None

    check_completed("Practice 5: roi_forward", roi_forward)
    check_completed("Practice 5: roi_lateral", roi_lateral)
    check_completed("Practice 5: roi_velocity", roi_velocity)
    check_completed("Practice 5: forward_min_m", forward_min_m)
    check_completed("Practice 5: forward_max_m", forward_max_m)
    check_completed("Practice 5: lateral_max_m", lateral_max_m)

    roi_mask = (
        np.isfinite(roi_forward)
        & np.isfinite(roi_lateral)
        & np.isfinite(roi_velocity)
        & (roi_forward > forward_min_m)
        & (roi_forward <= forward_max_m)
        & (np.abs(roi_lateral) <= lateral_max_m)
    )

    # ------------------------------------------------------------
    # Check. Dataset / Radar 기본 통계
    # ------------------------------------------------------------
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
    print("contract    : x_forward, y_left, z_up, relative_speed")
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

    # ------------------------------------------------------------
    # Practice 6. Radar BEV 입력 구성
    #
    # data[roi_mask] : ROI 조건을 통과한 Point만 선택
    #
    # X-axis : Lateral Position
    # Y-axis : Longitudinal Position
    # Color  : Relative Speed
    # ------------------------------------------------------------

    ## TODO 6
    bev_x_source = None
    bev_y_source = None
    bev_color_source = None

    check_completed("Practice 6: bev_x_source", bev_x_source)
    check_completed("Practice 6: bev_y_source", bev_y_source)
    check_completed("Practice 6: bev_color_source", bev_color_source)

    bev_x = bev_x_source[roi_mask]
    bev_y = bev_y_source[roi_mask]
    bev_value = bev_color_source[roi_mask]

    # ------------------------------------------------------------
    # Result 1. Radar Bird's-Eye View
    # ------------------------------------------------------------
    output_dir = root.parent / "results" / "lab01_student"
    output_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7.2, 6.2))

    scatter = ax.scatter(
        bev_x,
        bev_y,
        c=bev_value,
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

    # ------------------------------------------------------------
    # Result 2. Range–Velocity Distribution
    # ------------------------------------------------------------
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

    print()
    print("Radar BEV     :", bev_path)
    print("Range-Velocity:", rv_path)
    print("[PASS] Practice 01 completed")

    # Challenge
    # forward_max_m: 80 → 50 m
    # lateral_max_m: 20 → 5 m
    # 변경 전후 ROI Point 수와 Plot을 비교해보세요.


if __name__ == "__main__":
    main()
