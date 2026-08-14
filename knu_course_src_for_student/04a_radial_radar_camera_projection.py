#!/usr/bin/env python3
"""
Practice 04A: Radar-to-Camera Geometric Projection

목표
- Radar Point와 Camera Image의 좌표계를 연결합니다.
- RADIal calibration을 이용하여 Radar Point를 영상에 투영합니다.
- Radar ROI 및 Projection 결과를 확인합니다.
- Relative Speed를 색상으로 표현합니다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from _course_common import (
    load_radial_pcl,
    radial_root,
)


MAIN_CLIP_START = 9015
MAIN_CLIP_END = 9142

DEFAULT_SAMPLE_ID = 9050


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Projection ROI
#
# np.hypot(x, y): sqrt(x²+y²), Radar 평면 거리
#
# Forward ADAS용 조건:
#   Max Range     = 80 m
#   Lateral Limit = ±20 m
# ------------------------------------------------------------

## TODO 1
DEFAULT_MAX_RANGE_M = None
DEFAULT_LATERAL_LIMIT_M = None


# ------------------------------------------------------------
# Practice 2. Visualization Point Size
#
# plt.scatter(..., s=point_size)
# Radar Point를 영상 위에 표시할 크기
# 기본값 = 18
# ------------------------------------------------------------

## TODO 2
DEFAULT_POINT_SIZE = None


def load_calibration(root: Path):
    """수업 데이터에 포함된 RADIal Camera calibration을 불러옵니다."""

    path = root / "calibration" / "radar_camera.json"

    if not path.is_file():
        raise FileNotFoundError(
            f"Calibration file not found: {path}"
        )

    calibration = json.loads(
        path.read_text(encoding="utf-8")
    )

    camera_matrix = np.asarray(
        calibration["camera_matrix"],
        dtype=np.float64,
    )

    distortion = np.asarray(
        calibration["distortion_coefficients"],
        dtype=np.float64,
    )

    rotation_vector = np.asarray(
        calibration["rotation_vector"],
        dtype=np.float64,
    )

    translation_vector = np.asarray(
        calibration["translation_vector"],
        dtype=np.float64,
    )

    return (
        calibration,
        camera_matrix,
        distortion,
        rotation_vector,
        translation_vector,
    )


def scaled_camera_matrix(
    camera_matrix: np.ndarray,
    calibration: dict,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    """
    Calibration 기준 해상도와 실제 Course Image 해상도가 다르면
    Camera Intrinsic Matrix도 같은 비율로 조정해야 합니다.
    """

    ref_width = float(
        calibration["reference_image_width"]
    )

    ref_height = float(
        calibration["reference_image_height"]
    )

    scale_x = image_width / ref_width
    scale_y = image_height / ref_height

    K = camera_matrix.copy()

    K[0, 0] *= scale_x
    K[0, 2] *= scale_x

    K[1, 1] *= scale_y
    K[1, 2] *= scale_y

    return K


def project_radar_points(
    points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
    max_range_m: float,
    lateral_limit_m: float,
):

    # ------------------------------------------------------------
    # Radar PCL Contract
    #
    # column 0 : x_forward
    # column 1 : y_left
    # column 2 : z_up
    # column 3 : relative_speed
    # ------------------------------------------------------------

    x_forward = points[:, 0]
    y_left = points[:, 1]
    z_up = points[:, 2]
    relative_speed = points[:, 3]

    range_xy = np.hypot(
        x_forward,
        y_left,
    )

    # ------------------------------------------------------------
    # Radar ROI
    # ------------------------------------------------------------

    roi_mask = (
        np.isfinite(points).all(axis=1)
        & (x_forward > 0.0)
        & (range_xy <= max_range_m)
        & (np.abs(y_left) <= lateral_limit_m)
    )

    x_forward = x_forward[roi_mask]
    y_left = y_left[roi_mask]
    z_up = z_up[roi_mask]

    relative_speed = relative_speed[
        roi_mask
    ]

    range_xy = range_xy[
        roi_mask
    ]

    # ------------------------------------------------------------
    # Practice 3. Coordinate Convention 변환
    #
    # Course PCL:
    #   x_forward, y_left, z_up
    #
    # Projection 입력:
    #   x_left, y_forward, z_up
    #
    # np.column_stack((a,b,c))
    # → 각 좌표를 Nx3 Point Array로 결합
    # ------------------------------------------------------------

    ## TODO 3
    projection_x = None
    projection_y = None
    projection_z = None

    check_completed(
        "Practice 3: projection_x",
        projection_x,
    )
    check_completed(
        "Practice 3: projection_y",
        projection_y,
    )
    check_completed(
        "Practice 3: projection_z",
        projection_z,
    )

    projection_points = np.column_stack(
        (
            projection_x,
            projection_y,
            projection_z,
        )
    ).astype(np.float64)

    # ------------------------------------------------------------
    # Camera 뒤쪽 Point 제거
    #
    # cv2.Rodrigues(rvec)
    # → Rotation Vector를 3x3 Rotation Matrix로 변환
    # ------------------------------------------------------------

    rotation_matrix, _ = cv2.Rodrigues(
        rotation_vector
    )

    camera_points = (
        rotation_matrix
        @ projection_points.T
    ).T + translation_vector.reshape(1, 3)

    front_mask = (
        camera_points[:, 2] > 0.1
    )

    projection_points = projection_points[
        front_mask
    ]

    relative_speed = relative_speed[
        front_mask
    ]

    range_xy = range_xy[
        front_mask
    ]

    # ------------------------------------------------------------
    # Practice 4. 3D Radar Point → Camera Pixel
    #
    # cv2.projectPoints(
    #     objectPoints,
    #     rvec,
    #     tvec,
    #     cameraMatrix,
    #     distCoeffs
    # )
    #
    # Calibration을 사용해 3D Point를 2D Pixel로 변환합니다.
    # ------------------------------------------------------------

    ## TODO 4
    projection_input = None

    check_completed(
        "Practice 4: projection_input",
        projection_input,
    )

    pixels, _ = cv2.projectPoints(
        projection_input,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )

    pixels = pixels.reshape(-1, 2)

    return (
        pixels,
        relative_speed,
        range_xy,
    )


def main() -> None:

    check_completed(
        "Practice 1: DEFAULT_MAX_RANGE_M",
        DEFAULT_MAX_RANGE_M,
    )

    check_completed(
        "Practice 1: DEFAULT_LATERAL_LIMIT_M",
        DEFAULT_LATERAL_LIMIT_M,
    )

    check_completed(
        "Practice 2: DEFAULT_POINT_SIZE",
        DEFAULT_POINT_SIZE,
    )

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sample",
        type=int,
        default=DEFAULT_SAMPLE_ID,
    )

    parser.add_argument(
        "--max-range",
        type=float,
        default=DEFAULT_MAX_RANGE_M,
    )

    parser.add_argument(
        "--lateral-limit",
        type=float,
        default=DEFAULT_LATERAL_LIMIT_M,
    )

    parser.add_argument(
        "--point-size",
        type=float,
        default=DEFAULT_POINT_SIZE,
    )

    args = parser.parse_args()

    if not (
        MAIN_CLIP_START
        <= args.sample
        <= MAIN_CLIP_END
    ):
        raise ValueError(
            f"Main Clip Sample은 "
            f"{MAIN_CLIP_START}~{MAIN_CLIP_END} 범위를 사용하세요."
        )

    root = radial_root()

    image_path = (
        root
        / "camera"
        / f"image_{args.sample:06d}.jpg"
    )

    pcl_path = (
        root
        / "radar_PCL"
        / f"pcl_{args.sample:06d}.npy"
    )

    image_bgr = cv2.imread(
        str(image_path)
    )

    if image_bgr is None:
        raise FileNotFoundError(
            f"Camera image not found: {image_path}"
        )

    image_rgb = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2RGB,
    )

    points = load_radial_pcl(
        pcl_path
    )

    (
        calibration,
        camera_matrix,
        distortion,
        rotation_vector,
        translation_vector,
    ) = load_calibration(
        root
    )

    image_height, image_width = (
        image_rgb.shape[:2]
    )

    camera_matrix = scaled_camera_matrix(
        camera_matrix,
        calibration,
        image_width,
        image_height,
    )

    (
        pixels,
        relative_speed,
        range_xy,
    ) = project_radar_points(
        points,
        camera_matrix,
        distortion,
        rotation_vector,
        translation_vector,
        args.max_range,
        args.lateral_limit,
    )

    u = pixels[:, 0]
    v = pixels[:, 1]

    # 영상 밖으로 투영된 Point 제거
    image_mask = (
        np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0)
        & (u < image_width)
        & (v >= 0)
        & (v < image_height)
    )

    u = u[image_mask]
    v = v[image_mask]

    relative_speed = relative_speed[
        image_mask
    ]

    range_xy = range_xy[
        image_mask
    ]

    # ------------------------------------------------------------
    # Result. Camera + Radar Projection
    # ------------------------------------------------------------

    output_dir = (
        root.parent
        / "results"
        / "lab04a_student"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    fig, ax = plt.subplots(
        figsize=(12, 7)
    )

    ax.imshow(
        image_rgb
    )

    scatter = ax.scatter(
        u,
        v,
        c=relative_speed,
        s=args.point_size,
        cmap="coolwarm",
        alpha=0.85,
        vmin=-15.0,
        vmax=15.0,
    )

    cbar = fig.colorbar(
        scatter,
        ax=ax,
        fraction=0.025,
        pad=0.02,
    )

    cbar.set_label(
        "Relative speed [m/s]"
    )

    ax.set_title(
        f"RADIal Radar-to-Camera Projection — Sample {args.sample}"
    )

    ax.set_xlabel(
        "Image u [pixel]"
    )

    ax.set_ylabel(
        "Image v [pixel]"
    )

    ax.set_xlim(
        0,
        image_width,
    )

    ax.set_ylim(
        image_height,
        0,
    )

    fig.tight_layout()

    output_path = (
        output_dir
        / f"sample_{args.sample}_projection.png"
    )

    fig.savefig(
        output_path,
        dpi=180,
    )

    plt.close(fig)

    print(
        "sample          :",
        args.sample,
    )

    print(
        "input PCL points:",
        len(points),
    )

    print(
        "projected points:",
        len(u),
    )

    if len(range_xy):
        print(
            "projected range : "
            f"{range_xy.min():.1f} - "
            f"{range_xy.max():.1f} m"
        )

    print(
        "result          :",
        output_path,
    )

    print(
        "[PASS] Practice 04A completed"
    )


if __name__ == "__main__":
    main()


# ------------------------------------------------------------
# 실행
#
# python3 04a_radial_radar_camera_projection.py
#
# 결과 확인
# xdg-open ~/260818_0820_knu_course/knu_course_data/results/\
# lab04a_student/sample_9050_projection.png
#
#
# Mini Experiment 1: Range Gate
# python3 04a_radial_radar_camera_projection.py --max-range 40
#
# Mini Experiment 2: Lateral ROI
# python3 04a_radial_radar_camera_projection.py --lateral-limit 5
#
# Mini Experiment 3: Point Size
# python3 04a_radial_radar_camera_projection.py --point-size 50
#
# 각 설정에서 Projected Point 수와 Overlay 결과를 비교하세요.
# ------------------------------------------------------------
