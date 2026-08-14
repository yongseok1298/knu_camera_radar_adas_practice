#!/usr/bin/env python3
"""
Practice 05: Radar FFT Processing

목표
- Complex Radar FFT Tensor의 구조를 확인합니다.
- FFT Magnitude와 dB Scale의 의미를 이해합니다.
- 2D FFT Magnitude Map을 생성합니다.
- Percentile Threshold로 Strong-response Bin을 비교합니다.

주의:
현재 수업 subset에서는 plot axis를 실제 Range / Doppler 단위로
보정할 metadata가 충분하지 않으므로 Tensor Bin으로 표현합니다.
"""

from __future__ import annotations

import argparse

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from _course_common import radial_root


MAIN_CLIP_START = 9015
MAIN_CLIP_END = 9142
DEFAULT_SAMPLE_ID = 9050

EPSILON = 1e-12


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Strong-response Threshold
#
# np.percentile(data, q)
# → 전체 데이터에서 q percentile에 해당하는 값
#
# 기본값 99:
# Magnitude가 높은 상위 약 1% bin을 확인
# ------------------------------------------------------------

## TODO 1
DEFAULT_PERCENTILE = None


def reduce_to_2d(
    magnitude_db: np.ndarray,
) -> np.ndarray:

    # ------------------------------------------------------------
    # Practice 4에서 만든 dB Tensor의 추가 Channel Dimension을
    # 평균하여 2D Engineering Preview로 변환합니다.
    #
    # np.mean(axis=-1)
    # → 마지막 dimension의 평균
    # ------------------------------------------------------------

    view = magnitude_db

    if view.ndim >= 3:
        view = view.mean(
            axis=-1
        )

    view = np.squeeze(
        view
    )

    while view.ndim > 2:
        view = view.mean(
            axis=-1
        )

    if view.ndim != 2:
        raise ValueError(
            f"Could not reduce FFT tensor to 2D: {view.shape}"
        )

    return view


def main() -> None:
    check_completed(
        "Practice 1: DEFAULT_PERCENTILE",
        DEFAULT_PERCENTILE,
    )

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sample",
        type=int,
        default=DEFAULT_SAMPLE_ID,
    )

    parser.add_argument(
        "--percentile",
        type=float,
        default=DEFAULT_PERCENTILE,
    )

    args = parser.parse_args()

    if not MAIN_CLIP_START <= args.sample <= MAIN_CLIP_END:
        parser.error(
            f"--sample must be "
            f"{MAIN_CLIP_START}~{MAIN_CLIP_END}"
        )

    if not 0.0 < args.percentile < 100.0:
        parser.error(
            "--percentile must be between 0 and 100"
        )

    root = radial_root()

    path = (
        root
        / "radar_FFT"
        / f"fft_{args.sample:06d}.npy"
    )

    if not path.is_file():
        raise FileNotFoundError(
            f"FFT frame not found: {path}"
        )

    # ------------------------------------------------------------
    # Example. Complex FFT Tensor 불러오기
    #
    # np.load(): NumPy binary tensor 불러오기
    # ------------------------------------------------------------
    tensor = np.load(
        path,
        allow_pickle=False,
    )

    # ------------------------------------------------------------
    # Practice 2. Complex Magnitude
    #
    # np.abs(x)
    # → Complex Number a+jb의 magnitude sqrt(a²+b²)
    #
    # 어떤 데이터를 np.abs()에 넣어야 하는지 선택하세요.
    # ------------------------------------------------------------

    ## TODO 2
    magnitude_source = None

    check_completed(
        "Practice 2: magnitude_source",
        magnitude_source,
    )

    magnitude = np.abs(
        magnitude_source
    )

    # ------------------------------------------------------------
    # Practice 3. Magnitude → dB
    #
    # 20 * log10(|X|)
    #
    # np.maximum(x, EPSILON)
    # → log10(0) 문제를 방지하기 위한 최소값 적용
    # ------------------------------------------------------------

    ## TODO 3
    db_source = None

    check_completed(
        "Practice 3: db_source",
        db_source,
    )

    magnitude_db = 20.0 * np.log10(
        np.maximum(
            db_source,
            EPSILON,
        )
    )

    # 2D Preview
    view = reduce_to_2d(
        magnitude_db
    )

    # ------------------------------------------------------------
    # Practice 4. Percentile Threshold
    #
    # np.percentile(data, q)
    #
    # q = 99라면 전체 bin 중 magnitude가 높은
    # 상위 약 1%의 기준값을 계산합니다.
    # ------------------------------------------------------------

    ## TODO 4
    threshold_source = None

    check_completed(
        "Practice 4: threshold_source",
        threshold_source,
    )

    threshold_db = np.percentile(
        threshold_source,
        args.percentile,
    )

    strong_mask = (
        view >= threshold_db
    )

    strong_count = int(
        np.count_nonzero(
            strong_mask
        )
    )

    total_count = int(
        view.size
    )

    strong_ratio = (
        100.0
        * strong_count
        / total_count
    )

    # ------------------------------------------------------------
    # Check. Numeric FFT QA
    # ------------------------------------------------------------
    print("file        :", path)

    print(
        "tensor      :",
        tensor.shape,
        tensor.dtype,
        "complex=",
        np.iscomplexobj(tensor),
    )

    print(
        "2D view     :",
        view.shape,
    )

    print(
        "magnitude   : min/mean/max =",
        *(
            f"{v:.2f} dB"
            for v in (
                view.min(),
                view.mean(),
                view.max(),
            )
        ),
    )

    print(
        "threshold   :",
        f"{args.percentile:g} percentile = "
        f"{threshold_db:.2f} dB",
    )

    print(
        "strong bins :",
        f"{strong_count}/{total_count} "
        f"({strong_ratio:.2f}%)",
    )

    print(
        "axis notice : Tensor Bin, "
        "not calibrated Range/Doppler axis"
    )

    # ------------------------------------------------------------
    # Result 1. FFT Magnitude Map
    # ------------------------------------------------------------
    output_dir = (
        root.parent
        / "results"
        / "lab05_student"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    fig, ax = plt.subplots(
        figsize=(9.0, 5.2)
    )

    image = ax.imshow(
        view.T,
        origin="lower",
        aspect="auto",
        cmap="magma",
    )

    cbar = fig.colorbar(
        image,
        ax=ax,
    )

    cbar.set_label(
        "Mean FFT magnitude [dB]"
    )

    ax.set_xlabel(
        "Tensor bin 0"
    )

    ax.set_ylabel(
        "Tensor bin 1"
    )

    ax.set_title(
        f"RADIal FFT Magnitude — Sample {args.sample}"
    )

    fig.tight_layout()

    magnitude_path = (
        output_dir
        / f"sample_{args.sample}_fft_magnitude.png"
    )

    fig.savefig(
        magnitude_path,
        dpi=180,
    )

    plt.close(fig)

    # ------------------------------------------------------------
    # Result 2. Magnitude Histogram + Threshold
    # ------------------------------------------------------------
    fig, ax = plt.subplots(
        figsize=(8.0, 4.8)
    )

    ax.hist(
        view.ravel(),
        bins=80,
        alpha=0.85,
    )

    ax.axvline(
        threshold_db,
        linewidth=2.0,
        label=f"{args.percentile:g} percentile",
    )

    ax.set_xlabel(
        "FFT magnitude [dB]"
    )

    ax.set_ylabel(
        "Number of tensor bins"
    )

    ax.set_title(
        f"FFT Magnitude Distribution — Sample {args.sample}"
    )

    ax.grid(
        True,
        alpha=0.25,
    )

    ax.legend()

    fig.tight_layout()

    histogram_path = (
        output_dir
        / f"sample_{args.sample}_fft_histogram.png"
    )

    fig.savefig(
        histogram_path,
        dpi=180,
    )

    plt.close(fig)

    print()
    print(
        "FFT map     :",
        magnitude_path,
    )

    print(
        "Histogram   :",
        histogram_path,
    )

    print(
        "[PASS] Practice 05 completed"
    )


if __name__ == "__main__":
    main()


# ------------------------------------------------------------
# 실행
#
# python3 05_radial_fft_processing.py
#
#
# Mini Experiment 1. Threshold
#
# python3 05_radial_fft_processing.py --percentile 95
# python3 05_radial_fft_processing.py --percentile 99
# python3 05_radial_fft_processing.py --percentile 99.5
#
# Percentile이 증가할수록 Strong-response Bin 수가
# 어떻게 변하는지 비교하세요.
#
#
# Mini Experiment 2. Different Scene
#
# python3 05_radial_fft_processing.py --sample 9015
# python3 05_radial_fft_processing.py --sample 9050
# python3 05_radial_fft_processing.py --sample 9142
#
# Main Clip 안에서도 FFT Magnitude 분포가
# 어떻게 달라지는지 비교하세요.
# ------------------------------------------------------------
