#!/usr/bin/env python3
"""
Practice 12B: Lane Detection — BEV + Sliding Window + Polyfit

목표
- Camera Image를 Bird's-Eye View로 변환합니다.
- Sliding Window로 좌/우 Lane Pixel을 추적합니다.
- np.polyfit()을 이용해 곡선 차선을 근사합니다.
- Lane Center / Lateral Offset / Heading Error를 계산합니다.

12A와 비교
-----------
12A : Canny + Hough → Straight-line model
12B : BEV + Polyfit → Curved-lane model
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Canny Threshold
#
# cv2.Canny(image, low, high)
# 기본값: 40 / 120
# ------------------------------------------------------------

## TODO 1
CANNY_LOW = None
CANNY_HIGH = None


# ------------------------------------------------------------
# Practice 2. Sliding Window
#
# N_WINDOWS
# → BEV 영상을 세로 방향으로 몇 개 Window로 나눌지 설정
#
# WINDOW_MARGIN_PX
# → Lane Pixel 탐색 폭
#
# 기본값:
#   9 windows
#   ±55 pixel
# ------------------------------------------------------------

## TODO 2
N_WINDOWS = None
WINDOW_MARGIN_PX = None


# ------------------------------------------------------------
# Practice 3. Polynomial Order
#
# np.polyfit(y, x, order)
#
# order = 1 → 직선
# order = 2 → 2차 곡선
#
# 이번 실습은 곡선 차선을 표현하기 위해 2차식을 사용합니다.
# ------------------------------------------------------------

## TODO 3
POLY_ORDER = None


# Perspective Transform
ROI_TOP_Y = 0.58

SRC_TOP_LEFT = 0.43
SRC_TOP_RIGHT = 0.57

SRC_BOTTOM_LEFT = 0.24
SRC_BOTTOM_RIGHT = 0.76

DST_LEFT = 0.28
DST_RIGHT = 0.72

MIN_PIXELS_TO_RECENTER = 20
MIN_LANE_PIXELS = 80

LANE_WIDTH_M = 3.5
LOOKAHEAD_M = 12.0


def polynomial_x(
    coeff: np.ndarray,
    y,
):
    return np.polyval(
        coeff,
        y,
    )


class CarlaLaneDetectionPolyfit(Node):

    def __init__(self) -> None:

        check_completed(
            "Practice 1: CANNY_LOW",
            CANNY_LOW,
        )

        check_completed(
            "Practice 1: CANNY_HIGH",
            CANNY_HIGH,
        )

        check_completed(
            "Practice 2: N_WINDOWS",
            N_WINDOWS,
        )

        check_completed(
            "Practice 2: WINDOW_MARGIN_PX",
            WINDOW_MARGIN_PX,
        )

        check_completed(
            "Practice 3: POLY_ORDER",
            POLY_ORDER,
        )

        super().__init__(
            "carla_lane_detection_polyfit"
        )

        self.output_pub = self.create_publisher(
            PointStamped,
            "/carla/lane/center",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/carla/lane/polyfit_debug_image",
            10,
        )

        self.create_subscription(
            Image,
            "/carla/hero/camera_front/image",
            self.on_image,
            10,
        )

        self.get_logger().info(
            "BEV + Sliding Window + Polyfit lane detector started"
        )

    # --------------------------------------------------------
    # RGB → Canny
    # --------------------------------------------------------

    def make_binary(
        self,
        bgr: np.ndarray,
    ) -> np.ndarray:

        gray = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2GRAY,
        )

        blur = cv2.GaussianBlur(
            gray,
            (5, 5),
            0,
        )

        return cv2.Canny(
            blur,
            CANNY_LOW,
            CANNY_HIGH,
        )

    # --------------------------------------------------------
    # Perspective Transform
    # --------------------------------------------------------

    def perspective_transform(
        self,
        binary: np.ndarray,
    ):

        height, width = binary.shape

        src = np.float32([
            [
                SRC_TOP_LEFT * width,
                ROI_TOP_Y * height,
            ],
            [
                SRC_TOP_RIGHT * width,
                ROI_TOP_Y * height,
            ],
            [
                SRC_BOTTOM_RIGHT * width,
                height - 1,
            ],
            [
                SRC_BOTTOM_LEFT * width,
                height - 1,
            ],
        ])

        dst = np.float32([
            [DST_LEFT * width, 0],
            [DST_RIGHT * width, 0],
            [DST_RIGHT * width, height - 1],
            [DST_LEFT * width, height - 1],
        ])

        M = cv2.getPerspectiveTransform(
            src,
            dst,
        )

        Minv = cv2.getPerspectiveTransform(
            dst,
            src,
        )

        roi = np.zeros_like(
            binary
        )

        cv2.fillPoly(
            roi,
            [src.astype(np.int32)],
            255,
        )

        roi_edges = cv2.bitwise_and(
            binary,
            roi,
        )

        warped = cv2.warpPerspective(
            roi_edges,
            M,
            (width, height),
        )

        # 끊어진 차선 Edge를 약간 연결
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (3, 7),
        )

        warped = cv2.morphologyEx(
            warped,
            cv2.MORPH_CLOSE,
            kernel,
        )

        return warped, Minv

    # --------------------------------------------------------
    # Sliding Window + Polynomial Fitting
    # --------------------------------------------------------

    def fit_lane(
        self,
        binary_bev: np.ndarray,
    ):

        height, width = binary_bev.shape

        # BEV 하단 Histogram
        histogram = np.sum(
            binary_bev[
                height // 2:,
                :
            ] > 0,
            axis=0,
        )

        midpoint = width // 2

        left_hist = histogram[
            int(0.10 * width):
            midpoint
        ]

        right_hist = histogram[
            midpoint:
            int(0.90 * width)
        ]

        if (
            left_hist.max() <= 0
            or right_hist.max() <= 0
        ):
            return None

        left_base = (
            int(0.10 * width)
            + int(
                np.argmax(left_hist)
            )
        )

        right_base = (
            midpoint
            + int(
                np.argmax(right_hist)
            )
        )

        nonzero_y, nonzero_x = (
            binary_bev.nonzero()
        )

        nonzero_y = np.asarray(
            nonzero_y
        )

        nonzero_x = np.asarray(
            nonzero_x
        )

        window_height = (
            height // N_WINDOWS
        )

        left_current = left_base
        right_current = right_base

        left_indices = []
        right_indices = []

        for window in range(
            N_WINDOWS
        ):

            y_low = (
                height
                - (window + 1)
                * window_height
            )

            y_high = (
                height
                - window
                * window_height
            )

            good_left = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (
                    nonzero_x
                    >= left_current
                    - WINDOW_MARGIN_PX
                )
                & (
                    nonzero_x
                    < left_current
                    + WINDOW_MARGIN_PX
                )
            ).nonzero()[0]

            good_right = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (
                    nonzero_x
                    >= right_current
                    - WINDOW_MARGIN_PX
                )
                & (
                    nonzero_x
                    < right_current
                    + WINDOW_MARGIN_PX
                )
            ).nonzero()[0]

            left_indices.append(
                good_left
            )

            right_indices.append(
                good_right
            )

            if (
                len(good_left)
                > MIN_PIXELS_TO_RECENTER
            ):
                left_current = int(
                    np.mean(
                        nonzero_x[
                            good_left
                        ]
                    )
                )

            if (
                len(good_right)
                > MIN_PIXELS_TO_RECENTER
            ):
                right_current = int(
                    np.mean(
                        nonzero_x[
                            good_right
                        ]
                    )
                )

        left_indices = np.concatenate(
            left_indices
        )

        right_indices = np.concatenate(
            right_indices
        )

        if (
            len(left_indices)
            < MIN_LANE_PIXELS
            or len(right_indices)
            < MIN_LANE_PIXELS
        ):
            return None

        left_x = nonzero_x[
            left_indices
        ]

        left_y = nonzero_y[
            left_indices
        ]

        right_x = nonzero_x[
            right_indices
        ]

        right_y = nonzero_y[
            right_indices
        ]

        # np.polyfit(y, x, 2)
        # 여러 Lane Pixel을 하나의 2차 곡선으로 근사
        left_fit = np.polyfit(
            left_y,
            left_x,
            POLY_ORDER,
        )

        right_fit = np.polyfit(
            right_y,
            right_x,
            POLY_ORDER,
        )

        y_bottom = float(
            height - 1
        )

        y_mid = float(
            0.60 * height
        )

        left_bottom = float(
            polynomial_x(
                left_fit,
                y_bottom,
            )
        )

        right_bottom = float(
            polynomial_x(
                right_fit,
                y_bottom,
            )
        )

        left_mid = float(
            polynomial_x(
                left_fit,
                y_mid,
            )
        )

        right_mid = float(
            polynomial_x(
                right_fit,
                y_mid,
            )
        )

        lane_width_bottom = (
            right_bottom
            - left_bottom
        )

        lane_width_mid = (
            right_mid
            - left_mid
        )

        if not (
            0.20 * width
            <= lane_width_bottom
            <= 0.65 * width
        ):
            return None

        if not (
            0.15 * width
            <= lane_width_mid
            <= 0.65 * width
        ):
            return None

        confidence = min(
            1.0,
            (
                len(left_indices)
                + len(right_indices)
            ) / 1500.0,
        )

        return (
            left_fit,
            right_fit,
            confidence,
        )

    def on_image(
        self,
        msg: Image,
    ) -> None:

        channels = (
            4
            if msg.encoding.lower()
            in {
                "bgra8",
                "rgba8",
            }
            else 3
        )

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        expected = (
            msg.height
            * msg.width
            * channels
        )

        if (
            msg.height <= 0
            or msg.width <= 0
            or raw.size < expected
        ):
            return

        bgr = (
            raw[:expected]
            .reshape(
                msg.height,
                msg.width,
                channels,
            )
            [:, :, :3]
            .copy()
        )

        height, width = (
            bgr.shape[:2]
        )

        binary = self.make_binary(
            bgr
        )

        (
            binary_bev,
            Minv,
        ) = self.perspective_transform(
            binary
        )

        fitted = self.fit_lane(
            binary_bev
        )

        debug = bgr.copy()

        if fitted is None:

            cv2.putText(
                debug,
                "Lane not detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

            self.publish_debug(
                msg,
                debug,
            )

            return

        (
            left_fit,
            right_fit,
            confidence,
        ) = fitted

        y_bottom = float(
            height - 1
        )

        y_lookahead = float(
            0.60 * height
        )

        left_bottom = float(
            polynomial_x(
                left_fit,
                y_bottom,
            )
        )

        right_bottom = float(
            polynomial_x(
                right_fit,
                y_bottom,
            )
        )

        left_lookahead = float(
            polynomial_x(
                left_fit,
                y_lookahead,
            )
        )

        right_lookahead = float(
            polynomial_x(
                right_fit,
                y_lookahead,
            )
        )

        center_bottom = 0.5 * (
            left_bottom
            + right_bottom
        )

        center_lookahead = 0.5 * (
            left_lookahead
            + right_lookahead
        )

        lane_width_px = (
            right_bottom
            - left_bottom
        )

        metres_per_pixel = (
            LANE_WIDTH_M
            / max(
                lane_width_px,
                1.0,
            )
        )

        y_left_m = -(
            center_bottom
            - 0.5 * width
        ) * metres_per_pixel

        heading_error = math.atan2(
            -(
                center_lookahead
                - center_bottom
            )
            * metres_per_pixel,
            LOOKAHEAD_M,
        )

        # ----------------------------------------------------
        # ROS2 Lane Output
        # ----------------------------------------------------

        output = PointStamped(
            header=msg.header
        )

        output.header.frame_id = (
            "vehicle"
        )

        output.point.x = float(
            y_left_m
        )

        output.point.y = float(
            heading_error
        )

        output.point.z = float(
            confidence
        )

        self.output_pub.publish(
            output
        )

        # ----------------------------------------------------
        # Simple Result Overlay
        # ----------------------------------------------------

        plot_y = np.linspace(
            0,
            height - 1,
            height,
        )

        left_x = np.clip(
            polynomial_x(
                left_fit,
                plot_y,
            ),
            0,
            width - 1,
        )

        right_x = np.clip(
            polynomial_x(
                right_fit,
                plot_y,
            ),
            0,
            width - 1,
        )

        center_x = (
            0.5
            * (
                left_x
                + right_x
            )
        )

        bev_overlay = np.zeros(
            (
                height,
                width,
                3,
            ),
            dtype=np.uint8,
        )

        def draw_curve(
            xs,
            colour,
            thickness,
        ):
            points = np.column_stack(
                (
                    xs,
                    plot_y,
                )
            ).astype(
                np.int32
            )

            cv2.polylines(
                bev_overlay,
                [
                    points.reshape(
                        -1,
                        1,
                        2,
                    )
                ],
                False,
                colour,
                thickness,
            )

        draw_curve(
            left_x,
            (0, 255, 0),
            5,
        )

        draw_curve(
            right_x,
            (0, 255, 0),
            5,
        )

        draw_curve(
            center_x,
            (0, 0, 255),
            4,
        )

        camera_overlay = cv2.warpPerspective(
            bev_overlay,
            Minv,
            (
                width,
                height,
            ),
        )

        debug = cv2.addWeighted(
            debug,
            1.0,
            camera_overlay,
            0.85,
            0,
        )

        cv2.putText(
            debug,
            (
                f"offset={y_left_m:+.2f} m   "
                f"heading={math.degrees(heading_error):+.1f} deg"
            ),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        self.publish_debug(
            msg,
            debug,
        )

    def publish_debug(
        self,
        source: Image,
        bgr: np.ndarray,
    ) -> None:

        debug = Image(
            header=source.header,
            height=source.height,
            width=source.width,
            encoding="bgr8",
            is_bigendian=False,
            step=source.width * 3,
            data=bgr.tobytes(),
        )

        self.debug_pub.publish(
            debug
        )


def main() -> None:

    rclpy.init()

    node = (
        CarlaLaneDetectionPolyfit()
    )

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# ------------------------------------------------------------
# 실행
#
# python3 12b_carla_lane_detection_polyfit.py
#
# ros2 run rqt_image_view rqt_image_view
# → /carla/lane/polyfit_debug_image
#
# ros2 topic echo /carla/lane/center
#
#
# Mini Experiment 1
#
# N_WINDOWS
# 9 → 6 → 12
#
#
# Mini Experiment 2
#
# WINDOW_MARGIN_PX
# 55 → 35 → 80
#
#
# Mini Experiment 3
#
# POLY_ORDER
# 2 → 1
#
# 같은 곡선 구간에서 직선 모델과 곡선 모델의 차이를
# 비교하세요.
# ------------------------------------------------------------
