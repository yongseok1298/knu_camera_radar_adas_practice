#!/usr/bin/env python3
"""
Practice 12A: Lane Detection — Canny + Hough

목표
- RGB Camera에서 Canny Edge를 추출합니다.
- ROI 안에서 Hough Line을 검출합니다.
- Ego Vehicle을 감싸는 좌/우 차선을 선택합니다.
- Lane Center / Lateral Offset / Heading Error를 확인합니다.
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
# Practice 2. ROI
#
# 영상 높이 대비 ROI 상단 위치
# 기본값: 0.52
# ------------------------------------------------------------

## TODO 2
ROI_TOP_RATIO = None


# ------------------------------------------------------------
# Practice 3. Hough Threshold
#
# cv2.HoughLinesP(..., threshold=...)
# 기본값: 18
# ------------------------------------------------------------

## TODO 3
HOUGH_THRESHOLD = None


GAUSSIAN_KERNEL = 5

ROI_TOP_LEFT_RATIO = 0.30
ROI_TOP_RIGHT_RATIO = 0.70

MIN_LINE_LENGTH = 18
MAX_LINE_GAP = 80
MIN_ABS_SLOPE = 0.10

LANE_WIDTH_M = 3.5
LOOKAHEAD_M = 12.0


def x_at_y(
    line: tuple[float, float],
    y: float,
) -> float:
    a, b = line
    return a * y + b


def fit_ego_lane(
    edges: np.ndarray,
):
    check_completed(
        "Practice 3: HOUGH_THRESHOLD",
        HOUGH_THRESHOLD,
    )

    height, width = edges.shape

    segments = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )

    if segments is None:
        return None

    center_x = 0.5 * width
    y_bottom = height - 1.0
    y_top = 0.58 * height

    left_segments = []
    right_segments = []

    for x1, y1, x2, y2 in segments[:, 0]:
        dy = float(y2 - y1)

        if abs(dy) < 8.0:
            continue

        dx = float(x2 - x1)
        slope_xy = dx / dy

        if abs(slope_xy) < MIN_ABS_SLOPE:
            continue

        # x = a*y + b
        a = slope_xy
        b = x1 - a * y1

        xb = a * y_bottom + b
        xt = a * y_top + b

        if not (
            -0.25 * width
            <= xb
            <= 1.25 * width
        ):
            continue

        item = {
            "xb": xb,
            "xt": xt,
            "length": math.hypot(dx, dy),
            "points": (x1, y1, x2, y2),
        }

        if (
            slope_xy < 0.0
            and xb < center_x
        ):
            left_segments.append(item)

        elif (
            slope_xy > 0.0
            and xb > center_x
        ):
            right_segments.append(item)

    if (
        not left_segments
        or not right_segments
    ):
        return None

    # 차량 중심에 가장 가까운 좌/우 후보
    best_left = max(
        left_segments,
        key=lambda item: item["xb"],
    )

    best_right = min(
        right_segments,
        key=lambda item: item["xb"],
    )

    cluster_margin_px = 90.0

    left_points = []
    right_points = []

    for item in left_segments:
        if abs(
            item["xb"]
            - best_left["xb"]
        ) <= cluster_margin_px:

            x1, y1, x2, y2 = item["points"]

            left_points.extend(
                [
                    (y1, x1),
                    (y2, x2),
                ]
            )

    for item in right_segments:
        if abs(
            item["xb"]
            - best_right["xb"]
        ) <= cluster_margin_px:

            x1, y1, x2, y2 = item["points"]

            right_points.extend(
                [
                    (y1, x1),
                    (y2, x2),
                ]
            )

    if (
        len(left_points) < 4
        or len(right_points) < 4
    ):
        return None

    # np.polyfit(y, x, 1)
    # 여러 Hough Segment를 대표하는 직선 계산
    left_fit = tuple(
        np.polyfit(
            *zip(*left_points),
            1,
        )
    )

    right_fit = tuple(
        np.polyfit(
            *zip(*right_points),
            1,
        )
    )

    left_bottom = x_at_y(
        left_fit,
        y_bottom,
    )

    right_bottom = x_at_y(
        right_fit,
        y_bottom,
    )

    left_top = x_at_y(
        left_fit,
        y_top,
    )

    right_top = x_at_y(
        right_fit,
        y_top,
    )

    lane_width_bottom = (
        right_bottom
        - left_bottom
    )

    lane_width_top = (
        right_top
        - left_top
    )

    if lane_width_bottom <= 40:
        return None

    if lane_width_top <= 10:
        return None

    if lane_width_bottom >= 0.95 * width:
        return None

    center_bottom = 0.5 * (
        left_bottom
        + right_bottom
    )

    if abs(
        center_bottom
        - center_x
    ) > 0.30 * width:
        return None

    support = (
        len(left_points)
        + len(right_points)
    )

    confidence = min(
        1.0,
        support / 60.0,
    )

    return {
        "left": left_fit,
        "right": right_fit,
        "confidence": confidence,
    }


class CarlaLaneDetectionCanny(Node):

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
            "Practice 2: ROI_TOP_RATIO",
            ROI_TOP_RATIO,
        )

        super().__init__(
            "carla_lane_detection_canny"
        )

        self.output_pub = self.create_publisher(
            PointStamped,
            "/carla/lane/canny_center",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/carla/lane/canny_debug_image",
            10,
        )

        self.create_subscription(
            Image,
            "/carla/hero/camera_front/image",
            self.on_image,
            10,
        )

    def on_image(
        self,
        msg: Image,
    ) -> None:

        channels = (
            4
            if msg.encoding.lower()
            in {"bgra8", "rgba8"}
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
            raw.size < expected
            or msg.height <= 0
            or msg.width <= 0
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

        height, width = bgr.shape[:2]

        # Grayscale → Blur → Canny
        gray = cv2.cvtColor(
            bgr,
            cv2.COLOR_BGR2GRAY,
        )

        blur = cv2.GaussianBlur(
            gray,
            (
                GAUSSIAN_KERNEL,
                GAUSSIAN_KERNEL,
            ),
            0,
        )

        edges = cv2.Canny(
            blur,
            CANNY_LOW,
            CANNY_HIGH,
        )

        # ROI
        roi_mask = np.zeros_like(
            edges
        )

        polygon = np.array(
            [[
                (0, height - 1),
                (
                    int(
                        ROI_TOP_LEFT_RATIO
                        * width
                    ),
                    int(
                        ROI_TOP_RATIO
                        * height
                    ),
                ),
                (
                    int(
                        ROI_TOP_RIGHT_RATIO
                        * width
                    ),
                    int(
                        ROI_TOP_RATIO
                        * height
                    ),
                ),
                (
                    width - 1,
                    height - 1,
                ),
            ]],
            dtype=np.int32,
        )

        cv2.fillPoly(
            roi_mask,
            polygon,
            255,
        )

        roi_edges = cv2.bitwise_and(
            edges,
            roi_mask,
        )

        fitted = fit_ego_lane(
            roi_edges
        )

        debug = bgr.copy()

        cv2.polylines(
            debug,
            polygon,
            True,
            (255, 180, 0),
            2,
        )

        # Canny ROI Preview
        edge_preview = cv2.cvtColor(
            roi_edges,
            cv2.COLOR_GRAY2BGR,
        )

        preview_w = int(
            width * 0.28
        )
        preview_h = int(
            height * 0.28
        )

        edge_preview = cv2.resize(
            edge_preview,
            (
                preview_w,
                preview_h,
            ),
        )

        debug[
            height - preview_h:height,
            0:preview_w,
        ] = edge_preview

        cv2.putText(
            debug,
            "Canny ROI",
            (
                8,
                height - preview_h + 22,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )

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

        left = fitted["left"]
        right = fitted["right"]
        confidence = fitted["confidence"]

        y_bottom = height - 1.0
        y_top = 0.58 * height

        left_bottom = x_at_y(
            left,
            y_bottom,
        )

        right_bottom = x_at_y(
            right,
            y_bottom,
        )

        left_top = x_at_y(
            left,
            y_top,
        )

        right_top = x_at_y(
            right,
            y_top,
        )

        center_bottom = 0.5 * (
            left_bottom
            + right_bottom
        )

        center_top = 0.5 * (
            left_top
            + right_top
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
                center_top
                - center_bottom
            ) * metres_per_pixel,
            LOOKAHEAD_M,
        )

        output = PointStamped(
            header=msg.header
        )

        output.header.frame_id = "vehicle"

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

        # Selected Lane
        cv2.line(
            debug,
            (
                int(left_bottom),
                int(y_bottom),
            ),
            (
                int(left_top),
                int(y_top),
            ),
            (0, 255, 0),
            5,
        )

        cv2.line(
            debug,
            (
                int(right_bottom),
                int(y_bottom),
            ),
            (
                int(right_top),
                int(y_top),
            ),
            (0, 255, 0),
            5,
        )

        # Lane Center
        cv2.line(
            debug,
            (
                int(center_bottom),
                int(y_bottom),
            ),
            (
                int(center_top),
                int(y_top),
            ),
            (0, 0, 255),
            5,
        )

        cv2.putText(
            debug,
            (
                f"offset={y_left_m:+.2f}m  "
                f"heading={math.degrees(heading_error):+.1f}deg  "
                f"conf={confidence:.2f}"
            ),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
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
        CarlaLaneDetectionCanny()
    )

    try:
        rclpy.spin(node)

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
# python3 12a_carla_lane_detection_canny.py
#
# ros2 run rqt_image_view rqt_image_view
# → /carla/lane/canny_debug_image
#
# ros2 topic echo /carla/lane/canny_center
#
#
# Mini Experiment
#
# CANNY_LOW / CANNY_HIGH
# 40/120 → 80/180
#
# ROI_TOP_RATIO
# 0.52 → 0.60
#
# HOUGH_THRESHOLD
# 18 → 30
#
# Edge 수와 Lane Detection 결과를 비교하세요.
# ------------------------------------------------------------
