#!/usr/bin/env python3
"""
Practice 16: Camera-Radar BBox Association

목표
----
- YOLO 2D Bounding Box와 Radar Projection 결과를 결합합니다.
- Radar pixel이 BBox 내부에 있는지 확인합니다.
- 하나의 BBox 안에 여러 Radar Point가 있으면
  가장 가까운 Radar return을 선택합니다.
- Camera class/confidence + Radar range/relative speed를
  하나의 Fused Candidate로 만듭니다.

Pipeline
--------
14 YOLO Detection
        +
15 Radar Projection
        ↓
Pixel-space Association
        ↓
Fused Candidate
"""

from __future__ import annotations

import json
from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import (
    Image,
    PointCloud2,
    PointField,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


def check_completed(
    name: str,
    value,
) -> None:

    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. "
            "해당 ## TODO를 확인하세요."
        )


# ============================================================
# Topic Contract
# ============================================================

DETECTION_TOPIC = (
    "/carla/object_detection_2d/bounding_box"
)

YOLO_DEBUG_TOPIC = (
    "/carla/object_detection_2d/debug_image"
)

PROJECTED_RADAR_TOPIC = (
    "/carla/radar/projected_points"
)

FUSED_TOPIC = (
    "/adas/fused_candidates"
)

STATUS_TOPIC = (
    "/adas/association/status"
)

DEBUG_TOPIC = (
    "/adas/association/debug_image"
)


# ============================================================
# Practice 1. Association Parameters
#
# BBOX_MARGIN_PX
# → Projection 오차를 고려해 BBox를 약간 확장
#
# MAX_PAIR_DELTA_SEC
# → Camera Detection과 Radar 간 최대 허용 시간차
#
# 기본값
# BBox margin = 12 pixel
# Sync delta  = 0.12 sec
# ============================================================

## TODO 1
BBOX_MARGIN_PX = None
MAX_PAIR_DELTA_SEC = None


BUFFER_SIZE = 30


# 현재 Course YOLO에서 Radar와 결합할 Object
ASSOCIATABLE_CLASSES = {
    "vehicle",
    "pedestrian",
}


def stamp_seconds(msg) -> float:

    stamp = msg.header.stamp

    return (
        float(stamp.sec)
        + float(stamp.nanosec)
        * 1e-9
    )


def image_to_bgr(
    msg: Image,
) -> np.ndarray | None:

    encoding = msg.encoding.lower()

    channels = (
        4
        if encoding in {
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
        return None

    image = (
        raw[:expected]
        .reshape(
            msg.height,
            msg.width,
            channels,
        )
    )

    if encoding == "bgra8":

        return cv2.cvtColor(
            image,
            cv2.COLOR_BGRA2BGR,
        )

    if encoding == "rgba8":

        return cv2.cvtColor(
            image,
            cv2.COLOR_RGBA2BGR,
        )

    if encoding == "rgb8":

        return cv2.cvtColor(
            image,
            cv2.COLOR_RGB2BGR,
        )

    return image[
        :, :, :3
    ].copy()


def read_projected_points(
    msg: PointCloud2,
) -> np.ndarray:
    """
    Lab 15 output columns

    0 : u_px
    1 : v_px
    2 : x_forward_m
    3 : y_left_m
    4 : z_up_m
    5 : relative_speed_mps
    """

    rows = []

    for point in point_cloud2.read_points(
        msg,
        field_names=(
            "u_px",
            "v_px",
            "x_forward_m",
            "y_left_m",
            "z_up_m",
            "relative_speed_mps",
        ),
        skip_nans=True,
    ):

        rows.append(
            [
                float(value)
                for value in point
            ]
        )

    if not rows:

        return np.empty(
            (0, 6),
            dtype=np.float32,
        )

    return np.asarray(
        rows,
        dtype=np.float32,
    )


def detection_info(
    detection,
) -> dict:

    bbox = detection.bbox

    cx = float(
        bbox.center.position.x
    )

    cy = float(
        bbox.center.position.y
    )

    width = float(
        bbox.size_x
    )

    height = float(
        bbox.size_y
    )

    class_name = "unknown"
    confidence = 0.0

    if detection.results:

        hypothesis = (
            detection
            .results[0]
            .hypothesis
        )

        class_name = str(
            hypothesis.class_id
        )

        confidence = float(
            hypothesis.score
        )

    return {
        "x1": cx - width / 2.0,
        "y1": cy - height / 2.0,
        "x2": cx + width / 2.0,
        "y2": cy + height / 2.0,
        "class_name": class_name,
        "confidence": confidence,
    }


def inside_bbox(
    u_px: float,
    v_px: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> bool:
    """
    Radar projection pixel (u,v)가
    Camera Bounding Box 내부에 있는지 검사합니다.
    """

    # ========================================================
    # Practice 2. Pixel Association Gate
    #
    # x1 <= u <= x2
    # y1 <= v <= y2
    # ========================================================

    ## TODO 2
    matched = None

    check_completed(
        "TODO 2: matched",
        matched,
    )

    return bool(
        matched
    )


class CameraRadarAssociation(Node):

    def __init__(self) -> None:

        check_completed(
            "TODO 1: BBOX_MARGIN_PX",
            BBOX_MARGIN_PX,
        )

        check_completed(
            "TODO 1: MAX_PAIR_DELTA_SEC",
            MAX_PAIR_DELTA_SEC,
        )

        super().__init__(
            "camera_radar_association"
        )

        self.detection_buffer = deque(
            maxlen=BUFFER_SIZE
        )

        self.image_buffer = deque(
            maxlen=BUFFER_SIZE
        )

        # ----------------------------------------------------
        # Inputs
        # ----------------------------------------------------

        self.create_subscription(
            Detection2DArray,
            DETECTION_TOPIC,
            self.on_detection,
            10,
        )

        self.create_subscription(
            Image,
            YOLO_DEBUG_TOPIC,
            self.on_debug_image,
            10,
        )

        self.create_subscription(
            PointCloud2,
            PROJECTED_RADAR_TOPIC,
            self.on_radar,
            10,
        )

        # ----------------------------------------------------
        # Outputs
        # ----------------------------------------------------

        self.fused_pub = (
            self.create_publisher(
                PointCloud2,
                FUSED_TOPIC,
                10,
            )
        )

        self.status_pub = (
            self.create_publisher(
                String,
                STATUS_TOPIC,
                10,
            )
        )

        self.debug_pub = (
            self.create_publisher(
                Image,
                DEBUG_TOPIC,
                10,
            )
        )

        self.get_logger().info(
            f"detection : {DETECTION_TOPIC}"
        )

        self.get_logger().info(
            f"radar     : {PROJECTED_RADAR_TOPIC}"
        )

        self.get_logger().info(
            f"output    : {FUSED_TOPIC}"
        )

    # ========================================================
    # Camera Buffers
    # ========================================================

    def on_detection(
        self,
        msg: Detection2DArray,
    ) -> None:

        self.detection_buffer.append(
            (
                stamp_seconds(msg),
                msg,
            )
        )

    def on_debug_image(
        self,
        msg: Image,
    ) -> None:

        image = image_to_bgr(
            msg
        )

        if image is None:
            return

        self.image_buffer.append(
            (
                stamp_seconds(msg),
                image,
                msg.header,
            )
        )

    def nearest_detection(
        self,
        timestamp: float,
    ):

        if not self.detection_buffer:
            return None

        return min(
            self.detection_buffer,
            key=lambda item:
                abs(
                    item[0]
                    - timestamp
                ),
        )

    def nearest_image(
        self,
        timestamp: float,
    ):

        if not self.image_buffer:
            return None

        return min(
            self.image_buffer,
            key=lambda item:
                abs(
                    item[0]
                    - timestamp
                ),
        )

    # ========================================================
    # Radar Association
    # ========================================================

    def on_radar(
        self,
        msg: PointCloud2,
    ) -> None:

        radar_stamp = (
            stamp_seconds(msg)
        )

        radar_points = (
            read_projected_points(
                msg
            )
        )

        nearest = (
            self.nearest_detection(
                radar_stamp
            )
        )

        if nearest is None:

            self.publish_empty(
                msg,
                len(radar_points),
                "no_detection",
            )

            return

        (
            detection_stamp,
            detection_msg,
        ) = nearest

        sync_delta = (
            radar_stamp
            - detection_stamp
        )

        if (
            abs(sync_delta)
            > MAX_PAIR_DELTA_SEC
        ):

            self.publish_empty(
                msg,
                len(radar_points),
                "unsynchronized",
                sync_delta,
            )

            return

        detections = []

        for index, detection in enumerate(
            detection_msg.detections
        ):

            info = detection_info(
                detection
            )

            info["index"] = index

            detections.append(
                info
            )

        # 높은 confidence부터 Association
        detections.sort(
            key=lambda item:
                item["confidence"],
            reverse=True,
        )

        fused_rows = []
        fused_meta = []

        used_radar = set()

        for info in detections:

            class_name = (
                info["class_name"]
                .strip()
                .lower()
            )

            if (
                class_name
                not in ASSOCIATABLE_CLASSES
            ):
                continue

            # ------------------------------------------------
            # Margin을 포함한 BBox
            # ------------------------------------------------

            x1 = (
                info["x1"]
                - BBOX_MARGIN_PX
            )

            y1 = (
                info["y1"]
                - BBOX_MARGIN_PX
            )

            x2 = (
                info["x2"]
                + BBOX_MARGIN_PX
            )

            y2 = (
                info["y2"]
                + BBOX_MARGIN_PX
            )

            candidate_indices = []

            for (
                radar_index,
                point,
            ) in enumerate(
                radar_points
            ):

                if radar_index in used_radar:
                    continue

                u_px = float(
                    point[0]
                )

                v_px = float(
                    point[1]
                )

                if inside_bbox(
                    u_px,
                    v_px,
                    x1,
                    y1,
                    x2,
                    y2,
                ):

                    candidate_indices.append(
                        radar_index
                    )

            if not candidate_indices:
                continue

            # =================================================
            # Practice 3. Radar Selection
            #
            # 한 BBox에 여러 Radar return이 있으면
            # x_forward_m이 가장 작은 Point를 선택합니다.
            #
            # radar_points[index, 2] = x_forward_m
            # =================================================

            ## TODO 3
            selected_index = None

            check_completed(
                "TODO 3: selected_index",
                selected_index,
            )

            used_radar.add(
                selected_index
            )

            selected = (
                radar_points[
                    selected_index
                ]
            )

            # ------------------------------------------------
            # Fused Candidate
            # ------------------------------------------------

            fused_rows.append(
                [
                    float(selected[2]),
                    float(selected[3]),
                    float(selected[4]),
                    float(selected[5]),
                    float(info["index"]),
                    float(info["confidence"]),
                    float(selected[0]),
                    float(selected[1]),
                ]
            )

            fused_meta.append(
                {
                    "class_name": (
                        info["class_name"]
                    ),
                    "confidence": float(
                        info["confidence"]
                    ),
                    "range_m": float(
                        selected[2]
                    ),
                    "y_left_m": float(
                        selected[3]
                    ),
                    "relative_speed_mps": float(
                        selected[5]
                    ),
                    "u_px": float(
                        selected[0]
                    ),
                    "v_px": float(
                        selected[1]
                    ),
                }
            )

        self.publish_fused(
            msg,
            fused_rows,
        )

        status = {
            "mode": "associated",
            "detections": len(
                detection_msg.detections
            ),
            "radar_points": len(
                radar_points
            ),
            "fused_candidates": len(
                fused_rows
            ),
            "sync_delta_ms": float(
                sync_delta
                * 1000.0
            ),
            "bbox_margin_px": float(
                BBOX_MARGIN_PX
            ),
            "candidates": fused_meta,
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status
                )
            )
        )

        self.publish_debug(
            radar_stamp,
            radar_points,
            fused_meta,
        )

    # ========================================================
    # Fused PointCloud2
    # ========================================================

    def publish_fused(
        self,
        source: PointCloud2,
        rows,
    ) -> None:

        names = [
            "x_forward_m",
            "y_left_m",
            "z_up_m",
            "relative_speed_mps",
            "detection_index",
            "detection_confidence",
            "u_px",
            "v_px",
        ]

        fields = [
            PointField(
                name=name,
                offset=index * 4,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            )
            for index, name
            in enumerate(names)
        ]

        header = source.header

        header.frame_id = (
            "vehicle"
        )

        cloud = (
            point_cloud2.create_cloud(
                header,
                fields,
                rows,
            )
        )

        self.fused_pub.publish(
            cloud
        )

    def publish_empty(
        self,
        source: PointCloud2,
        radar_count: int,
        mode: str,
        sync_delta=None,
    ) -> None:

        self.publish_fused(
            source,
            [],
        )

        status = {
            "mode": mode,
            "radar_points": int(
                radar_count
            ),
            "fused_candidates": 0,
            "sync_delta_ms": (
                None
                if sync_delta is None
                else float(
                    sync_delta * 1000.0
                )
            ),
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status
                )
            )
        )

    # ========================================================
    # Debug Image
    # ========================================================

    def publish_debug(
        self,
        timestamp: float,
        radar_points: np.ndarray,
        fused_meta,
    ) -> None:

        nearest = (
            self.nearest_image(
                timestamp
            )
        )

        if nearest is None:
            return

        (
            image_stamp,
            image,
            header,
        ) = nearest

        if (
            abs(
                timestamp
                - image_stamp
            )
            > MAX_PAIR_DELTA_SEC
        ):
            return

        # YOLO debug image를 그대로 사용
        debug = image.copy()

        # 모든 Projected Radar = Red
        for point in radar_points:

            cv2.circle(
                debug,
                (
                    int(point[0]),
                    int(point[1]),
                ),
                3,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )

        # Associated Radar = Cyan
        for candidate in fused_meta:

            u_px = int(
                candidate["u_px"]
            )

            v_px = int(
                candidate["v_px"]
            )

            cv2.circle(
                debug,
                (
                    u_px,
                    v_px,
                ),
                8,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                debug,
                (
                    f"{candidate['range_m']:.1f} m  "
                    f"{candidate['relative_speed_mps']:+.1f} m/s"
                ),
                (
                    u_px + 10,
                    max(
                        25,
                        v_px - 10,
                    ),
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            debug,
            (
                f"radar={len(radar_points)}  "
                f"fused={len(fused_meta)}"
            ),
            (20, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        output = Image()

        output.header = header

        output.height = int(
            debug.shape[0]
        )

        output.width = int(
            debug.shape[1]
        )

        output.encoding = "bgr8"
        output.is_bigendian = False

        output.step = (
            debug.shape[1]
            * 3
        )

        output.data = (
            debug.tobytes()
        )

        self.debug_pub.publish(
            output
        )


def main() -> None:

    rclpy.init()

    node = (
        CameraRadarAssociation()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# ============================================================
# 실행
#
# 07 + 08 + 14 + 15가 실행 중이어야 합니다.
#
# python3 16_carla_bbox_radar_association.py
#
#
# Debug Image
#
# ros2 run rqt_image_view rqt_image_view
#
# /adas/association/debug_image
#
#
# Association Status
#
# ros2 topic echo \
# /adas/association/status \
# --once
#
#
# Mini Experiment 1
#
# BBOX_MARGIN_PX
#
# 0 → 12 → 30
#
# Radar Association 성공률이 어떻게 달라지는지 비교합니다.
#
#
# Mini Experiment 2
#
# MAX_PAIR_DELTA_SEC
#
# 0.05 → 0.12 → 0.20
#
# Sensor synchronization 조건을 비교합니다.
# ============================================================
