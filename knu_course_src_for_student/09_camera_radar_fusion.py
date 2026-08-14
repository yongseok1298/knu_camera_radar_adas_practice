#!/usr/bin/env python3
"""
Practice 09: CARLA Live Camera–Radar Late Fusion

목표
- CARLA Camera / Radar Topic을 동시에 Subscribe합니다.
- Camera 영상의 Health 상태를 계산합니다.
- Radar PointCloud에서 MIO를 선택합니다.
- Timestamp 기반으로 Camera와 Radar Frame을 Pairing합니다.
- Camera unavailable 상황의 Degraded Mode를 확인합니다.

※ Pixel-level association은 Practice 04A/04B에서 다룹니다.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque

import numpy as np
import rclpy

from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from _course_common import (
    choose_fusion_mode,
    image_health_confidence,
    select_mio,
)


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


def stamp_seconds(message) -> float:
    return (
        message.header.stamp.sec
        + message.header.stamp.nanosec * 1e-9
    )


# ------------------------------------------------------------
# Practice 1. CARLA Sensor Topic
# ------------------------------------------------------------

## TODO 1
CAMERA_TOPIC = None
RADAR_TOPIC = None


# ------------------------------------------------------------
# Practice 2. Radar PointCloud Field
#
# Lab 07 Contract:
# x, y, relative_speed_mps
# ------------------------------------------------------------

## TODO 2
RADAR_FIELDS = None


# 최근 Camera Frame 저장
CAMERA_BUFFER_SIZE = 40

# Radar가 먼저 Callback될 경우 Camera Frame을 기다리는 시간
PAIR_WAIT_SEC = 0.08

# Pending Radar 확인 주기
PAIR_TIMER_SEC = 0.01

# Camera Health 계산량 감소
IMAGE_SAMPLE_STRIDE = 8


class CameraRadarFusion(Node):

    def __init__(
        self,
        force_camera_unavailable: bool,
    ) -> None:

        check_completed(
            "Practice 1: CAMERA_TOPIC",
            CAMERA_TOPIC,
        )

        check_completed(
            "Practice 1: RADAR_TOPIC",
            RADAR_TOPIC,
        )

        check_completed(
            "Practice 2: RADAR_FIELDS",
            RADAR_FIELDS,
        )

        super().__init__(
            "camera_radar_fusion"
        )

        self.force_camera_unavailable = (
            force_camera_unavailable
        )

        # (timestamp, camera confidence)
        self.camera_buffer = deque(
            maxlen=CAMERA_BUFFER_SIZE
        )

        # Camera Pairing을 기다리는 Radar Frame
        self.pending_radars = deque()

        self.target_pub = self.create_publisher(
            Vector3Stamped,
            "/adas/fused_target",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/adas/fusion/status",
            10,
        )

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.on_camera,
            10,
        )

        self.create_subscription(
            PointCloud2,
            RADAR_TOPIC,
            self.on_radar,
            10,
        )

        self.create_timer(
            PAIR_TIMER_SEC,
            self.process_pending_radars,
        )

        self.get_logger().info(
            f"timestamp-aware fusion | "
            f"camera buffer={CAMERA_BUFFER_SIZE}"
        )

    def on_camera(
        self,
        msg: Image,
    ) -> None:

        camera_stamp_s = stamp_seconds(
            msg
        )

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
            msg.height <= 0
            or msg.width <= 0
            or raw.size < expected
        ):
            confidence = 0.0

        else:
            pixels = (
                raw[:expected]
                .reshape(
                    msg.height,
                    msg.width,
                    channels,
                )
                [
                    ::IMAGE_SAMPLE_STRIDE,
                    ::IMAGE_SAMPLE_STRIDE,
                    :3,
                ]
                .astype(float)
            )

            # ----------------------------------------------------
            # Practice 3. BGR → Grayscale
            #
            # Gray =
            # 0.114 B + 0.587 G + 0.299 R
            # ----------------------------------------------------

            ## TODO 3
            blue = None
            green = None
            red = None

            check_completed(
                "Practice 3: blue",
                blue,
            )

            check_completed(
                "Practice 3: green",
                green,
            )

            check_completed(
                "Practice 3: red",
                red,
            )

            gray = (
                0.114 * blue
                + 0.587 * green
                + 0.299 * red
            )

            confidence = (
                image_health_confidence(
                    gray
                )
            )

        self.camera_buffer.append(
            (
                camera_stamp_s,
                confidence,
            )
        )

        # Radar가 먼저 들어와 있었다면 바로 Pairing 재시도
        self.process_pending_radars()

    def on_radar(
        self,
        msg: PointCloud2,
    ) -> None:

        targets = []

        for point in point_cloud2.read_points(
            msg,
            field_names=RADAR_FIELDS,
            skip_nans=True,
        ):
            try:
                values = tuple(
                    point[name]
                    for name in RADAR_FIELDS
                )

            except (
                IndexError,
                TypeError,
                ValueError,
            ):
                values = tuple(point)

            targets.append(
                tuple(
                    map(
                        float,
                        values[:3],
                    )
                )
            )

        mio = select_mio(
            targets
        )

        # --------------------------------------------------------
        # Practice 4. Radar Timestamp
        #
        # stamp_seconds(msg)
        # → Radar Message timestamp [s]
        # --------------------------------------------------------

        ## TODO 4
        radar_stamp_s = None

        check_completed(
            "Practice 4: radar_stamp_s",
            radar_stamp_s,
        )

        radar_item = {
            "message": msg,
            "timestamp": radar_stamp_s,
            "targets": targets,
            "mio": mio,
            "received_wall_time": time.monotonic(),
        }

        # Camera Failure 실험에서는 바로 Radar-only 처리
        if self.force_camera_unavailable:
            self.publish_fusion(
                radar_item,
                camera_confidence=0.0,
                sync_delta_s=0.0,
            )
            return

        # 정상 상태에서는 Timestamp Pairing 대기
        self.pending_radars.append(
            radar_item
        )

        self.process_pending_radars()

    def nearest_camera_sample(
        self,
        radar_stamp_s: float,
    ):
        """Radar timestamp와 가장 가까운 Camera Frame 검색."""

        if not self.camera_buffer:
            return None

        return min(
            self.camera_buffer,
            key=lambda sample: abs(
                radar_stamp_s - sample[0]
            ),
        )

    def process_pending_radars(
        self,
    ) -> None:

        if not self.pending_radars:
            return

        remaining = deque()
        now_wall = time.monotonic()

        while self.pending_radars:

            radar_item = (
                self.pending_radars.popleft()
            )

            radar_stamp_s = (
                radar_item["timestamp"]
            )

            camera_sample = (
                self.nearest_camera_sample(
                    radar_stamp_s
                )
            )

            age_wall = (
                now_wall
                - radar_item[
                    "received_wall_time"
                ]
            )

            if camera_sample is None:

                if age_wall < PAIR_WAIT_SEC:
                    remaining.append(
                        radar_item
                    )
                    continue

                self.publish_fusion(
                    radar_item,
                    camera_confidence=0.0,
                    sync_delta_s=math.nan,
                )
                continue

            camera_stamp_s, confidence = (
                camera_sample
            )

            sync_delta_s = (
                radar_stamp_s
                - camera_stamp_s
            )

            # choose_fusion_mode()의 Sync 허용 범위: ±0.06 s
            if abs(sync_delta_s) <= 0.06:

                self.publish_fusion(
                    radar_item,
                    camera_confidence=confidence,
                    sync_delta_s=sync_delta_s,
                )
                continue

            # 같은 시점 Camera Callback이 아직 안 왔을 수 있음
            if age_wall < PAIR_WAIT_SEC:
                remaining.append(
                    radar_item
                )
                continue

            # 대기 후에도 Pairing 실패
            self.publish_fusion(
                radar_item,
                camera_confidence=confidence,
                sync_delta_s=sync_delta_s,
            )

        self.pending_radars.extend(
            remaining
        )

    def publish_fusion(
        self,
        radar_item,
        camera_confidence: float,
        sync_delta_s: float,
    ) -> None:

        msg = radar_item["message"]
        targets = radar_item["targets"]
        mio = radar_item["mio"]

        mode = choose_fusion_mode(
            camera_confidence,
            sync_delta_s,
            mio is not None,
        )

        status = {
            "source": "carla_live_late_fusion",
            "mode": mode,
            "valid": mio is not None,
            "camera_confidence": camera_confidence,
            "sync_delta_s": (
                sync_delta_s
                if math.isfinite(sync_delta_s)
                else None
            ),
            "radar_target_count": len(targets),
        }

        if mio:

            output = Vector3Stamped(
                header=msg.header
            )

            # Compact Fused Target Contract
            # x = Forward / Range
            # y = Lateral
            # z = Relative Speed
            output.vector.x = float(mio[0])
            output.vector.y = float(mio[1])
            output.vector.z = float(mio[2])

            self.target_pub.publish(
                output
            )

            status.update(
                range_m=float(mio[0]),
                y_left_m=float(mio[1]),
                relative_speed_mps=float(mio[2]),
            )

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    allow_nan=False,
                )
            )
        )


def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--force-camera-unavailable",
        action="store_true",
    )

    args = parser.parse_args()

    rclpy.init()

    node = CameraRadarFusion(
        args.force_camera_unavailable
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
# python3 09_camera_radar_fusion.py
#
# Fusion 상태:
# ros2 topic echo /adas/fusion/status
#
# Target:
# ros2 topic echo /adas/fused_target
#
#
# Mini Experiment 1 — Camera Unavailable
#
# python3 09_camera_radar_fusion.py \
#   --force-camera-unavailable
#
# fused → radar_only_camera_unavailable 변화를 확인하세요.
#
#
# Mini Experiment 2 — System Evidence
#
# python3 11_collect_system_evidence.py \
#   --source carla \
#   --duration 10
#
# Normal 상태에서는 radar_only_unsynchronized가
# 거의 발생하지 않는지 확인하세요.
# ------------------------------------------------------------
