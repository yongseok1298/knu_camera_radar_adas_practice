#!/usr/bin/env python3
"""
Lab 09: timestamp-aware CARLA Camera–Radar late fusion.

Camera와 Radar callback의 도착 순서가 서로 다를 수 있으므로,
Radar Message를 짧게 Pending 상태로 유지한 뒤 가장 가까운
Camera timestamp와 pairing합니다.

Camera:
    availability / health

Radar:
    metric target / relative velocity

Output:
    /adas/fused_target
    /adas/fusion/status

Simulator object ground truth는 사용하지 않습니다.
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


# 최근 Camera Frame 저장 개수
CAMERA_BUFFER_SIZE = 40

# Radar가 먼저 도착했을 때 Camera callback을 기다리는 시간
PAIR_WAIT_SEC = 0.08

# Pending Radar 확인 주기
PAIR_TIMER_SEC = 0.01


def stamp_seconds(message) -> float:
    """ROS2 Header timestamp → seconds."""

    return (
        message.header.stamp.sec
        + message.header.stamp.nanosec * 1e-9
    )


class CameraRadarFusion(Node):

    def __init__(
        self,
        force_camera_unavailable: bool,
    ) -> None:

        super().__init__(
            "camera_radar_fusion"
        )

        self.force_camera_unavailable = (
            force_camera_unavailable
        )

        # (camera_timestamp, camera_confidence)
        self.camera_buffer = deque(
            maxlen=CAMERA_BUFFER_SIZE
        )

        # 아직 Camera pairing을 기다리는 Radar frame
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
            "/carla/hero/camera_front/image",
            self.on_camera,
            10,
        )

        self.create_subscription(
            PointCloud2,
            "/carla/hero/radar/point_cloud",
            self.on_radar,
            10,
        )

        # Pending Radar를 주기적으로 확인
        self.create_timer(
            PAIR_TIMER_SEC,
            self.process_pending_radars,
        )

        self.get_logger().info(
            "timestamp-aware fusion started | "
            f"camera_buffer={CAMERA_BUFFER_SIZE} | "
            f"pair_wait={PAIR_WAIT_SEC:.2f} s"
        )

    # ============================================================
    # Camera
    # ============================================================

    def on_camera(
        self,
        msg: Image,
    ) -> None:

        camera_stamp_s = (
            stamp_seconds(msg)
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

            # 계산량을 줄이기 위해 8 pixel 간격 Sampling
            pixels = (
                raw[:expected]
                .reshape(
                    msg.height,
                    msg.width,
                    channels,
                )
                [::8, ::8, :3]
                .astype(float)
            )

            # BGRA/BGR → Grayscale proxy
            gray = (
                0.114 * pixels[..., 0]
                + 0.587 * pixels[..., 1]
                + 0.299 * pixels[..., 2]
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

        # Radar가 먼저 들어와 기다리고 있다면
        # Camera 도착 직후 다시 pairing 시도
        self.process_pending_radars()

    # ============================================================
    # Radar
    # ============================================================

    def on_radar(
        self,
        msg: PointCloud2,
    ) -> None:

        names = (
            "x",
            "y",
            "relative_speed_mps",
        )

        targets = []

        for point in point_cloud2.read_points(
            msg,
            field_names=names,
            skip_nans=True,
        ):

            try:

                values = tuple(
                    point[name]
                    for name in names
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

        radar_item = {
            "message": msg,
            "timestamp": stamp_seconds(msg),
            "targets": targets,
            "mio": mio,
            "received_wall_time": time.monotonic(),
        }

        # Camera Failure 실험에서는 기다릴 필요 없이
        # Radar-only mode로 즉시 처리
        if self.force_camera_unavailable:

            self.publish_fusion(
                radar_item,
                camera_confidence=0.0,
                sync_delta_s=0.0,
            )

            return

        # 정상 상태에서는 Camera pairing을 기다림
        self.pending_radars.append(
            radar_item
        )

        self.process_pending_radars()

    # ============================================================
    # Timestamp Pairing
    # ============================================================

    def nearest_camera_sample(
        self,
        radar_stamp_s: float,
    ):
        """Radar timestamp와 가장 가까운 Camera frame 검색."""

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
                - radar_item["received_wall_time"]
            )

            # ----------------------------------------------------
            # Camera가 아직 한 장도 없다면 잠깐 기다림
            # ----------------------------------------------------

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

            # ----------------------------------------------------
            # 충분히 가까운 Camera frame을 찾음
            # → 즉시 Fusion
            # ----------------------------------------------------

            if abs(sync_delta_s) <= 0.06:

                self.publish_fusion(
                    radar_item,
                    camera_confidence=confidence,
                    sync_delta_s=sync_delta_s,
                )

                continue

            # ----------------------------------------------------
            # 아직 같은 시점 Camera가 callback에 들어오지 않았을
            # 가능성이 있으므로 PAIR_WAIT_SEC까지 기다림
            # ----------------------------------------------------

            if age_wall < PAIR_WAIT_SEC:

                remaining.append(
                    radar_item
                )

                continue

            # 기다렸는데도 pairing 실패
            self.publish_fusion(
                radar_item,
                camera_confidence=confidence,
                sync_delta_s=sync_delta_s,
            )

        self.pending_radars.extend(
            remaining
        )

    # ============================================================
    # Fusion Output
    # ============================================================

    def publish_fusion(
        self,
        radar_item,
        camera_confidence: float,
        sync_delta_s: float,
    ) -> None:

        msg = radar_item[
            "message"
        ]

        targets = radar_item[
            "targets"
        ]

        mio = radar_item[
            "mio"
        ]

        mode = choose_fusion_mode(
            camera_confidence,
            sync_delta_s,
            mio is not None,
        )

        status = {
            "source": "carla_live_late_fusion",
            "mode": mode,
            "valid": mio is not None,
            "camera_confidence": (
                camera_confidence
            ),
            "sync_delta_s": (
                sync_delta_s
                if math.isfinite(sync_delta_s)
                else None
            ),
            "radar_target_count": len(
                targets
            ),
        }

        if mio:

            output = Vector3Stamped(
                header=msg.header
            )

            # Compact Fused Target Contract
            #
            # x = Forward / Range
            # y = Lateral Position
            # z = Relative Speed

            output.vector.x = float(
                mio[0]
            )

            output.vector.y = float(
                mio[1]
            )

            output.vector.z = float(
                mio[2]
            )

            self.target_pub.publish(
                output
            )

            status.update(
                range_m=float(
                    mio[0]
                ),
                y_left_m=float(
                    mio[1]
                ),
                relative_speed_mps=float(
                    mio[2]
                ),
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
