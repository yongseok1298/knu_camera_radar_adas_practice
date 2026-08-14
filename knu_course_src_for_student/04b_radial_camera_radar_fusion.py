#!/usr/bin/env python3
"""
Practice 04B: Camera–Radar Association & Fused Target Selection

목표
- Camera annotation과 실제 Radar Point를 association합니다.
- 서로 다른 좌표계 convention을 맞춥니다.
- Camera BBox와 Radar measurement를 하나의 candidate로 구성합니다.
- Fused Target(MIO)의 Range / Lateral Position / Relative Speed를 확인합니다.

※ Camera detection 성능의 영향을 제거하기 위해
   이번 실습에서는 RADIal annotation을 detection proxy로 사용합니다.
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from _course_common import (
    load_labels,
    load_radial_pcl,
    nearest_radar_point,
    radial_root,
    select_fused_mio,
)


MAIN_CLIP_START = 9015
MAIN_CLIP_END = 9142

REPLAY_PERIOD_SEC = 0.2
DIFFICULT_CONFIDENCE = 0.6


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Radar PCL Feature
#
# column 0 : x_forward
# column 1 : y_left
# column 2 : z_up
# column 3 : relative_speed
# ------------------------------------------------------------

## TODO 1
RADAR_FORWARD_COL = None
RADAR_LATERAL_COL = None
RADAR_SPEED_COL = None


class RadialFusionReplay(Node):

    def __init__(self) -> None:
        check_completed(
            "Practice 1: RADAR_FORWARD_COL",
            RADAR_FORWARD_COL,
        )
        check_completed(
            "Practice 1: RADAR_LATERAL_COL",
            RADAR_LATERAL_COL,
        )
        check_completed(
            "Practice 1: RADAR_SPEED_COL",
            RADAR_SPEED_COL,
        )

        super().__init__(
            "radial_camera_radar_fusion"
        )

        self.root = radial_root()

        self.labels = load_labels(
            self.root / "labels.csv"
        )

        self.samples = [
            sid
            for sid in sorted(self.labels)
            if MAIN_CLIP_START <= sid <= MAIN_CLIP_END
            and (
                self.root
                / "camera"
                / f"image_{sid:06d}.jpg"
            ).is_file()
            and (
                self.root
                / "radar_PCL"
                / f"pcl_{sid:06d}.npy"
            ).is_file()
        ]

        if not self.samples:
            raise FileNotFoundError(
                "Main Clip에서 Camera/Radar sample을 찾을 수 없습니다."
            )

        self.index = 0

        self.image_pub = self.create_publisher(
            CompressedImage,
            "/radial/camera/image/compressed",
            10,
        )

        self.radar_pub = self.create_publisher(
            PointCloud2,
            "/radial/radar/points",
            10,
        )

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

        self.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="relative_speed_mps",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        self.create_timer(
            REPLAY_PERIOD_SEC,
            self.tick,
        )

        self.get_logger().info(
            f"Main Clip {self.samples[0]}-{self.samples[-1]} | "
            f"{len(self.samples)} frames | "
            f"{1.0 / REPLAY_PERIOD_SEC:.1f} Hz"
        )

    def tick(self) -> None:
        sample_id = self.samples[self.index]

        self.index = (
            self.index + 1
        ) % len(self.samples)

        stamp = self.get_clock().now().to_msg()

        # --------------------------------------------------------
        # Camera Image
        # --------------------------------------------------------
        image = CompressedImage(
            header=self._header(
                stamp,
                "camera_front_optical",
            ),
            format="jpeg",
        )

        image.data = (
            self.root
            / "camera"
            / f"image_{sample_id:06d}.jpg"
        ).read_bytes()

        self.image_pub.publish(image)

        # --------------------------------------------------------
        # Radar PointCloud2
        # --------------------------------------------------------
        points = load_radial_pcl(
            self.root
            / "radar_PCL"
            / f"pcl_{sample_id:06d}.npy"
        )

        cloud = point_cloud2.create_cloud(
            Header(
                stamp=stamp,
                frame_id="vehicle",
            ),
            self.fields,
            points.tolist(),
        )

        self.radar_pub.publish(cloud)

        # --------------------------------------------------------
        # Practice 1 결과
        #
        # nearest_radar_point() 입력:
        # (x_forward, y_left, relative_speed)
        # --------------------------------------------------------
        radar_targets = [
            (
                float(p[RADAR_FORWARD_COL]),
                float(p[RADAR_LATERAL_COL]),
                float(p[RADAR_SPEED_COL]),
            )
            for p in points
        ]

        candidates = []

        for label in self.labels[sample_id]:

            # ----------------------------------------------------
            # Practice 2. Coordinate Convention
            #
            # RADIal Label:
            # radar_Y_m = Forward
            # radar_X_m = Right-positive
            #
            # Vehicle Frame:
            # x = Forward
            # y = Left-positive
            #
            # Right → Left 변환에는 부호 반전이 필요합니다.
            # ----------------------------------------------------

            ## TODO 2
            label_forward_m = None
            label_y_left_m = None

            check_completed(
                "Practice 2: label_forward_m",
                label_forward_m,
            )
            check_completed(
                "Practice 2: label_y_left_m",
                label_y_left_m,
            )

            # 가장 가까운 실제 Radar Detection과 연결
            matched = nearest_radar_point(
                label_forward_m,
                label_y_left_m,
                radar_targets,
            )

            if matched is None:
                continue

            # ----------------------------------------------------
            # Practice 3. Camera BBox Center
            #
            # center_x = (x1 + x2) / 2
            #
            # label key:
            # x1_pix, x2_pix
            # ----------------------------------------------------

            ## TODO 3
            bbox_x1 = None
            bbox_x2 = None

            check_completed(
                "Practice 3: bbox_x1",
                bbox_x1,
            )
            check_completed(
                "Practice 3: bbox_x2",
                bbox_x2,
            )

            center_x = 0.5 * (
                bbox_x1 + bbox_x2
            )

            # ----------------------------------------------------
            # Practice 4. Camera-side Confidence
            #
            # label.get(key, default)
            # → key가 없으면 default 반환
            #
            # Difficult > 0 : confidence = 0.6
            # Normal        : confidence = 1.0
            # ----------------------------------------------------

            ## TODO 4
            difficult = None

            check_completed(
                "Practice 4: difficult",
                difficult,
            )

            confidence = (
                DIFFICULT_CONFIDENCE
                if difficult > 0.0
                else 1.0
            )

            candidates.append(
                (
                    center_x,
                    *matched,
                    confidence,
                )
            )

        # --------------------------------------------------------
        # Fused Target / MIO Selection
        #
        # select_fused_mio()는 수업용 공통 함수로 제공합니다.
        # --------------------------------------------------------
        mio = select_fused_mio(
            candidates
        )

        status = {
            "sample_id": sample_id,
            "source": "radial_annotation_proxy",
            "mode": (
                "fused"
                if mio
                else "invalid_no_target"
            ),
            "valid": mio is not None,
            "sync_delta_s": 0.0,
            "candidate_count": len(candidates),
        }

        if mio:
            target = Vector3Stamped(
                header=self._header(
                    stamp,
                    "vehicle",
                )
            )

            # Compact target contract:
            # x = forward/range
            # y = lateral
            # z = relative speed
            target.vector.x = float(mio[0])
            target.vector.y = float(mio[1])
            target.vector.z = float(mio[2])

            self.target_pub.publish(
                target
            )

            status.update(
                range_m=float(mio[0]),
                y_left_m=float(mio[1]),
                relative_speed_mps=float(mio[2]),
                camera_confidence=float(mio[3]),
            )

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    allow_nan=False,
                )
            )
        )

        if self.index % 10 == 1:
            if mio:
                self.get_logger().info(
                    f"sample={sample_id} | "
                    f"candidates={len(candidates)} | "
                    f"range={mio[0]:.1f} m | "
                    f"y={mio[1]:.1f} m | "
                    f"rel_v={mio[2]:.1f} m/s | "
                    f"conf={mio[3]:.1f}"
                )
            else:
                self.get_logger().info(
                    f"sample={sample_id} | "
                    f"no fused target"
                )

    @staticmethod
    def _header(stamp, frame_id):
        return Header(
            stamp=stamp,
            frame_id=frame_id,
        )


def main() -> None:
    rclpy.init()

    node = RadialFusionReplay()

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
# Terminal 1
# python3 04b_radial_camera_radar_fusion.py
#
# Camera 확인
# rqt_image_view
# → /radial/camera/image/compressed
#
# Radar 확인
# rviz2
# → Fixed Frame: vehicle
# → PointCloud2: /radial/radar/points
#
# Fusion 상태
# ros2 topic echo /adas/fusion/status
#
# Fused Target
# ros2 topic echo /adas/fused_target
#
# Replay Rate
# ros2 topic hz /radial/camera/image/compressed
# ros2 topic hz /radial/radar/points
#
# Mini Experiment
# DIFFICULT_CONFIDENCE = 0.6 → 0.3 → 0.9
# /adas/fusion/status의 camera_confidence 변화를 비교하세요.
# ------------------------------------------------------------
