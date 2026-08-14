#!/usr/bin/env python3
"""
Practice 04: RADIal Camera–Radar Late Fusion

목표
- 동일 Frame의 Camera / Radar 데이터를 함께 사용합니다.
- Camera annotation과 실제 Radar Point를 연관시킵니다.
- 서로 다른 좌표계의 방향 정의를 맞춥니다.
- Fused Target(MIO)의 Range / Lateral Position / Relative Speed를 확인합니다.

※ 본 실습은 Camera annotation을 proxy로 사용하는 Late Fusion 실습입니다.
  실제 Radar-to-Camera geometric projection은 별도로 다룹니다.
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


# ------------------------------------------------------------
# 수업용 Main Continuous Clip
# Camera + Radar PCL + Radar FFT가 모두 연속으로 존재
# ------------------------------------------------------------
MAIN_CLIP_START = 9015
MAIN_CLIP_END = 9142

REPLAY_PERIOD_SEC = 0.2       # 5 Hz
DIFFICULT_CONFIDENCE = 0.6


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Radar PCL Feature 선택
#
# Radar PCL Contract:
# column 0 : x_forward [m]
# column 1 : y_left [m]
# column 2 : z_up [m]
# column 3 : relative_speed [m/s]
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

        root = radial_root()
        self.root = root

        self.labels = load_labels(
            root / "labels.csv"
        )

        # Main Clip 범위 안에서 Camera + Radar가 모두 존재하는 Frame만 선택
        self.samples = [
            sid
            for sid in sorted(self.labels)
            if MAIN_CLIP_START <= sid <= MAIN_CLIP_END
            and (
                root / "camera" / f"image_{sid:06d}.jpg"
            ).is_file()
            and (
                root / "radar_PCL" / f"pcl_{sid:06d}.npy"
            ).is_file()
        ]

        if not self.samples:
            raise FileNotFoundError(
                "Main Clip에서 matched Camera/Radar sample을 찾을 수 없습니다."
            )

        self.index = 0

        # Camera / Radar / Fusion 결과 Topic
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

        self.create_timer(
            REPLAY_PERIOD_SEC,
            self.tick,
        )

        self.get_logger().info(
            f"Main Clip {self.samples[0]}-{self.samples[-1]} | "
            f"{len(self.samples)} matched frames | "
            f"{1.0 / REPLAY_PERIOD_SEC:.1f} Hz"
        )

    def tick(self) -> None:

        sample_id = self.samples[self.index]

        self.index = (
            self.index + 1
        ) % len(self.samples)

        stamp = self.get_clock().now().to_msg()

        # --------------------------------------------------------
        # Camera Image Publish
        # 동일 tick에서 Camera와 Radar에 같은 ROS Time Stamp 사용
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
        # Radar PointCloud2 Publish
        # --------------------------------------------------------
        points = load_radial_pcl(
            self.root
            / "radar_PCL"
            / f"pcl_{sample_id:06d}.npy"
        )

        fields = [
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

        radar_cloud = point_cloud2.create_cloud(
            Header(
                stamp=stamp,
                frame_id="vehicle",
            ),
            fields,
            points.tolist(),
        )

        self.radar_pub.publish(
            radar_cloud
        )

        # --------------------------------------------------------
        # Practice 1 결과 사용
        #
        # nearest_radar_point()에 전달할 Radar Target 형식:
        #
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
            # Practice 2. Label 좌표계 → Vehicle 좌표계
            #
            # RADIal Label:
            #   radar_Y_m : Forward
            #   radar_X_m : Right-positive
            #
            # ROS Vehicle:
            #   x : Forward
            #   y : Left-positive
            #
            # Right-positive → Left-positive 이므로 부호가 바뀝니다.
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

            # 가장 가까운 실제 Radar Detection 검색
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
            # BBox:
            # x1_pix ---------------- x2_pix
            #
            # Center = (x1 + x2) / 2
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
            # Practice 4. Camera Confidence
            #
            # Difficult > 0 : 0.6
            # Normal         : 1.0
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

            # candidate:
            # camera center + Radar measurement + confidence
            candidates.append(
                (
                    center_x,
                    *matched,
                    confidence,
                )
            )

        # --------------------------------------------------------
        # Fused MIO Selection
        # _course_common.py에서 제공
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
        }

        if mio:
            target = Vector3Stamped(
                header=self._header(
                    stamp,
                    "vehicle",
                )
            )

            target.vector.x = float(mio[0])
            target.vector.y = float(mio[1])
            target.vector.z = float(mio[2])

            self.target_pub.publish(
                target
            )

            status.update(
                range_m=mio[0],
                y_left_m=mio[1],
                relative_speed_mps=mio[2],
                camera_confidence=mio[3],
            )

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    allow_nan=False,
                )
            )
        )

        # Terminal에서 결과를 너무 빠르게 출력하지 않도록
        # 약 2초 간격으로 Fusion 상태 확인
        if self.index % 10 == 1:

            if mio:
                self.get_logger().info(
                    f"sample={sample_id} | "
                    f"range={mio[0]:.1f} m | "
                    f"y_left={mio[1]:.1f} m | "
                    f"rel_speed={mio[2]:.1f} m/s | "
                    f"conf={mio[3]:.1f}"
                )
            else:
                self.get_logger().info(
                    f"sample={sample_id} | no fused target"
                )

    @staticmethod
    def _header(
        stamp,
        frame_id,
    ):
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
# 실행 방법
#
# Terminal 1
# python3 04_radial_camera_radar_fusion.py
#
# Terminal 2
# rqt_image_view /radial/camera/image/compressed
#
# Terminal 3
# rviz2
#
#   Fixed Frame = vehicle
#   PointCloud2 = /radial/radar/points
#   Channel Name = relative_speed_mps
#
# Fusion 결과 확인
# ros2 topic echo /adas/fusion/status
#
# Fused Target 확인
# ros2 topic echo /adas/fused_target
#
# Camera / Radar Rate 비교
# ros2 topic hz /radial/camera/image/compressed
# ros2 topic hz /radial/radar/points
#
# Mini Experiment
# DIFFICULT_CONFIDENCE = 0.6 → 0.3 또는 0.9
# 변경 후 /adas/fusion/status의 camera_confidence를 비교하세요.
# ------------------------------------------------------------
