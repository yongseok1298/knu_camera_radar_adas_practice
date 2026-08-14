#!/usr/bin/env python3
"""
Practice 03: RADIal Radar PointCloud2 Replay

목표
- RADIal Radar PCL을 ROS2 PointCloud2 Message로 변환합니다.
- PointCloud2의 field / frame_id 의미를 확인합니다.
- Radar Point Cloud를 ROS2 Topic으로 Replay합니다.
- ros2 topic hz와 RViz2로 실제 출력 결과를 확인합니다.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from _course_common import (
    load_radial_pcl,
    radial_root,
    sample_id_from_path,
)


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Radar PointCloud Topic
#
# ROS2 Topic 이름:
# /radial/radar/points
# ------------------------------------------------------------

## TODO 1
TOPIC_NAME = None


# ------------------------------------------------------------
# Practice 2. Coordinate Frame
#
# PointCloud2 Header의 frame_id는
# Point들이 어느 좌표계에서 표현되는지 나타냅니다.
#
# 이번 RADIal PCL은 차량 기준 좌표계를 사용합니다.
# frame_id = "vehicle"
# ------------------------------------------------------------

## TODO 2
FRAME_ID = None


# ------------------------------------------------------------
# Practice 3. 기본 Replay Frequency
#
# Timer period = 1 / frequency
# 이번 실습 기본값: 5 Hz
#
# 실행할 때 --hz 옵션으로 변경할 수도 있습니다.
# ------------------------------------------------------------

## TODO 3
DEFAULT_HZ = None


# ------------------------------------------------------------
# Practice 4. PointCloud2 Relative Speed Field
#
# PointField는 PointCloud2 내부의 각 Feature를 정의합니다.
#
# FLOAT32 = 4 byte
#
# x              : offset 0
# y              : offset 4
# z              : offset 8
# relative_speed : offset 12
#
# 마지막 Field의 이름과 byte offset을 완성하세요.
# ------------------------------------------------------------

## TODO 4
REL_SPEED_FIELD_NAME = None
REL_SPEED_OFFSET = None


class RadialPclReplay(Node):
    def __init__(self, hz: float) -> None:
        check_completed("Practice 1: TOPIC_NAME", TOPIC_NAME)
        check_completed("Practice 2: FRAME_ID", FRAME_ID)
        check_completed(
            "Practice 4: REL_SPEED_FIELD_NAME",
            REL_SPEED_FIELD_NAME,
        )
        check_completed(
            "Practice 4: REL_SPEED_OFFSET",
            REL_SPEED_OFFSET,
        )

        super().__init__("radial_pcl_replay")

        # RADIal Radar PCL 파일 목록
        self.files = sorted(
            (radial_root() / "radar_PCL").glob("pcl_*.npy")
        )

        if not self.files:
            raise FileNotFoundError(
                "RADIal radar_PCL files not found"
            )

        self.index = 0

        # create_publisher(type, topic, queue_depth)
        self.publisher = self.create_publisher(
            type(self)._message_type(),
            TOPIC_NAME,
            10,
        )

        # ----------------------------------------------------
        # PointCloud2 Field 구성
        #
        # Point 한 개:
        # [x, y, z, relative_speed_mps]
        # ----------------------------------------------------
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
                name=REL_SPEED_FIELD_NAME,
                offset=REL_SPEED_OFFSET,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        # 5 Hz → period = 1 / 5 = 0.2 sec
        self.create_timer(
            1.0 / hz,
            self.tick,
        )

        self.get_logger().info(
            f"loaded {len(self.files)} PCL frames | "
            f"topic={TOPIC_NAME} | "
            f"frame={FRAME_ID} | "
            f"rate={hz:g} Hz"
        )

    @staticmethod
    def _message_type():
        from sensor_msgs.msg import PointCloud2

        return PointCloud2

    def tick(self) -> None:
        # 현재 Radar Frame 선택
        path = self.files[self.index]

        # 다음 Frame으로 이동
        self.index = (
            self.index + 1
        ) % len(self.files)

        # Nx4 Radar Point Cloud
        points = load_radial_pcl(path)

        # Header:
        # stamp    → 현재 ROS2 Time
        # frame_id → PointCloud 좌표계
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=FRAME_ID,
        )

        # create_cloud(header, fields, points)
        # Python Point List → sensor_msgs/msg/PointCloud2
        cloud_message = point_cloud2.create_cloud(
            header,
            self.fields,
            points.tolist(),
        )

        self.publisher.publish(
            cloud_message
        )

        # 너무 많은 Log가 나오지 않도록 25 Frame마다 출력
        if self.index % 25 == 1:
            self.get_logger().info(
                f"sample={sample_id_from_path(path)} "
                f"points={len(points)}"
            )


def main() -> None:
    check_completed(
        "Practice 3: DEFAULT_HZ",
        DEFAULT_HZ,
    )

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_HZ,
        help="Radar PCL replay frequency [Hz]",
    )

    args = parser.parse_args()

    if args.hz <= 0.0:
        parser.error("--hz must be positive")

    rclpy.init()

    node = RadialPclReplay(
        args.hz
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
# 실행 방법
#
# Terminal 1
# ----------
# python3 03_radial_pointcloud_replay.py
#
#
# Terminal 2
# ----------
# ros2 topic list
# ros2 topic info /radial/radar/points
# ros2 topic hz /radial/radar/points
#
#
# PointCloud2 Message 구조 확인
# ---------------------------
# ros2 interface show sensor_msgs/msg/PointCloud2
#
#
# RViz2
# -----
# rviz2
#
# Fixed Frame:
#   vehicle
#
# Add:
#   PointCloud2
#
# Topic:
#   /radial/radar/points
#
#
# Mini Experiment
# ---------------
#
# 1) 2 Hz Replay
# python3 03_radial_pointcloud_replay.py --hz 2
#
# 2) 10 Hz Replay
# python3 03_radial_pointcloud_replay.py --hz 10
#
# 각 경우 ros2 topic hz로 실제 Frequency를 측정하고,
# PointCloud 재생 속도의 차이를 비교하세요.
# ------------------------------------------------------------
