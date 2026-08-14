#!/usr/bin/env python3
"""
Practice 11: ADAS System Evidence Collection

목표
- Camera / Radar / Fusion / ADAS Topic의 실행 상태를 확인합니다.
- 일정 시간 동안 Message Count와 Average Rate를 측정합니다.
- Fusion Mode와 마지막 ADAS Command를 확인합니다.
- Normal / Degraded 조건의 결과를 비교합니다.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from std_msgs.msg import String


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Evidence 수집 시간
#
# 10초 동안 각 Topic의 Message 수를 수집합니다.
# Average Rate = Message Count / Duration
# ------------------------------------------------------------

## TODO 1
DEFAULT_DURATION_SEC = None


# ------------------------------------------------------------
# Practice 2. System Validation 대상
#
# Camera / Radar / Fusion Status / Shadow Command가
# 모두 수신되어야 기본 Pipeline을 PASS로 판단합니다.
# ------------------------------------------------------------

## TODO 2
REQUIRED_SIGNALS = None


class EvidenceCollector(Node):

    def __init__(self, source: str) -> None:
        super().__init__("adas_evidence_collector")

        self.counts = Counter()
        self.modes = Counter()
        self.last_command = None

        # RADIal / CARLA에 따라 Sensor Topic만 변경
        if source == "radial":
            image_type = CompressedImage
            image_topic = "/radial/camera/image/compressed"
            radar_topic = "/radial/radar/points"

        else:
            image_type = Image
            image_topic = "/carla/hero/camera_front/image"
            radar_topic = "/carla/hero/radar/point_cloud"

        self.create_subscription(
            image_type,
            image_topic,
            lambda _: self.hit("camera"),
            10,
        )

        self.create_subscription(
            PointCloud2,
            radar_topic,
            lambda _: self.hit("radar"),
            10,
        )

        self.create_subscription(
            Vector3Stamped,
            "/adas/fused_target",
            lambda _: self.hit("target"),
            10,
        )

        self.create_subscription(
            String,
            "/adas/fusion/status",
            self.status,
            10,
        )

        self.create_subscription(
            String,
            "/adas/shadow_command",
            self.command,
            10,
        )

    def hit(self, name: str) -> None:
        self.counts[name] += 1

    def status(self, msg: String) -> None:
        self.hit("status")

        try:
            mode = json.loads(
                msg.data
            ).get(
                "mode",
                "unknown",
            )

            self.modes[mode] += 1

        except json.JSONDecodeError:
            self.modes[
                "invalid_json"
            ] += 1

    def command(self, msg: String) -> None:
        self.hit("shadow")

        try:
            self.last_command = json.loads(
                msg.data
            )

        except json.JSONDecodeError:
            self.last_command = {
                "error": "invalid_json"
            }


def main() -> int:
    check_completed(
        "Practice 1: DEFAULT_DURATION_SEC",
        DEFAULT_DURATION_SEC,
    )

    check_completed(
        "Practice 2: REQUIRED_SIGNALS",
        REQUIRED_SIGNALS,
    )

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--source",
        choices=("radial", "carla"),
        required=True,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SEC,
    )

    args = parser.parse_args()

    rclpy.init()

    node = EvidenceCollector(
        args.source
    )

    start = time.monotonic()

    while (
        rclpy.ok()
        and time.monotonic() - start < args.duration
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    elapsed = max(
        time.monotonic() - start,
        1e-6,
    )

    node.destroy_node()
    rclpy.shutdown()

    # ------------------------------------------------------------
    # Practice 3. Average Message Rate
    #
    # rate [Hz] = message_count / elapsed_time
    # ------------------------------------------------------------

    print()
    print("=" * 68)
    print("ADAS System Evidence")
    print("=" * 68)

    print(
        f"source   : {args.source}"
    )

    print(
        f"duration : {elapsed:.2f} s"
    )

    print()

    for name in REQUIRED_SIGNALS:

        count = node.counts[name]

        ## TODO 3
        rate_hz = None

        check_completed(
            "Practice 3: rate_hz",
            rate_hz,
        )

        print(
            f"[{'PASS' if count else 'FAIL'}] "
            f"{name:7s}: "
            f"{count:4d} messages | "
            f"{rate_hz:6.2f} Hz"
        )

    target_count = node.counts["target"]
    target_rate = (
        target_count / elapsed
    )

    print(
        f"[{'PASS' if target_count else 'NOT RUN'}] "
        f"target : "
        f"{target_count:4d} messages | "
        f"{target_rate:6.2f} Hz"
    )

    print()
    print(
        "fusion modes:",
        dict(node.modes),
    )

    print(
        "last shadow :",
        node.last_command,
    )

    passed = all(
        node.counts[name]
        for name in REQUIRED_SIGNALS
    )

    print()
    print(
        "[PASS] System evidence collected"
        if passed
        else "[FAIL] Required pipeline messages are missing"
    )

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(
        main()
    )


# ------------------------------------------------------------
# 실행 전제
#
# CARLA Server
# 07 Camera/Radar Bridge
# 09 Camera/Radar Fusion
# 10 ADAS Decision Shadow
#
# 가 실행 중이어야 합니다.
#
#
# Normal 상태
#
# python3 11_collect_system_evidence.py \
#   --source carla \
#   --duration 10
#
#
# Mini Experiment 1 — Camera Degraded
#
# Lab 09를 다음과 같이 실행:
#
# python3 09_camera_radar_fusion.py \
#   --force-camera-unavailable
#
# 다시 Evidence 수집 후:
# fusion modes / last shadow 비교
#
#
# Mini Experiment 2 — Fusion Stop
#
# Lab 09 종료 후 Evidence를 다시 수집합니다.
#
# target message와 ADAS 상태가 어떻게 변하는지 확인하세요.
#
#
# Mini Experiment 3 — RADIal
#
# 04B가 실행 중이라면:
#
# python3 11_collect_system_evidence.py \
#   --source radial \
#   --duration 10
# ------------------------------------------------------------
