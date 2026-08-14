#!/usr/bin/env python3
"""
Practice 10: ACC / AEB Shadow Decision

목표
- Camera–Radar Fusion 결과를 ADAS longitudinal decision으로 변환합니다.
- Range / Relative Speed / TTC의 관계를 이해합니다.
- CRUISE / ACC_HOLD / ACC_BRAKE / AEB 상태를 확인합니다.
- Sensor/Fusion 정보가 끊겼을 때 FALLBACK_HOLD를 확인합니다.

주의
----
SHADOW MODE입니다.
계산된 Acceleration / Brake Command는 실제 CARLA 차량에 적용하지 않습니다.
"""

from __future__ import annotations

import json
import math
import time

import rclpy

from geometry_msgs.msg import (
    AccelStamped,
    Vector3Stamped,
)
from rclpy.node import Node
from std_msgs.msg import String

from _course_common import (
    shadow_longitudinal_command,
    time_to_collision,
)


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Control Update Period
#
# create_timer(period, callback)
#
# 0.1 sec → 10 Hz
# ------------------------------------------------------------

## TODO 1
CONTROL_PERIOD_SEC = None


# ------------------------------------------------------------
# Practice 2. Target Stale Timeout
#
# 마지막 Target을 받은 뒤 0.5초 이상 새 데이터가 없으면
# 오래된 정보(stale)라고 판단합니다.
# ------------------------------------------------------------

## TODO 2
TARGET_STALE_TIMEOUT_SEC = None


class AdasDecisionShadow(Node):

    def __init__(self) -> None:

        check_completed(
            "Practice 1: CONTROL_PERIOD_SEC",
            CONTROL_PERIOD_SEC,
        )

        check_completed(
            "Practice 2: TARGET_STALE_TIMEOUT_SEC",
            TARGET_STALE_TIMEOUT_SEC,
        )

        super().__init__(
            "adas_decision_shadow"
        )

        self.last_target = None
        self.last_target_time = -math.inf
        self.fusion_mode = "not_received"

        # ADAS 판단 결과
        self.command_pub = self.create_publisher(
            String,
            "/adas/shadow_command",
            10,
        )

        # Longitudinal acceleration reference
        self.accel_pub = self.create_publisher(
            AccelStamped,
            "/adas/shadow_accel",
            10,
        )

        # Lab 09 Fusion 결과
        self.create_subscription(
            Vector3Stamped,
            "/adas/fused_target",
            self.on_target,
            10,
        )

        self.create_subscription(
            String,
            "/adas/fusion/status",
            self.on_status,
            10,
        )

        self.create_timer(
            CONTROL_PERIOD_SEC,
            self.tick,
        )

        self.get_logger().warning(
            "SHADOW MODE: output is NOT connected to CARLA actuators"
        )

    def on_target(
        self,
        msg: Vector3Stamped,
    ) -> None:

        self.last_target = msg
        self.last_target_time = (
            time.monotonic()
        )

    def on_status(
        self,
        msg: String,
    ) -> None:

        try:
            status = json.loads(
                msg.data
            )

            self.fusion_mode = status.get(
                "mode",
                "unknown",
            )

        except json.JSONDecodeError:

            self.fusion_mode = (
                "invalid_status_json"
            )

    def tick(self) -> None:

        # --------------------------------------------------------
        # Practice 3. Target Freshness
        #
        # time.monotonic()
        # → 시스템의 단조 증가 시간
        #
        # age = current_time - last_target_time
        # --------------------------------------------------------

        ## TODO 3
        target_age_sec = None

        check_completed(
            "Practice 3: target_age_sec",
            target_age_sec,
        )

        stale = (
            target_age_sec
            > TARGET_STALE_TIMEOUT_SEC
        )

        degraded = (
            self.fusion_mode.startswith("invalid")
            or stale
            or self.last_target is None
        )

        # --------------------------------------------------------
        # Invalid / Stale Target
        # --------------------------------------------------------
        if degraded:

            command = {
                "mode": "FALLBACK_HOLD",
                "acceleration_mps2": 0.0,
                "brake_0_to_1": 0.0,
                "ttc_s": None,
                "shadow_mode": True,
                "reason": "invalid_or_stale_target",
            }

        else:

            v = self.last_target.vector

            # ----------------------------------------------------
            # Practice 4. Fused Target Contract
            #
            # /adas/fused_target
            #
            # vector.x → Target Range / Forward Position [m]
            # vector.y → Lateral Position [m]
            # vector.z → Relative Speed [m/s]
            #
            # ADAS longitudinal decision에는
            # Range와 Relative Speed를 사용합니다.
            # ----------------------------------------------------

            ## TODO 4
            target_range_m = None
            relative_speed_mps = None

            check_completed(
                "Practice 4: target_range_m",
                target_range_m,
            )

            check_completed(
                "Practice 4: relative_speed_mps",
                relative_speed_mps,
            )

            # ----------------------------------------------------
            # TTC
            #
            # relative_speed < 0 → Target 접근 중
            #
            # TTC = Range / (-Relative Speed)
            #
            # time_to_collision() 함수는 공통 코드로 제공합니다.
            # ----------------------------------------------------

            ttc_s = time_to_collision(
                target_range_m,
                relative_speed_mps,
            )

            # ----------------------------------------------------
            # Teaching Policy
            #
            # TTC < 2 s
            #   → AEB
            #
            # TTC < 4 s OR Range < 15 m
            #   → ACC_BRAKE
            #
            # Range < 30 m
            #   → ACC_HOLD
            #
            # Otherwise
            #   → CRUISE
            # ----------------------------------------------------

            command = (
                shadow_longitudinal_command(
                    target_range_m,
                    relative_speed_mps,
                )
            )

            # JSON에는 Infinity를 넣지 않으므로 None 처리
            if math.isinf(
                float(command["ttc_s"])
            ):
                command["ttc_s"] = None

        # 현재 Fusion 상태도 함께 기록
        command["fusion_mode"] = (
            self.fusion_mode
        )

        # --------------------------------------------------------
        # ADAS Shadow Command
        # --------------------------------------------------------

        self.command_pub.publish(
            String(
                data=json.dumps(
                    command,
                    allow_nan=False,
                )
            )
        )

        # --------------------------------------------------------
        # Acceleration Reference
        # --------------------------------------------------------

        accel = AccelStamped()

        accel.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        accel.header.frame_id = (
            "vehicle"
        )

        accel.accel.linear.x = float(
            command[
                "acceleration_mps2"
            ]
        )

        self.accel_pub.publish(
            accel
        )


def main() -> None:

    rclpy.init()

    node = AdasDecisionShadow()

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
# 실행 전제
#
# CARLA Server
# 07_carla_camera_radar_bridge.py
# 08_carla_lead_vehicle_scenario.py
# 09_camera_radar_fusion.py
#
# 가 실행 중이어야 합니다.
#
#
# Practice 10 실행
#
# python3 10_adas_decision_shadow.py
#
#
# ADAS Command 확인
#
# ros2 topic echo /adas/shadow_command
#
#
# Acceleration 확인
#
# ros2 topic echo /adas/shadow_accel
#
#
# ------------------------------------------------------------
# Mini Experiment 1. Lead Vehicle Distance
# ------------------------------------------------------------
#
# Lab 08을 다음 거리로 다시 실행합니다.
#
# --distance 12
# --distance 20
# --distance 30
#
# 예상 상태를 먼저 생각한 뒤
# /adas/shadow_command를 확인하세요.
#
#
# ------------------------------------------------------------
# Mini Experiment 2. Fusion Data Loss
# ------------------------------------------------------------
#
# Lab 09를 Ctrl+C로 종료합니다.
#
# 0.5초 이상 지난 뒤:
#
# mode = FALLBACK_HOLD
#
# 로 변경되는지 확인하세요.
#
#
# ------------------------------------------------------------
# Mini Experiment 3. Camera Unavailable
# ------------------------------------------------------------
#
# Lab 09를 다음과 같이 실행합니다.
#
# python3 09_camera_radar_fusion.py \
#   --force-camera-unavailable
#
# Fusion Mode와 Shadow Command가 어떻게 연결되는지
# 확인하세요.
# ------------------------------------------------------------
