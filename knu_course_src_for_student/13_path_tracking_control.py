#!/usr/bin/env python3
"""
Practice 13: Lane / Path Tracking Control

목표
- Practice 12B의 Lane Center를 Steering 입력으로 사용합니다.
- Lateral Offset + Heading Error로 Lookahead Target을 계산합니다.
- Pure Pursuit로 Steering Angle을 계산합니다.
- Lane Tracking과 ACC/AEB 종방향 상태를 함께 확인합니다.

실행 순서
---------
1. Shadow Mode
2. Lateral-only Live Control
3. Integrated Lane + ACC/AEB Control
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy

from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from _course_common import pure_pursuit_steering


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Control Period
#
# 0.05 sec → 20 Hz
# ------------------------------------------------------------

## TODO 1
CONTROL_PERIOD_SEC = None


# ------------------------------------------------------------
# Practice 2. Lane Validation
#
# Stale Timeout = 0.5 sec
# Minimum Confidence = 0.15
# ------------------------------------------------------------

## TODO 2
LANE_STALE_TIMEOUT_SEC = None
MIN_LANE_CONFIDENCE = None


# ------------------------------------------------------------
# Practice 3. Pure Pursuit Lookahead
#
# 기본값: 8 m
# ------------------------------------------------------------

## TODO 3
LOOKAHEAD_M = None


# ------------------------------------------------------------
# Target Speed
# ------------------------------------------------------------

LATERAL_ONLY_SPEED_MPS = 5.0

CRUISE_SPEED_MPS = 6.0
ACC_HOLD_SPEED_MPS = 5.0
ACC_BRAKE_SPEED_MPS = 3.0

# Longitudinal feedback
SPEED_DEADBAND_MPS = 0.15

THROTTLE_KP = 0.18
BRAKE_KP = 0.12

MAX_THROTTLE = 0.45
MAX_SERVICE_BRAKE = 0.30

FALLBACK_BRAKE = 0.40
AEB_BRAKE = 1.00

MAX_STEERING_RAD = 0.60


class PathTrackingController(Node):

    def __init__(
        self,
        apply_control: bool,
        lateral_only: bool,
        host: str,
        port: int,
    ) -> None:

        check_completed(
            "Practice 1: CONTROL_PERIOD_SEC",
            CONTROL_PERIOD_SEC,
        )

        check_completed(
            "Practice 2: LANE_STALE_TIMEOUT_SEC",
            LANE_STALE_TIMEOUT_SEC,
        )

        check_completed(
            "Practice 2: MIN_LANE_CONFIDENCE",
            MIN_LANE_CONFIDENCE,
        )

        check_completed(
            "Practice 3: LOOKAHEAD_M",
            LOOKAHEAD_M,
        )

        super().__init__(
            "path_tracking_controller"
        )

        self.apply_control = apply_control
        self.lateral_only = lateral_only

        self.last_lane = None
        self.last_lane_time = -math.inf

        self.adas = {
            "mode": "CRUISE",
            "brake_0_to_1": 0.0,
            "acceleration_mps2": 0.0,
        }

        self.vehicle = None

        self.command_pub = self.create_publisher(
            Twist,
            "/carla/hero/cmd_vel",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/vehicle/control/status",
            10,
        )

        self.create_subscription(
            PointStamped,
            "/carla/lane/center",
            self.on_lane,
            10,
        )

        self.create_subscription(
            String,
            "/adas/shadow_command",
            self.on_adas,
            10,
        )

        if apply_control:

            import carla

            client = carla.Client(
                host,
                port,
            )

            client.set_timeout(
                10.0
            )

            vehicles = [
                actor
                for actor
                in client.get_world()
                .get_actors()
                .filter("vehicle.*")
                if actor.attributes.get("role_name")
                == "knu_hero"
            ]

            if not vehicles:
                raise RuntimeError(
                    "knu_hero not found; run Lab 07 first"
                )

            self.vehicle = vehicles[0]

            self.get_logger().warning(
                "LIVE CONTROL ENABLED"
            )

        else:

            self.get_logger().warning(
                "SHADOW MODE"
            )

        self.create_timer(
            CONTROL_PERIOD_SEC,
            self.tick,
        )

    def on_lane(
        self,
        msg: PointStamped,
    ) -> None:

        self.last_lane = msg
        self.last_lane_time = (
            time.monotonic()
        )

    def on_adas(
        self,
        msg: String,
    ) -> None:

        try:
            self.adas = json.loads(
                msg.data
            )

        except json.JSONDecodeError:
            self.adas = {
                "mode": "FALLBACK_HOLD"
            }

    def longitudinal_target(
        self,
    ):

        if self.lateral_only:

            return (
                LATERAL_ONLY_SPEED_MPS,
                0.0,
                "LANE_TRACK_ONLY",
                "normal_speed_feedback",
            )

        adas_mode = str(
            self.adas.get(
                "mode",
                "CRUISE",
            )
        )

        if adas_mode == "AEB":

            return (
                0.0,
                AEB_BRAKE,
                "LANE_TRACK+AEB",
                "emergency_brake",
            )

        if adas_mode == "FALLBACK_HOLD":

            return (
                0.0,
                FALLBACK_BRAKE,
                "LANE_TRACK+FALLBACK_HOLD",
                "fallback_brake",
            )

        if adas_mode == "ACC_BRAKE":

            return (
                ACC_BRAKE_SPEED_MPS,
                0.0,
                "LANE_TRACK+ACC_BRAKE",
                "decelerate_only",
            )

        if adas_mode == "ACC_HOLD":

            return (
                ACC_HOLD_SPEED_MPS,
                0.0,
                "LANE_TRACK+ACC_HOLD",
                "normal_speed_feedback",
            )

        return (
            CRUISE_SPEED_MPS,
            0.0,
            "LANE_TRACK+CRUISE",
            "normal_speed_feedback",
        )

    def speed_feedback(
        self,
        current_speed_mps: float,
        target_speed_mps: float,
        strategy: str,
        hard_brake: float,
    ):

        if hard_brake > 0.05:

            return (
                0.0,
                hard_brake,
            )

        speed_error = (
            target_speed_mps
            - current_speed_mps
        )

        # ACC_BRAKE:
        # 목표속도까지 감속하되 다시 가속하지 않음
        if strategy == "decelerate_only":

            if (
                current_speed_mps
                > target_speed_mps
                + SPEED_DEADBAND_MPS
            ):

                brake = min(
                    MAX_SERVICE_BRAKE,
                    BRAKE_KP
                    * (
                        current_speed_mps
                        - target_speed_mps
                    ),
                )

                return (
                    0.0,
                    brake,
                )

            return (
                0.0,
                0.0,
            )

        # Normal speed feedback
        if (
            speed_error
            > SPEED_DEADBAND_MPS
        ):

            throttle = min(
                MAX_THROTTLE,
                THROTTLE_KP
                * speed_error,
            )

            return (
                throttle,
                0.0,
            )

        if (
            speed_error
            < -SPEED_DEADBAND_MPS
        ):

            brake = min(
                MAX_SERVICE_BRAKE,
                BRAKE_KP
                * (-speed_error),
            )

            return (
                0.0,
                brake,
            )

        return (
            0.0,
            0.0,
        )

    def tick(self) -> None:

        lane_age = (
            time.monotonic()
            - self.last_lane_time
        )

        stale = (
            lane_age
            > LANE_STALE_TIMEOUT_SEC
        )

        confidence = (
            0.0
            if self.last_lane is None
            else float(
                self.last_lane.point.z
            )
        )

        if (
            stale
            or confidence
            < MIN_LANE_CONFIDENCE
        ):

            target_speed = 0.0
            steering_rad = 0.0

            hard_brake = FALLBACK_BRAKE

            mode = "LANE_FALLBACK"
            strategy = "fallback_brake"

            lane_y_left = 0.0
            heading_error = 0.0
            target_y_left = 0.0

        else:

            lane_y_left = float(
                self.last_lane.point.x
            )

            heading_error = float(
                self.last_lane.point.y
            )

            # ----------------------------------------------------
            # Practice 4. Lookahead Target
            #
            # target_y =
            # lane_offset + lookahead * tan(heading_error)
            # ----------------------------------------------------

            ## TODO 4
            target_y_left = None

            check_completed(
                "Practice 4: target_y_left",
                target_y_left,
            )

            steering_rad = (
                pure_pursuit_steering(
                    LOOKAHEAD_M,
                    target_y_left,
                )
            )

            (
                target_speed,
                hard_brake,
                mode,
                strategy,
            ) = self.longitudinal_target()

        # Shadow command
        command = Twist()

        command.linear.x = float(
            target_speed
        )

        command.angular.z = float(
            steering_rad
        )

        self.command_pub.publish(
            command
        )

        current_speed = None
        throttle_cmd = 0.0
        brake_cmd = hard_brake

        # --------------------------------------------------------
        # Optional CARLA Live Control
        # --------------------------------------------------------

        if self.vehicle is not None:

            import carla

            velocity = (
                self.vehicle.get_velocity()
            )

            current_speed = math.sqrt(
                velocity.x * velocity.x
                + velocity.y * velocity.y
                + velocity.z * velocity.z
            )

            (
                throttle_cmd,
                brake_cmd,
            ) = self.speed_feedback(
                current_speed,
                target_speed,
                strategy,
                hard_brake,
            )

            # CARLA positive steer = right
            # Course positive y = left
            steer_cmd = max(
                -1.0,
                min(
                    1.0,
                    -steering_rad
                    / MAX_STEERING_RAD,
                ),
            )

            self.vehicle.apply_control(
                carla.VehicleControl(
                    throttle=float(
                        throttle_cmd
                    ),
                    brake=float(
                        brake_cmd
                    ),
                    steer=float(
                        steer_cmd
                    ),
                )
            )

        status = {
            "mode": mode,
            "target_speed_mps": float(
                target_speed
            ),
            "current_speed_mps": (
                None
                if current_speed is None
                else float(current_speed)
            ),
            "steering_rad": float(
                steering_rad
            ),
            "steering_deg": float(
                math.degrees(
                    steering_rad
                )
            ),
            "lane_confidence": float(
                confidence
            ),
            "throttle_0_to_1": float(
                throttle_cmd
            ),
            "brake_0_to_1": float(
                brake_cmd
            ),
            "actuator_connected": bool(
                self.apply_control
            ),
            "lateral_only": bool(
                self.lateral_only
            ),
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status,
                    allow_nan=False,
                )
            )
        )

    def destroy_node(
        self,
    ) -> bool:

        if (
            self.vehicle is not None
            and self.vehicle.is_alive
        ):

            import carla

            self.vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    brake=1.0,
                    steer=0.0,
                )
            )

        return super().destroy_node()


def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--apply-control",
        action="store_true",
    )

    parser.add_argument(
        "--lateral-only",
        action="store_true",
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=2000,
    )

    args = parser.parse_args()

    rclpy.init()

    node = PathTrackingController(
        apply_control=args.apply_control,
        lateral_only=args.lateral_only,
        host=args.host,
        port=args.port,
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
# 실행 순서
#
# 1. Shadow Mode
#
# python3 13_path_tracking_control.py
#
# ros2 topic echo /vehicle/control/status
#
#
# 2. Lane Tracking Only
#
# python3 13_path_tracking_control.py \
#   --apply-control \
#   --lateral-only
#
# 약 5 m/s의 저속으로 Lane Tracking 확인
#
#
# 3. Integrated Control
#
# python3 13_path_tracking_control.py \
#   --apply-control
#
# Lane Tracking + ACC/AEB 동작 확인
#
#
# Mini Experiment
#
# LOOKAHEAD_M
# 5 → 8 → 12
#
# Steering 반응을 비교하세요.
# ------------------------------------------------------------
