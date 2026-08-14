#!/usr/bin/env python3
"""
Challenge 19: GNSS Global Path Tracking + Pure Pursuit + PID

목표
----
Lab 18에서 생성한 Global Path를 실제 차량이 추종하도록 만듭니다.

횡방향 제어
-----------
GNSS Current Position
        ↓
Global Path
        ↓
Nearest Point
        ↓
Lookahead Target
        +
Camera Lane Correction
        ↓
Pure Pursuit
        ↓
Steering

종방향 제어
-----------
Target Speed
        ↓
PID
        ↓
Throttle / Brake

Input
-----
/carla/path/global
/carla/hero/gnss
/carla/lane/center

Output
------
/carla/path/tracking_target
/vehicle/control/status

기본은 SHADOW MODE입니다.

실제 제어:
python3 19_path_tracking_pid_control.py --apply-control
"""

from __future__ import annotations

import argparse
import json
import math
import time

import carla
import rclpy

from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
)
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from _course_common import (
    pure_pursuit_steering,
)


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
# Topics
# ============================================================

GLOBAL_PATH_TOPIC = (
    "/carla/path/global"
)

GNSS_TOPIC = (
    "/carla/hero/gnss"
)

LANE_TOPIC = (
    "/carla/lane/center"
)

LOOKAHEAD_TOPIC = (
    "/carla/path/tracking_target"
)

CONTROL_STATUS_TOPIC = (
    "/vehicle/control/status"
)


# ============================================================
# Course Contract
# ============================================================

EXPECTED_MAP = "Town04"

EGO_ROLE_NAME = "knu_hero"

MAP_FRAME = "map"


# ============================================================
# Challenge 1. Control Parameters
#
# Reference
# ---------
# Lookahead      : 8.0 m
# Cruise Speed   : 5.0 m/s
#
# PID
# Kp = 0.35
# Ki = 0.04
# Kd = 0.03
# ============================================================

## TODO 1
LOOKAHEAD_DISTANCE_M = None

CRUISE_SPEED_MPS = None

PID_KP = None
PID_KI = None
PID_KD = None


MAX_STEERING_RAD = 0.60

PID_INTEGRAL_LIMIT = 5.0

MAX_THROTTLE = 0.50

MAX_SERVICE_BRAKE = 0.40

SPEED_DEADBAND_MPS = 0.10


# ============================================================
# Camera Lane Correction
# ============================================================

USE_LANE_CORRECTION = True

LANE_CONFIDENCE_MIN = 0.15

LANE_STALE_TIMEOUT_SEC = 0.50

LANE_OFFSET_GAIN = 0.35

LANE_HEADING_GAIN = 0.35

MAX_LANE_CORRECTION_M = 0.75


# ============================================================
# Goal
# ============================================================

GOAL_SLOWDOWN_DISTANCE_M = 15.0

GOAL_STOP_DISTANCE_M = 2.0


# ============================================================
# Safety / Rate
# ============================================================

GNSS_STALE_TIMEOUT_SEC = 0.50

PATH_STALE_TIMEOUT_SEC = 2.0

CONTROL_PERIOD_SEC = 0.05


# ============================================================
# Helpers
# ============================================================

def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:

    return max(
        minimum,
        min(
            maximum,
            value,
        ),
    )


def carla_to_ros_xy(
    location: carla.Location,
):

    return (
        float(location.x),
        float(-location.y),
    )


def carla_yaw_to_ros_yaw(
    yaw_deg: float,
) -> float:

    return -math.radians(
        float(yaw_deg)
    )


def find_actor_by_role(
    world,
    role_name: str,
):

    for actor in (
        world
        .get_actors()
        .filter("vehicle.*")
    ):

        if (
            actor.attributes.get(
                "role_name"
            )
            == role_name
        ):
            return actor

    return None


def speed_mps(
    vehicle,
) -> float:

    velocity = (
        vehicle.get_velocity()
    )

    return math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )


def path_xy_from_msg(
    msg: Path,
):

    return [
        (
            float(
                pose.pose.position.x
            ),
            float(
                pose.pose.position.y
            ),
        )
        for pose in msg.poses
    ]


# ============================================================
# Challenge 2. Nearest Route Point
#
# 현재 GNSS 위치 (x,y)와
# Global Path의 모든 Point 사이 거리를 비교합니다.
#
# Hint:
#
# distance_sq =
#     (path_x - x)**2
#     + (path_y - y)**2
#
# 가장 작은 index를 반환하세요.
# ============================================================

def nearest_path_index(
    x: float,
    y: float,
    path_xy,
) -> int | None:

    if not path_xy:
        return None

    ## TODO 2
    best_index = None

    # Hint:
    #
    # best_distance_sq = float("inf")
    #
    # for index, (path_x, path_y) in enumerate(path_xy):
    #     ...
    #
    # return best_index

    check_completed(
        "TODO 2: best_index",
        best_index,
    )

    return best_index


# ============================================================
# Lookahead Point
# ============================================================

def lookahead_path_index(
    path_xy,
    start_index: int,
    lookahead_m: float,
) -> int:

    start_index = int(
        clamp(
            start_index,
            0,
            len(path_xy) - 1,
        )
    )

    travelled = 0.0

    previous = (
        path_xy[
            start_index
        ]
    )

    for index in range(
        start_index + 1,
        len(path_xy),
    ):

        current = (
            path_xy[
                index
            ]
        )

        travelled += math.hypot(
            current[0]
            - previous[0],
            current[1]
            - previous[1],
        )

        if (
            travelled
            >= lookahead_m
        ):
            return index

        previous = current

    return (
        len(path_xy)
        - 1
    )


# ============================================================
# Challenge 3. map → vehicle Coordinate Transform
#
# 입력
# ----
# Target : map frame
# Ego    : map frame
# Yaw    : map frame
#
# 출력
# ----
# x_forward
# y_left
#
# Hint
# ----
#
# dx = target_x - ego_x
# dy = target_y - ego_y
#
# x_forward =
#     cos(yaw)*dx + sin(yaw)*dy
#
# y_left =
#    -sin(yaw)*dx + cos(yaw)*dy
# ============================================================

def map_to_vehicle(
    target_x: float,
    target_y: float,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
):

    dx = (
        target_x
        - ego_x
    )

    dy = (
        target_y
        - ego_y
    )

    ## TODO 3
    x_forward = None
    y_left = None

    check_completed(
        "TODO 3: x_forward",
        x_forward,
    )

    check_completed(
        "TODO 3: y_left",
        y_left,
    )

    return (
        float(x_forward),
        float(y_left),
    )


# ============================================================
# Node
# ============================================================

class PathTrackingPidController(Node):

    def __init__(
        self,
        apply_control: bool,
        lane_correction: bool,
    ) -> None:

        check_completed(
            "TODO 1: LOOKAHEAD_DISTANCE_M",
            LOOKAHEAD_DISTANCE_M,
        )

        check_completed(
            "TODO 1: CRUISE_SPEED_MPS",
            CRUISE_SPEED_MPS,
        )

        check_completed(
            "TODO 1: PID_KP",
            PID_KP,
        )

        check_completed(
            "TODO 1: PID_KI",
            PID_KI,
        )

        check_completed(
            "TODO 1: PID_KD",
            PID_KD,
        )

        super().__init__(
            "path_tracking_pid_controller"
        )

        self.apply_control = (
            apply_control
        )

        self.use_lane_correction = (
            lane_correction
        )

        # ----------------------------------------------------
        # CARLA
        # ----------------------------------------------------

        self.client = carla.Client(
            "127.0.0.1",
            2000,
        )

        self.client.set_timeout(
            10.0
        )

        self.world = (
            self.client.get_world()
        )

        self.carla_map = (
            self.world.get_map()
        )

        if (
            EXPECTED_MAP
            not in self.carla_map.name
        ):

            raise RuntimeError(
                "Town04가 필요합니다."
            )

        self.ego = (
            find_actor_by_role(
                self.world,
                EGO_ROLE_NAME,
            )
        )

        if self.ego is None:

            raise RuntimeError(
                "knu_hero가 없습니다. "
                "Lab 07을 먼저 실행하세요."
            )

        # Traffic Manager와 제어권 충돌 방지
        self.ego.set_autopilot(
            False
        )

        # ----------------------------------------------------
        # State
        # ----------------------------------------------------

        self.path_xy = []

        self.path_time = (
            -math.inf
        )

        self.gnss_xy = None

        self.gnss_time = (
            -math.inf
        )

        self.gnss_yaw = None

        self.last_lane = None

        self.last_lane_time = (
            -math.inf
        )

        self.pid_integral = 0.0

        self.pid_previous_error = 0.0

        self.pid_previous_time = (
            time.monotonic()
        )

        self.goal_reached = False

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        self.create_subscription(
            Path,
            GLOBAL_PATH_TOPIC,
            self.on_path,
            10,
        )

        self.create_subscription(
            NavSatFix,
            GNSS_TOPIC,
            self.on_gnss,
            10,
        )

        self.create_subscription(
            PointStamped,
            LANE_TOPIC,
            self.on_lane,
            10,
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.lookahead_pub = (
            self.create_publisher(
                PoseStamped,
                LOOKAHEAD_TOPIC,
                10,
            )
        )

        self.status_pub = (
            self.create_publisher(
                String,
                CONTROL_STATUS_TOPIC,
                10,
            )
        )

        self.create_timer(
            CONTROL_PERIOD_SEC,
            self.control_loop,
        )

        if self.apply_control:

            self.get_logger().warning(
                "LIVE CONTROL ENABLED"
            )

        else:

            self.get_logger().warning(
                "SHADOW MODE"
            )

    # ========================================================
    # Global Path
    # ========================================================

    def on_path(
        self,
        msg: Path,
    ) -> None:

        path_xy = (
            path_xy_from_msg(
                msg
            )
        )

        if (
            len(path_xy)
            < 2
        ):
            return

        self.path_xy = path_xy

        self.path_time = (
            time.monotonic()
        )

        self.goal_reached = False

    # ========================================================
    # GNSS
    # ========================================================

    def on_gnss(
        self,
        msg: NavSatFix,
    ) -> None:

        geo = carla.GeoLocation(
            latitude=float(
                msg.latitude
            ),
            longitude=float(
                msg.longitude
            ),
            altitude=float(
                msg.altitude
            ),
        )

        converted = (
            self.carla_map
            .geolocation_to_transform(
                geo
            )
        )

        if hasattr(
            converted,
            "location",
        ):

            location = (
                converted.location
            )

        else:

            location = converted

        new_xy = (
            carla_to_ros_xy(
                location
            )
        )

        # GNSS 위치 변화로 Heading 추정
        if (
            self.gnss_xy
            is not None
        ):

            dx = (
                new_xy[0]
                - self.gnss_xy[0]
            )

            dy = (
                new_xy[1]
                - self.gnss_xy[1]
            )

            movement = (
                math.hypot(
                    dx,
                    dy,
                )
            )

            if movement >= 0.05:

                self.gnss_yaw = (
                    math.atan2(
                        dy,
                        dx,
                    )
                )

        self.gnss_xy = new_xy

        self.gnss_time = (
            time.monotonic()
        )

    # ========================================================
    # Lane Detection
    # ========================================================

    def on_lane(
        self,
        msg: PointStamped,
    ) -> None:

        self.last_lane = msg

        self.last_lane_time = (
            time.monotonic()
        )

    # ========================================================
    # Challenge 4. PID
    #
    # PID:
    #
    # error = target - current
    #
    # integral += error * dt
    #
    # derivative =
    #     (error - previous_error) / dt
    #
    # output =
    #     Kp*error
    #     + Ki*integral
    #     + Kd*derivative
    # ========================================================

    def pid_control(
        self,
        target_speed: float,
        current_speed: float,
    ):

        now = (
            time.monotonic()
        )

        dt = (
            now
            - self.pid_previous_time
        )

        dt = clamp(
            dt,
            0.001,
            0.20,
        )

        error = (
            target_speed
            - current_speed
        )

        self.pid_integral += (
            error
            * dt
        )

        self.pid_integral = (
            clamp(
                self.pid_integral,
                -PID_INTEGRAL_LIMIT,
                PID_INTEGRAL_LIMIT,
            )
        )

        derivative = (
            (
                error
                - self.pid_previous_error
            )
            / dt
        )

        # ====================================================
        # TODO 4
        # ====================================================

        pid_output = None

        check_completed(
            "TODO 4: pid_output",
            pid_output,
        )

        self.pid_previous_error = (
            error
        )

        self.pid_previous_time = (
            now
        )

        if (
            abs(error)
            < SPEED_DEADBAND_MPS
        ):

            throttle = 0.0
            brake = 0.0

        elif pid_output >= 0.0:

            throttle = clamp(
                pid_output,
                0.0,
                MAX_THROTTLE,
            )

            brake = 0.0

        else:

            throttle = 0.0

            brake = clamp(
                -pid_output,
                0.0,
                MAX_SERVICE_BRAKE,
            )

        return (
            throttle,
            brake,
            error,
            pid_output,
        )

    # ========================================================
    # Control Loop
    # ========================================================

    def control_loop(
        self,
    ) -> None:

        now = (
            time.monotonic()
        )

        if (
            not self.path_xy
            or (
                now
                - self.path_time
                > PATH_STALE_TIMEOUT_SEC
            )
        ):

            self.safe_hold(
                "PATH_UNAVAILABLE"
            )

            return

        if (
            self.gnss_xy is None
            or (
                now
                - self.gnss_time
                > GNSS_STALE_TIMEOUT_SEC
            )
        ):

            self.safe_hold(
                "GNSS_UNAVAILABLE"
            )

            return

        ego_x = float(
            self.gnss_xy[0]
        )

        ego_y = float(
            self.gnss_xy[1]
        )

        # 정지 중에는 차량 Orientation 사용
        if (
            self.gnss_yaw
            is None
        ):

            ego_yaw = (
                carla_yaw_to_ros_yaw(
                    self.ego
                    .get_transform()
                    .rotation
                    .yaw
                )
            )

        else:

            ego_yaw = (
                self.gnss_yaw
            )

        # ----------------------------------------------------
        # Route Localization
        # ----------------------------------------------------

        nearest = (
            nearest_path_index(
                ego_x,
                ego_y,
                self.path_xy,
            )
        )

        if nearest is None:

            self.safe_hold(
                "PATH_SEARCH_FAILED"
            )

            return

        lookahead_index = (
            lookahead_path_index(
                self.path_xy,
                nearest,
                LOOKAHEAD_DISTANCE_M,
            )
        )

        (
            target_x,
            target_y,
        ) = (
            self.path_xy[
                lookahead_index
            ]
        )

        (
            target_x_forward,
            target_y_left,
        ) = map_to_vehicle(
            target_x,
            target_y,
            ego_x,
            ego_y,
            ego_yaw,
        )

        # ----------------------------------------------------
        # Challenge 5. Camera Lane Correction
        #
        # Global Path가 Primary Reference입니다.
        #
        # Lane Detection이 정상일 때만:
        #
        # correction =
        #     offset_gain * lane_offset
        #     +
        #     heading_gain
        #     * lookahead
        #     * tan(lane_heading)
        #
        # 이후 target_y_left에 더합니다.
        # ----------------------------------------------------

        lane_valid = False
        lane_correction = 0.0

        if (
            self.last_lane
            is not None
        ):

            lane_age = (
                now
                - self.last_lane_time
            )

            confidence = float(
                self.last_lane
                .point
                .z
            )

            lane_valid = (
                self.use_lane_correction
                and lane_age
                <= LANE_STALE_TIMEOUT_SEC
                and confidence
                >= LANE_CONFIDENCE_MIN
            )

            if lane_valid:

                lane_offset = float(
                    self.last_lane
                    .point
                    .x
                )

                lane_heading = float(
                    self.last_lane
                    .point
                    .y
                )

                # ============================================
                # TODO 5
                # ============================================

                lane_correction = None

                check_completed(
                    "TODO 5: lane_correction",
                    lane_correction,
                )

                lane_correction = (
                    clamp(
                        lane_correction,
                        -MAX_LANE_CORRECTION_M,
                        MAX_LANE_CORRECTION_M,
                    )
                )

                target_y_left += (
                    lane_correction
                )

        # ----------------------------------------------------
        # Pure Pursuit
        # ----------------------------------------------------

        if (
            target_x_forward
            <= 0.10
        ):

            steering_rad = 0.0

        else:

            steering_rad = (
                pure_pursuit_steering(
                    target_x_forward,
                    target_y_left,
                )
            )

        # ----------------------------------------------------
        # Goal Distance
        # ----------------------------------------------------

        goal_x, goal_y = (
            self.path_xy[-1]
        )

        goal_distance = (
            math.hypot(
                goal_x
                - ego_x,
                goal_y
                - ego_y,
            )
        )

        if (
            goal_distance
            <= GOAL_STOP_DISTANCE_M
        ):

            target_speed = 0.0

            self.goal_reached = True

            mode = "GOAL_REACHED"

        elif (
            goal_distance
            < GOAL_SLOWDOWN_DISTANCE_M
        ):

            ratio = (
                (
                    goal_distance
                    - GOAL_STOP_DISTANCE_M
                )
                / (
                    GOAL_SLOWDOWN_DISTANCE_M
                    - GOAL_STOP_DISTANCE_M
                )
            )

            ratio = clamp(
                ratio,
                0.0,
                1.0,
            )

            target_speed = (
                CRUISE_SPEED_MPS
                * ratio
            )

            mode = "GOAL_APPROACH"

        else:

            target_speed = (
                CRUISE_SPEED_MPS
            )

            mode = "PATH_TRACK"

        # ----------------------------------------------------
        # PID
        # ----------------------------------------------------

        current_speed = (
            speed_mps(
                self.ego
            )
        )

        (
            throttle,
            brake,
            speed_error,
            pid_output,
        ) = self.pid_control(
            target_speed,
            current_speed,
        )

        if self.goal_reached:

            throttle = 0.0

            brake = max(
                brake,
                0.60,
            )

        # CARLA steer:
        # positive = right
        #
        # Course:
        # positive = left
        steer_cmd = clamp(
            -steering_rad
            / MAX_STEERING_RAD,
            -1.0,
            1.0,
        )

        if self.apply_control:

            self.ego.apply_control(
                carla.VehicleControl(
                    throttle=float(
                        throttle
                    ),
                    brake=float(
                        brake
                    ),
                    steer=float(
                        steer_cmd
                    ),
                )
            )

        self.publish_lookahead(
            target_x,
            target_y,
        )

        progress = (
            100.0
            * nearest
            / max(
                len(self.path_xy)
                - 1,
                1,
            )
        )

        status = {
            "mode": mode,

            "actuator_connected": (
                self.apply_control
            ),

            "path_progress_percent": float(
                progress
            ),

            "goal_distance_m": float(
                goal_distance
            ),

            "steering_deg": float(
                math.degrees(
                    steering_rad
                )
            ),

            "lane_valid": bool(
                lane_valid
            ),

            "lane_correction_m": float(
                lane_correction
            ),

            "target_speed_mps": float(
                target_speed
            ),

            "current_speed_mps": float(
                current_speed
            ),

            "speed_error_mps": float(
                speed_error
            ),

            "pid_output": float(
                pid_output
            ),

            "throttle": float(
                throttle
            ),

            "brake": float(
                brake
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
    # Lookahead
    # ========================================================

    def publish_lookahead(
        self,
        x_map: float,
        y_map: float,
    ) -> None:

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            MAP_FRAME
        )

        msg.pose.position.x = float(
            x_map
        )

        msg.pose.position.y = float(
            y_map
        )

        msg.pose.position.z = 0.5

        msg.pose.orientation.w = 1.0

        self.lookahead_pub.publish(
            msg
        )

    # ========================================================
    # Fail-safe
    # ========================================================

    def safe_hold(
        self,
        reason: str,
    ) -> None:

        if self.apply_control:

            self.ego.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    brake=0.50,
                    steer=0.0,
                )
            )

        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "mode": reason,
                        "target_speed_mps": 0.0,
                        "throttle": 0.0,
                        "brake": 0.50,
                    }
                )
            )
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(
        self,
    ) -> bool:

        if (
            self.apply_control
            and self.ego is not None
            and self.ego.is_alive
        ):

            self.ego.apply_control(
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
        "--no-lane-correction",
        action="store_true",
    )

    args = parser.parse_args()

    rclpy.init()

    node = (
        PathTrackingPidController(
            apply_control=(
                args.apply_control
            ),
            lane_correction=(
                not args.no_lane_correction
            ),
        )
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
# Challenge 19 — Success Criteria
#
# [ ] Global Path 수신
# [ ] GNSS 위치 → Route Localization
# [ ] Nearest Route Point 계산
# [ ] Lookahead Target 계산
# [ ] map → vehicle 좌표 변환
# [ ] Pure Pursuit Steering
# [ ] Camera Lane Correction
# [ ] PID Speed Control
# [ ] Goal 접근 시 감속
# [ ] Goal에서 정지
#
#
# 실행 순서
# ---------
#
# Shadow:
#
# python3 19_path_tracking_pid_control.py
#
#
# Live:
#
# python3 19_path_tracking_pid_control.py \
#   --apply-control
#
#
# Map Path only:
#
# python3 19_path_tracking_pid_control.py \
#   --apply-control \
#   --no-lane-correction
#
#
# Mini Experiment
# ---------------
#
# LOOKAHEAD_DISTANCE_M
# 5 → 8 → 12
#
# PID_KP
# 0.2 → 0.35 → 0.6
# ============================================================
