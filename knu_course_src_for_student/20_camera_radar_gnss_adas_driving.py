#!/usr/bin/env python3
"""
실습 20: Camera-Radar-GNSS 통합 ADAS 주행

목표
----
지금까지 구성한 인지·추적·경로계획·제어 기능을 하나의
ADAS 주행 시스템으로 연결합니다.

횡방향
------
Global Path
    +
GNSS Localization
    +
Camera Lane Correction
    ↓
Pure Pursuit
    ↓
Steering

종방향
------
Camera + Radar
    ↓
Association
    ↓
Tracking / MIO
    ↓
ACC / AEB
    ↓
Target Speed
    ↓
PID
    ↓
Throttle / Brake

AEB가 발생하면 일반 PID 제어보다 우선하여
직접 제동 명령을 적용합니다.

입력
----
/carla/path/global
/carla/hero/gnss
/carla/lane/center
/adas/fused_target
/adas/tracking/status
/adas/shadow_command

출력
----
/carla/path/tracking_target
/vehicle/control/status

실제 차량 제어
--------------
python3 20_camera_radar_gnss_adas_driving.py --apply-control
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
    Vector3Stamped,
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

MIO_TOPIC = (
    "/adas/fused_target"
)

TRACKING_STATUS_TOPIC = (
    "/adas/tracking/status"
)

ADAS_COMMAND_TOPIC = (
    "/adas/shadow_command"
)

LOOKAHEAD_TOPIC = (
    "/carla/path/tracking_target"
)

CONTROL_STATUS_TOPIC = (
    "/vehicle/control/status"
)


# ============================================================
# 공통 설정
# ============================================================

EXPECTED_MAP = "Town04"

EGO_ROLE_NAME = "knu_hero"

MAP_FRAME = "map"

CONTROL_PERIOD_SEC = 0.05


# ============================================================
# 횡방향 제어
# ============================================================

LOOKAHEAD_DISTANCE_M = 8.0

MAX_STEERING_RAD = 0.60


# Camera Lane Correction
USE_LANE_CORRECTION = True

LANE_CONFIDENCE_MIN = 0.15

LANE_STALE_TIMEOUT_SEC = 0.50

LANE_OFFSET_GAIN = 0.35

LANE_HEADING_GAIN = 0.35

MAX_LANE_CORRECTION_M = 0.75


# ============================================================
# 종방향 PID
# ============================================================

CRUISE_SPEED_MPS = 5.0

PID_KP = 0.35
PID_KI = 0.04
PID_KD = 0.03

PID_INTEGRAL_LIMIT = 5.0

MAX_THROTTLE = 0.50

MAX_SERVICE_BRAKE = 0.45

SPEED_DEADBAND_MPS = 0.10


# ============================================================
# ACC
# ============================================================

ACC_MIN_GAP_M = 6.0

ACC_TIME_HEADWAY_SEC = 1.5

ACC_GAP_GAIN = 0.25

ACC_REL_SPEED_GAIN = 0.80

ACC_BRAKE_MAX_SPEED_MPS = 3.0


# ============================================================
# AEB / Fail-safe
# ============================================================

AEB_BRAKE = 1.0

FAILSAFE_BRAKE = 0.50


# ============================================================
# Goal
# ============================================================

GOAL_SLOWDOWN_DISTANCE_M = 15.0

GOAL_STOP_DISTANCE_M = 2.0


# ============================================================
# 입력 데이터 Timeout
# ============================================================

GNSS_STALE_TIMEOUT_SEC = 0.50

PATH_STALE_TIMEOUT_SEC = 2.0

TRACKING_STATUS_TIMEOUT_SEC = 0.50

MIO_STALE_TIMEOUT_SEC = 0.50

ADAS_COMMAND_TIMEOUT_SEC = 0.50


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


def nearest_path_index(
    x: float,
    y: float,
    path_xy,
) -> int | None:

    if not path_xy:
        return None

    best_index = 0
    best_distance_sq = float(
        "inf"
    )

    for index, (
        path_x,
        path_y,
    ) in enumerate(
        path_xy
    ):

        distance_sq = (
            (path_x - x) ** 2
            + (path_y - y) ** 2
        )

        if (
            distance_sq
            < best_distance_sq
        ):

            best_distance_sq = (
                distance_sq
            )

            best_index = (
                index
            )

    return best_index


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

    cos_yaw = math.cos(
        ego_yaw
    )

    sin_yaw = math.sin(
        ego_yaw
    )

    x_forward = (
        cos_yaw
        * dx
        + sin_yaw
        * dy
    )

    y_left = (
        -sin_yaw
        * dx
        + cos_yaw
        * dy
    )

    return (
        x_forward,
        y_left,
    )


# ============================================================
# Node
# ============================================================

class IntegratedAdasDriving(Node):

    def __init__(
        self,
        apply_control: bool,
        ignore_adas: bool,
        lane_correction: bool,
    ) -> None:

        super().__init__(
            "camera_radar_gnss_adas_driving"
        )

        self.apply_control = (
            apply_control
        )

        self.ignore_adas = (
            ignore_adas
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
                "센서 구성 프로그램을 먼저 실행하세요."
            )

        # Ego 제어권은 이 프로그램이 사용합니다.
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

        self.gnss_yaw = None

        self.gnss_time = (
            -math.inf
        )

        self.last_lane = None

        self.last_lane_time = (
            -math.inf
        )

        self.mio = None

        self.mio_time = (
            -math.inf
        )

        self.tracking_status = {}

        self.tracking_status_time = (
            -math.inf
        )

        self.adas_command = {}

        self.adas_command_time = (
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

        self.create_subscription(
            Vector3Stamped,
            MIO_TOPIC,
            self.on_mio,
            10,
        )

        self.create_subscription(
            String,
            TRACKING_STATUS_TOPIC,
            self.on_tracking_status,
            10,
        )

        self.create_subscription(
            String,
            ADAS_COMMAND_TOPIC,
            self.on_adas_command,
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
                "차량 제어 활성화"
            )

        else:

            self.get_logger().warning(
                "제어 명령 계산만 수행"
            )

    # ========================================================
    # Input callbacks
    # ========================================================

    def on_path(
        self,
        msg: Path,
    ) -> None:

        new_path = (
            path_xy_from_msg(
                msg
            )
        )

        if len(new_path) < 2:
            return

        self.path_xy = new_path

        self.path_time = (
            time.monotonic()
        )

        self.goal_reached = False

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

    def on_lane(
        self,
        msg: PointStamped,
    ) -> None:

        self.last_lane = msg

        self.last_lane_time = (
            time.monotonic()
        )

    def on_mio(
        self,
        msg: Vector3Stamped,
    ) -> None:
        """
        실습 17 출력

        vector.x : tracked forward range [m]
        vector.y : lateral position, left positive [m]
        vector.z : tracked range-rate [m/s]

        vector.z < 0
        → Closing
        """

        self.mio = (
            float(
                msg.vector.x
            ),
            float(
                msg.vector.y
            ),
            float(
                msg.vector.z
            ),
        )

        self.mio_time = (
            time.monotonic()
        )

    def on_tracking_status(
        self,
        msg: String,
    ) -> None:

        try:

            self.tracking_status = (
                json.loads(
                    msg.data
                )
            )

        except json.JSONDecodeError:

            self.tracking_status = {}

        self.tracking_status_time = (
            time.monotonic()
        )

    def on_adas_command(
        self,
        msg: String,
    ) -> None:

        try:

            self.adas_command = (
                json.loads(
                    msg.data
                )
            )

        except json.JSONDecodeError:

            self.adas_command = {}

        self.adas_command_time = (
            time.monotonic()
        )

    # ========================================================
    # PID
    # ========================================================

    def reset_pid(
        self,
    ) -> None:

        self.pid_integral = 0.0

        self.pid_previous_error = 0.0

        self.pid_previous_time = (
            time.monotonic()
        )

    def pid_control(
        self,
        target_speed: float,
        current_speed: float,
    ):

        now = (
            time.monotonic()
        )

        dt = clamp(
            now
            - self.pid_previous_time,
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

        self.pid_integral = clamp(
            self.pid_integral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT,
        )

        derivative = (
            (
                error
                - self.pid_previous_error
            )
            / dt
        )

        output = (
            PID_KP
            * error
            + PID_KI
            * self.pid_integral
            + PID_KD
            * derivative
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

        elif output >= 0.0:

            throttle = clamp(
                output,
                0.0,
                MAX_THROTTLE,
            )

            brake = 0.0

        else:

            throttle = 0.0

            brake = clamp(
                -output,
                0.0,
                MAX_SERVICE_BRAKE,
            )

        return (
            throttle,
            brake,
            error,
            output,
        )

    # ========================================================
    # ACC / AEB
    # ========================================================

    def resolve_adas(
        self,
        current_speed: float,
    ):
        """
        반환값
        ------
        adas_mode
        adas_target_speed
        hard_brake
        desired_gap
        """

        if self.ignore_adas:

            return (
                "CRUISE",
                CRUISE_SPEED_MPS,
                0.0,
                None,
            )

        now = (
            time.monotonic()
        )

        # Tracking 프로그램이 동작하지 않는 경우
        if (
            now
            - self.tracking_status_time
            > TRACKING_STATUS_TIMEOUT_SEC
        ):

            return (
                "PERCEPTION_FALLBACK",
                0.0,
                FAILSAFE_BRAKE,
                None,
            )

        mio_track_id = (
            self.tracking_status.get(
                "mio_track_id"
            )
        )

        # Tracking은 정상이나 선행 객체가 없음
        if mio_track_id is None:

            return (
                "CRUISE",
                CRUISE_SPEED_MPS,
                0.0,
                None,
            )

        if (
            self.mio is None
            or (
                now
                - self.mio_time
                > MIO_STALE_TIMEOUT_SEC
            )
        ):

            return (
                "MIO_FALLBACK",
                0.0,
                FAILSAFE_BRAKE,
                None,
            )

        if (
            now
            - self.adas_command_time
            > ADAS_COMMAND_TIMEOUT_SEC
        ):

            return (
                "ADAS_FALLBACK",
                0.0,
                FAILSAFE_BRAKE,
                None,
            )

        adas_mode = str(
            self.adas_command.get(
                "mode",
                "CRUISE",
            )
        )

        range_m = float(
            self.mio[0]
        )

        relative_speed = float(
            self.mio[2]
        )

        # ----------------------------------------------------
        # TODO 1
        #
        # ACC 목표 차간거리
        #
        # desired_gap =
        #     minimum_gap
        #     + time_headway * ego_speed
        # ----------------------------------------------------

        desired_gap = None

        check_completed(
            "TODO 1: desired_gap",
            desired_gap,
        )

        # ----------------------------------------------------
        # TODO 2
        #
        # MIO 기반 ACC 목표속도
        #
        # gap_error =
        #     actual_range - desired_gap
        #
        # acc_target =
        #     current_speed
        #     + K_gap * gap_error
        #     + K_rel * relative_speed
        #
        # relative_speed < 0 → Closing
        # ----------------------------------------------------

        gap_error = (
            range_m
            - desired_gap
        )

        acc_target = None

        check_completed(
            "TODO 2: acc_target",
            acc_target,
        )

        acc_target = clamp(
            acc_target,
            0.0,
            CRUISE_SPEED_MPS,
        )

        # ----------------------------------------------------
        # TODO 3
        #
        # AEB는 일반 ACC/PID보다 우선합니다.
        #
        # adas_mode가 "AEB"이면:
        #
        # mode        = "AEB"
        # target      = 0.0
        # hard_brake  = AEB_BRAKE
        # ----------------------------------------------------

        if adas_mode == "AEB":

            result = None

            check_completed(
                "TODO 3: AEB result",
                result,
            )

            return result

        if adas_mode == "FALLBACK_HOLD":

            return (
                "ADAS_FALLBACK",
                0.0,
                FAILSAFE_BRAKE,
                desired_gap,
            )

        if adas_mode == "ACC_BRAKE":

            acc_target = min(
                acc_target,
                ACC_BRAKE_MAX_SPEED_MPS,
            )

            return (
                "ACC_BRAKE",
                acc_target,
                0.0,
                desired_gap,
            )

        if adas_mode == "ACC_HOLD":

            return (
                "ACC_HOLD",
                acc_target,
                0.0,
                desired_gap,
            )

        return (
            "CRUISE",
            CRUISE_SPEED_MPS,
            0.0,
            desired_gap,
        )

    # ========================================================
    # Main Control
    # ========================================================

    def control_loop(
        self,
    ) -> None:

        now = (
            time.monotonic()
        )

        # ----------------------------------------------------
        # 입력 상태 확인
        # ----------------------------------------------------

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
        # Global Path
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

        target_x, target_y = (
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
        # Camera Lane Correction
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

                lane_correction = (
                    LANE_OFFSET_GAIN
                    * lane_offset
                    + LANE_HEADING_GAIN
                    * LOOKAHEAD_DISTANCE_M
                    * math.tan(
                        lane_heading
                    )
                )

                lane_correction = clamp(
                    lane_correction,
                    -MAX_LANE_CORRECTION_M,
                    MAX_LANE_CORRECTION_M,
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

        steer_cmd = clamp(
            -steering_rad
            / MAX_STEERING_RAD,
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # Goal 속도 제한
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

            goal_speed_limit = 0.0

            self.goal_reached = True

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

            goal_speed_limit = (
                CRUISE_SPEED_MPS
                * ratio
            )

        else:

            goal_speed_limit = (
                CRUISE_SPEED_MPS
            )

        # ----------------------------------------------------
        # ADAS 종방향 판단
        # ----------------------------------------------------

        current_speed = (
            speed_mps(
                self.ego
            )
        )

        (
            adas_mode,
            adas_speed_limit,
            hard_brake,
            desired_gap,
        ) = self.resolve_adas(
            current_speed
        )

        # ----------------------------------------------------
        # TODO 4
        #
        # Goal 접근 속도 제한과
        # ACC 속도 제한 중
        # 더 보수적인 값을 사용합니다.
        # ----------------------------------------------------

        target_speed = None

        check_completed(
            "TODO 4: target_speed",
            target_speed,
        )

        # ----------------------------------------------------
        # TODO 5
        #
        # hard_brake가 존재하면
        # 일반 PID를 우회합니다.
        #
        # hard_brake > 0.05:
        #     throttle = 0
        #     brake = hard_brake
        #     PID reset
        #
        # 그 외:
        #     pid_control(...)
        # ----------------------------------------------------

        if hard_brake > 0.05:

            throttle = None
            brake = None

            check_completed(
                "TODO 5: throttle",
                throttle,
            )

            check_completed(
                "TODO 5: brake",
                brake,
            )

            speed_error = (
                target_speed
                - current_speed
            )

            pid_output = 0.0

            self.reset_pid()

        else:

            (
                throttle,
                brake,
                speed_error,
                pid_output,
            ) = self.pid_control(
                target_speed,
                current_speed,
            )

        # ----------------------------------------------------
        # Goal Hold
        # ----------------------------------------------------

        if self.goal_reached:

            target_speed = 0.0

            throttle = 0.0

            brake = max(
                brake,
                0.60,
            )

            mode = (
                "GOAL_REACHED"
            )

        elif (
            goal_distance
            < GOAL_SLOWDOWN_DISTANCE_M
        ):

            mode = (
                f"GOAL_APPROACH+{adas_mode}"
            )

        else:

            mode = (
                f"PATH_TRACK+{adas_mode}"
            )

        # ----------------------------------------------------
        # CARLA Control
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Visualization / Status
        # ----------------------------------------------------

        self.publish_lookahead(
            target_x,
            target_y,
        )

        mio_range = None

        mio_relative_speed = None

        if (
            self.mio is not None
            and (
                now
                - self.mio_time
                <= MIO_STALE_TIMEOUT_SEC
            )
        ):

            mio_range = float(
                self.mio[0]
            )

            mio_relative_speed = float(
                self.mio[2]
            )

        status = {
            "mode": mode,

            "actuator_connected": (
                self.apply_control
            ),

            "goal_distance_m": float(
                goal_distance
            ),

            "path_progress_percent": float(
                100.0
                * nearest
                / max(
                    len(
                        self.path_xy
                    )
                    - 1,
                    1,
                )
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

            "mio_range_m": (
                mio_range
            ),

            "mio_relative_speed_mps": (
                mio_relative_speed
            ),

            "adas_mode": (
                adas_mode
            ),

            "desired_gap_m": (
                desired_gap
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

        self.reset_pid()

        if self.apply_control:

            self.ego.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    brake=FAILSAFE_BRAKE,
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
                        "brake": FAILSAFE_BRAKE,
                    }
                )
            )
        )

    # ========================================================
    # 종료
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
        "--ignore-adas",
        action="store_true",
    )

    parser.add_argument(
        "--no-lane-correction",
        action="store_true",
    )

    args = parser.parse_args()

    rclpy.init()

    node = IntegratedAdasDriving(
        apply_control=(
            args.apply_control
        ),
        ignore_adas=(
            args.ignore_adas
        ),
        lane_correction=(
            not args.no_lane_correction
        ),
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
# 확인 항목
#
# [ ] Global Path 수신
# [ ] GNSS 기반 현재 위치 계산
# [ ] Pure Pursuit 경로 추종
# [ ] Camera Lane Correction
# [ ] Camera-Radar MIO 수신
# [ ] MIO Range-rate 기반 ACC
# [ ] ACC 목표속도 → PID
# [ ] AEB 발생 시 PID 우회
# [ ] Goal 접근 시 감속
# [ ] Goal에서 정지
#
#
# 단계별 실행
# -----------
#
# 1. ADAS 없이 경로 추종
#
# python3 20_camera_radar_gnss_adas_driving.py \
#   --apply-control \
#   --ignore-adas
#
#
# 2. Camera-Radar-GNSS 통합
#
# python3 20_camera_radar_gnss_adas_driving.py \
#   --apply-control
#
#
# 비교 실험
# ---------
#
# ACC_TIME_HEADWAY_SEC
# 1.0 → 1.5 → 2.0
#
# ACC_REL_SPEED_GAIN
# 0.4 → 0.8 → 1.2
# ============================================================
