#!/usr/bin/env python3
"""
Practice 07: CARLA Camera / Radar ROS2 Bridge

목표
----
- CARLA에 knu_hero 차량을 생성합니다.
- Front Camera와 Front Radar를 장착합니다.
- Camera Image와 Radar PointCloud2를 ROS2 Topic으로 Publish합니다.
- Radar sensor-local 좌표를 vehicle frame으로 변환합니다.

Course Contract
---------------
Ego role_name : knu_hero

Vehicle frame
    x = forward
    y = left
    z = up

Camera
    /carla/hero/camera_front/image

Radar
    /carla/hero/radar/point_cloud
"""

from __future__ import annotations

import argparse
import math

import carla
import rclpy

from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import (
    Image,
    PointCloud2,
    PointField,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


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
# Sensor Mounting Position
#
# Vehicle frame:
# x = forward
# y = left
# z = up
#
# Camera와 Radar의 장착 위치는 Course Contract로 고정합니다.
# ============================================================

CAMERA_X_M = 1.5
CAMERA_Y_M = 0.0
CAMERA_Z_M = 2.0

RADAR_X_M = 2.0
RADAR_Y_M = 0.0
RADAR_Z_M = 1.0


# ============================================================
# Practice 1. Camera Parameters
#
# Reference
# ----------
# Resolution : 640 x 360
# FOV        : 90 deg
# ============================================================

## TODO 1
CAMERA_WIDTH = None
CAMERA_HEIGHT = None
CAMERA_FOV_DEG = None


# ============================================================
# Practice 2. Radar Parameters
#
# Reference
# ----------
# Horizontal FOV : 30 deg
# Vertical FOV   : 10 deg
# Range          : 80 m
# ============================================================

## TODO 2
RADAR_HORIZONTAL_FOV_DEG = None
RADAR_VERTICAL_FOV_DEG = None
RADAR_RANGE_M = None


# ============================================================
# Practice 3. Sensor Rate
#
# sensor_tick = 0.05 sec
#
# 1 / 0.05 = 20 Hz
# ============================================================

## TODO 3
SENSOR_TICK_SEC = None


def time_message(
    seconds: float,
) -> Time:
    """
    CARLA timestamp [sec]
    → ROS2 builtin_interfaces/Time
    """

    sec = int(
        seconds
    )

    nanosec = int(
        (
            seconds
            - sec
        )
        * 1_000_000_000
    )

    return Time(
        sec=sec,
        nanosec=nanosec,
    )


class CarlaCameraRadarBridge(Node):

    def __init__(
        self,
        host: str,
        port: int,
    ) -> None:

        check_completed(
            "TODO 1: CAMERA_WIDTH",
            CAMERA_WIDTH,
        )

        check_completed(
            "TODO 1: CAMERA_HEIGHT",
            CAMERA_HEIGHT,
        )

        check_completed(
            "TODO 1: CAMERA_FOV_DEG",
            CAMERA_FOV_DEG,
        )

        check_completed(
            "TODO 2: RADAR_HORIZONTAL_FOV_DEG",
            RADAR_HORIZONTAL_FOV_DEG,
        )

        check_completed(
            "TODO 2: RADAR_VERTICAL_FOV_DEG",
            RADAR_VERTICAL_FOV_DEG,
        )

        check_completed(
            "TODO 2: RADAR_RANGE_M",
            RADAR_RANGE_M,
        )

        check_completed(
            "TODO 3: SENSOR_TICK_SEC",
            SENSOR_TICK_SEC,
        )

        super().__init__(
            "carla_camera_radar_bridge"
        )

        # ----------------------------------------------------
        # CARLA Connection
        # ----------------------------------------------------

        self.client = carla.Client(
            host,
            port,
        )

        self.client.set_timeout(
            10.0
        )

        self.world = (
            self.client.get_world()
        )

        self.actors: list[
            carla.Actor
        ] = []

        # ----------------------------------------------------
        # ROS2 Publishers
        # ----------------------------------------------------

        self.clock_pub = (
            self.create_publisher(
                Clock,
                "/clock",
                10,
            )
        )

        self.camera_pub = (
            self.create_publisher(
                Image,
                "/carla/hero/camera_front/image",
                10,
            )
        )

        self.radar_pub = (
            self.create_publisher(
                PointCloud2,
                "/carla/hero/radar/point_cloud",
                10,
            )
        )

        # ----------------------------------------------------
        # Ego + Sensors
        # ----------------------------------------------------

        self.ego = (
            self.find_or_spawn_ego()
        )

        self.spawn_sensors()

        self.create_timer(
            0.05,
            self.publish_clock,
        )

    # ========================================================
    # Ego Vehicle
    # ========================================================

    def find_or_spawn_ego(
        self,
    ):

        # 기존 knu_hero가 있으면 재사용
        for actor in (
            self.world
            .get_actors()
            .filter("vehicle.*")
        ):

            if (
                actor.attributes.get(
                    "role_name"
                )
                == "knu_hero"
            ):

                self.get_logger().info(
                    (
                        "Using existing "
                        f"knu_hero id={actor.id}"
                    )
                )

                return actor

        # 없으면 Tesla Model 3 생성
        blueprint = (
            self.world
            .get_blueprint_library()
            .find(
                "vehicle.tesla.model3"
            )
        )

        blueprint.set_attribute(
            "role_name",
            "knu_hero",
        )

        spawn_points = (
            self.world
            .get_map()
            .get_spawn_points()
        )

        for transform in spawn_points:

            actor = (
                self.world.try_spawn_actor(
                    blueprint,
                    transform,
                )
            )

            if actor is not None:

                self.actors.append(
                    actor
                )

                self.get_logger().info(
                    (
                        "Spawned "
                        f"knu_hero id={actor.id}"
                    )
                )

                return actor

        raise RuntimeError(
            "Could not spawn knu_hero."
        )

    # ========================================================
    # Camera / Radar
    # ========================================================

    def spawn_sensors(
        self,
    ) -> None:

        library = (
            self.world
            .get_blueprint_library()
        )

        # ----------------------------------------------------
        # RGB Camera
        # ----------------------------------------------------

        camera_bp = (
            library.find(
                "sensor.camera.rgb"
            )
        )

        camera_bp.set_attribute(
            "image_size_x",
            str(
                CAMERA_WIDTH
            ),
        )

        camera_bp.set_attribute(
            "image_size_y",
            str(
                CAMERA_HEIGHT
            ),
        )

        camera_bp.set_attribute(
            "fov",
            str(
                CAMERA_FOV_DEG
            ),
        )

        camera_bp.set_attribute(
            "sensor_tick",
            str(
                SENSOR_TICK_SEC
            ),
        )

        camera_transform = (
            carla.Transform(
                carla.Location(
                    x=CAMERA_X_M,
                    y=-CAMERA_Y_M,
                    z=CAMERA_Z_M,
                )
            )
        )

        camera = (
            self.world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=self.ego,
            )
        )

        self.actors.append(
            camera
        )

        camera.listen(
            self.on_camera
        )

        # ----------------------------------------------------
        # Front Radar
        # ----------------------------------------------------

        radar_bp = (
            library.find(
                "sensor.other.radar"
            )
        )

        radar_bp.set_attribute(
            "horizontal_fov",
            str(
                RADAR_HORIZONTAL_FOV_DEG
            ),
        )

        radar_bp.set_attribute(
            "vertical_fov",
            str(
                RADAR_VERTICAL_FOV_DEG
            ),
        )

        radar_bp.set_attribute(
            "range",
            str(
                RADAR_RANGE_M
            ),
        )

        radar_bp.set_attribute(
            "sensor_tick",
            str(
                SENSOR_TICK_SEC
            ),
        )

        radar_transform = (
            carla.Transform(
                carla.Location(
                    x=RADAR_X_M,
                    y=-RADAR_Y_M,
                    z=RADAR_Z_M,
                )
            )
        )

        radar = (
            self.world.spawn_actor(
                radar_bp,
                radar_transform,
                attach_to=self.ego,
            )
        )

        self.actors.append(
            radar
        )

        radar.listen(
            self.on_radar
        )

        # ----------------------------------------------------
        # Information
        # ----------------------------------------------------

        sensor_hz = (
            1.0
            / SENSOR_TICK_SEC
        )

        self.get_logger().info(
            (
                f"Camera/Radar publishing "
                f"at {sensor_hz:.1f} Hz"
            )
        )

        self.get_logger().info(
            (
                "Camera position(vehicle): "
                f"x={CAMERA_X_M:.1f}, "
                f"y={CAMERA_Y_M:.1f}, "
                f"z={CAMERA_Z_M:.1f} m"
            )
        )

        self.get_logger().info(
            (
                "Radar position(vehicle): "
                f"x={RADAR_X_M:.1f}, "
                f"y={RADAR_Y_M:.1f}, "
                f"z={RADAR_Z_M:.1f} m"
            )
        )

    # ========================================================
    # Simulation Clock
    # ========================================================

    def publish_clock(
        self,
    ) -> None:

        snapshot = (
            self.world
            .get_snapshot()
        )

        elapsed = (
            snapshot
            .timestamp
            .elapsed_seconds
        )

        self.clock_pub.publish(
            Clock(
                clock=time_message(
                    elapsed
                )
            )
        )

    # ========================================================
    # Camera Callback
    # ========================================================

    def on_camera(
        self,
        image: carla.Image,
    ) -> None:

        msg = Image()

        msg.header = Header(
            stamp=time_message(
                image.timestamp
            ),
            frame_id=(
                "camera_front_optical"
            ),
        )

        msg.height = (
            image.height
        )

        msg.width = (
            image.width
        )

        # CARLA raw image = BGRA
        msg.encoding = "bgra8"

        msg.is_bigendian = False

        msg.step = (
            image.width
            * 4
        )

        msg.data = bytes(
            image.raw_data
        )

        self.camera_pub.publish(
            msg
        )

    # ========================================================
    # Radar Callback
    # ========================================================

    def on_radar(
        self,
        measurement: carla.RadarMeasurement,
    ) -> None:

        points = []

        for detection in measurement:

            # ------------------------------------------------
            # CARLA Radar Measurement
            #
            # depth     : range [m]
            # azimuth   : horizontal angle [rad]
            # altitude  : vertical angle [rad]
            # velocity  : relative radial speed [m/s]
            # ------------------------------------------------

            horizontal = (
                detection.depth
                * math.cos(
                    detection.altitude
                )
            )

            # ------------------------------------------------
            # Radar Sensor-Local Cartesian
            #
            # Course convention:
            # x = forward
            # y = left
            # z = up
            # ------------------------------------------------

            x_radar = (
                horizontal
                * math.cos(
                    detection.azimuth
                )
            )

            y_radar = -(
                horizontal
                * math.sin(
                    detection.azimuth
                )
            )

            z_radar = (
                detection.depth
                * math.sin(
                    detection.altitude
                )
            )

            # ------------------------------------------------
            # Radar Sensor Frame → Vehicle Frame
            #
            # Radar mounting rotation = 0
            # 따라서 현재 실습에서는 Translation만 적용합니다.
            #
            # 단순히 frame_id="vehicle"이라고 적는 것만으로
            # 좌표가 자동 변환되는 것은 아닙니다.
            # ------------------------------------------------

            x_vehicle = (
                RADAR_X_M
                + x_radar
            )

            y_vehicle = (
                RADAR_Y_M
                + y_radar
            )

            z_vehicle = (
                RADAR_Z_M
                + z_radar
            )

            points.append(
                (
                    float(
                        x_vehicle
                    ),
                    float(
                        y_vehicle
                    ),
                    float(
                        z_vehicle
                    ),
                    float(
                        -detection.velocity
                    ),
                )
            )

        # ----------------------------------------------------
        # PointCloud2 Fields
        # ----------------------------------------------------

        fields = [
            PointField(
                name="x",
                offset=0,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            ),
            PointField(
                name=(
                    "relative_speed_mps"
                ),
                offset=12,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            ),
        ]

        header = Header(
            stamp=time_message(
                measurement.timestamp
            ),
            frame_id="vehicle",
        )

        cloud = (
            point_cloud2.create_cloud(
                header,
                fields,
                points,
            )
        )

        self.radar_pub.publish(
            cloud
        )

    # ========================================================
    # Safe Shutdown
    # ========================================================

    def destroy_node(
        self,
    ) -> bool:

        for actor in reversed(
            self.actors
        ):

            if not actor.is_alive:
                continue

            # Sensor는 destroy 전에 stop
            if isinstance(
                actor,
                carla.Sensor,
            ):
                actor.stop()

            actor.destroy()

        return super().destroy_node()


def main() -> None:

    parser = (
        argparse.ArgumentParser()
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

    node = (
        CarlaCameraRadarBridge(
            host=args.host,
            port=args.port,
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
# 실행
#
# source /opt/ros/humble/setup.bash
#
# python3 07_carla_camera_radar_bridge.py
#
#
# Camera 확인
#
# ros2 run rqt_image_view rqt_image_view
#
# /carla/hero/camera_front/image
#
#
# Radar 확인
#
# rviz2
#
# Fixed Frame:
# vehicle
#
# PointCloud2:
# /carla/hero/radar/point_cloud
#
#
# Topic Rate 확인
#
# ros2 topic hz /carla/hero/camera_front/image
#
# ros2 topic hz /carla/hero/radar/point_cloud
#
#
# Mini Experiment 1
#
# CAMERA_FOV_DEG
# 90 → 70 → 110
#
# Camera 화면의 시야각 변화를 비교합니다.
#
#
# Mini Experiment 2
#
# RADAR_RANGE_M
# 80 → 40
#
# Radar detection 범위의 변화를 비교합니다.
#
#
# Mini Experiment 3
#
# SENSOR_TICK_SEC
# 0.05 → 0.10
#
# 20 Hz → 10 Hz로 Topic rate가 변하는지 확인합니다.
# ============================================================
