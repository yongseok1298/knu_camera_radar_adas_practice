#!/usr/bin/env python3
"""
Lab 07: CARLA Camera / Radar ROS2 Bridge

Course contract
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

Important
---------
CARLA Radar detections are originally expressed in the Radar sensor frame.
This node transforms them into the vehicle frame before publishing.
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


# ============================================================
# Course Sensor Contract
# ============================================================

CAMERA_X_M = 1.5
CAMERA_Y_M = 0.0
CAMERA_Z_M = 2.0

RADAR_X_M = 2.0
RADAR_Y_M = 0.0
RADAR_Z_M = 1.0

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360
CAMERA_FOV_DEG = 90.0

RADAR_HORIZONTAL_FOV_DEG = 30.0
RADAR_VERTICAL_FOV_DEG = 10.0
RADAR_RANGE_M = 80.0

SENSOR_TICK_SEC = 0.05


def time_message(
    seconds: float,
) -> Time:

    sec = int(
        seconds
    )

    return Time(
        sec=sec,
        nanosec=int(
            (
                seconds
                - sec
            )
            * 1_000_000_000
        ),
    )


class CarlaCameraRadarBridge(Node):

    def __init__(
        self,
        host: str,
        port: int,
        spawn_index: int | None,
    ) -> None:

        super().__init__(
            "carla_camera_radar_bridge"
        )

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

        self.spawn_index = spawn_index

        self.actors: list[
            carla.Actor
        ] = []

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

        self.ego = (
            self._find_or_spawn_ego()
        )

        self._spawn_sensors()

        self.create_timer(
            0.05,
            self.publish_clock,
        )

    # ========================================================
    # Ego
    # ========================================================

    def _find_or_spawn_ego(
        self,
    ):

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
                        "using existing "
                        f"knu_hero id={actor.id}"
                    )
                )

                return actor

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

        if self.spawn_index is not None:
            if not 0 <= self.spawn_index < len(spawn_points):
                raise ValueError(
                    f"spawn_index must be 0~{len(spawn_points) - 1}"
                )

            actor = self.world.try_spawn_actor(
                blueprint,
                spawn_points[self.spawn_index],
            )

            if actor is None:
                raise RuntimeError(
                    f"spawn point {self.spawn_index} is occupied"
                )

            self.actors.append(actor)
            self.get_logger().info(
                f"spawned knu_hero at spawn_index={self.spawn_index} id={actor.id}"
            )
            return actor

        for transform in spawn_points:
            actor = self.world.try_spawn_actor(
                blueprint,
                transform,
            )

            if actor is not None:
                self.actors.append(actor)
                self.get_logger().info(
                    f"spawned stationary knu_hero id={actor.id}"
                )
                return actor

        raise RuntimeError(
            (
                "could not spawn knu_hero; "
                "reload the CARLA map"
            )
        )

    # ========================================================
    # Sensors
    # ========================================================

    def _spawn_sensors(
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

        for key, value in (
            (
                "image_size_x",
                str(CAMERA_WIDTH),
            ),
            (
                "image_size_y",
                str(CAMERA_HEIGHT),
            ),
            (
                "fov",
                str(CAMERA_FOV_DEG),
            ),
            (
                "sensor_tick",
                str(SENSOR_TICK_SEC),
            ),
        ):

            camera_bp.set_attribute(
                key,
                value,
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
        # Radar
        # ----------------------------------------------------

        radar_bp = (
            library.find(
                "sensor.other.radar"
            )
        )

        for key, value in (
            (
                "horizontal_fov",
                str(
                    RADAR_HORIZONTAL_FOV_DEG
                ),
            ),
            (
                "vertical_fov",
                str(
                    RADAR_VERTICAL_FOV_DEG
                ),
            ),
            (
                "range",
                str(RADAR_RANGE_M),
            ),
            (
                "sensor_tick",
                str(SENSOR_TICK_SEC),
            ),
        ):

            radar_bp.set_attribute(
                key,
                value,
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

        self.get_logger().info(
            (
                "publishing 20 Hz RGB camera "
                "and vehicle-frame radar"
            )
        )

        self.get_logger().info(
            (
                "camera xyz(vehicle) = "
                f"({CAMERA_X_M:.1f}, "
                f"{CAMERA_Y_M:.1f}, "
                f"{CAMERA_Z_M:.1f}) m"
            )
        )

        self.get_logger().info(
            (
                "radar xyz(vehicle) = "
                f"({RADAR_X_M:.1f}, "
                f"{RADAR_Y_M:.1f}, "
                f"{RADAR_Z_M:.1f}) m"
            )
        )

    # ========================================================
    # Clock
    # ========================================================

    def publish_clock(
        self,
    ) -> None:

        elapsed = (
            self.world
            .get_snapshot()
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
    # Camera
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
    # Radar
    # ========================================================

    def on_radar(
        self,
        measurement: carla.RadarMeasurement,
    ) -> None:

        points = []

        for detection in measurement:

            # ------------------------------------------------
            # 1. CARLA Radar polar measurement
            #    → Radar sensor-local Cartesian coordinates
            #
            # Course convention:
            # x = forward
            # y = left
            # z = up
            # ------------------------------------------------

            horizontal = (
                detection.depth
                * math.cos(
                    detection.altitude
                )
            )

            x_radar = (
                horizontal
                * math.cos(
                    detection.azimuth
                )
            )

            # CARLA azimuth positive direction
            # → course y-left convention
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
            # 2. Radar sensor frame → Vehicle frame
            #
            # Radar mounting orientation = zero rotation,
            # therefore only translation is needed here.
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
    # Shutdown
    # ========================================================

    def destroy_node(
        self,
    ) -> bool:

        for actor in reversed(
            self.actors
        ):

            if actor.is_alive:

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

    parser.add_argument(
        "--spawn-index",
        type=int,
        default=None,
    )

    args = parser.parse_args()

    rclpy.init()

    node = (
        CarlaCameraRadarBridge(
            args.host,
            args.port,
            args.spawn_index,
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
