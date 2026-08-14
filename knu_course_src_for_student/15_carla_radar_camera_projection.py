#!/usr/bin/env python3
"""
Practice 15: CARLA Live Radar-to-Camera Projection

목표
----
- Radar Point를 Vehicle Frame에서 Camera Frame으로 변환합니다.
- Perspective Projection으로 Radar Point의 pixel 좌표를 계산합니다.
- Camera 영상 위에 실제 Radar return을 표시합니다.

Coordinate Contract
-------------------
Vehicle frame
    x = forward
    y = left
    z = up

Camera optical frame
    X = right
    Y = down
    Z = forward
"""

from __future__ import annotations

import json
import math

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import (
    Image,
    PointCloud2,
    PointField,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def check_completed(
    name: str,
    value,
) -> None:

    if value is None:

        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. "
            "해당 ## TODO를 확인하세요."
        )


def stamp_seconds(msg) -> float:

    stamp = msg.header.stamp

    return (
        float(stamp.sec)
        + float(stamp.nanosec)
        * 1e-9
    )


CAMERA_TOPIC = (
    "/carla/hero/camera_front/image"
)

RADAR_TOPIC = (
    "/carla/hero/radar/point_cloud"
)

PROJECTED_TOPIC = (
    "/carla/radar/projected_points"
)

DEBUG_TOPIC = (
    "/carla/radar_projection/debug_image"
)

STATUS_TOPIC = (
    "/carla/radar_projection/status"
)


# ============================================================
# Practice 1. Camera Intrinsic
#
# Lab 07 Front Camera FOV
# 기본값 = 90 deg
# ============================================================

## TODO 1
CAMERA_FOV_DEG = None


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360


# ============================================================
# Practice 2. Camera Extrinsic
#
# Camera position in vehicle frame
#
# x = forward
# y = left
# z = up
#
# Lab 07:
# x = 1.5 m
# y = 0.0 m
# z = 2.0 m
# ============================================================

## TODO 2
CAMERA_X_M = None
CAMERA_Z_M = None

CAMERA_Y_M = 0.0


MAX_RANGE_M = 80.0
MAX_LATERAL_M = 20.0

MIN_CAMERA_DEPTH_M = 0.10
MAX_SYNC_DELTA_SEC = 0.10

POINT_RADIUS_PX = 4


def focal_length(
    width: int,
    fov_deg: float,
) -> float:
    """
    Horizontal FOV → focal length [pixel]

                  width
    fx = -----------------------
         2 * tan(FOV / 2)
    """

    return (
        float(width)
        / (
            2.0
            * math.tan(
                math.radians(
                    fov_deg
                )
                / 2.0
            )
        )
    )


def image_to_bgr(
    msg: Image,
) -> np.ndarray | None:

    encoding = (
        msg.encoding.lower()
    )

    channels = (
        4
        if encoding
        in {
            "bgra8",
            "rgba8",
        }
        else 3
    )

    raw = np.frombuffer(
        msg.data,
        dtype=np.uint8,
    )

    expected = (
        msg.height
        * msg.width
        * channels
    )

    if (
        msg.height <= 0
        or msg.width <= 0
        or raw.size < expected
    ):
        return None

    image = (
        raw[:expected]
        .reshape(
            msg.height,
            msg.width,
            channels,
        )
    )

    if encoding == "bgra8":

        return cv2.cvtColor(
            image,
            cv2.COLOR_BGRA2BGR,
        )

    if encoding == "rgba8":

        return cv2.cvtColor(
            image,
            cv2.COLOR_RGBA2BGR,
        )

    return image[
        :, :, :3
    ].copy()


def read_radar_points(
    msg: PointCloud2,
) -> np.ndarray:

    rows = []

    for point in point_cloud2.read_points(
        msg,
        field_names=(
            "x",
            "y",
            "z",
            "relative_speed_mps",
        ),
        skip_nans=True,
    ):

        rows.append(
            [
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(point[3]),
            ]
        )

    if not rows:

        return np.empty(
            (0, 4),
            dtype=np.float32,
        )

    return np.asarray(
        rows,
        dtype=np.float32,
    )


def project_vehicle_points(
    radar_points: np.ndarray,
    image_width: int,
    image_height: int,
) -> np.ndarray:

    check_completed(
        "TODO 1: CAMERA_FOV_DEG",
        CAMERA_FOV_DEG,
    )

    check_completed(
        "TODO 2: CAMERA_X_M",
        CAMERA_X_M,
    )

    check_completed(
        "TODO 2: CAMERA_Z_M",
        CAMERA_Z_M,
    )

    if radar_points.size == 0:

        return np.empty(
            (0, 6),
            dtype=np.float32,
        )

    x_forward = radar_points[:, 0]
    y_left = radar_points[:, 1]
    z_up = radar_points[:, 2]

    relative_speed = (
        radar_points[:, 3]
    )

    range_m = np.hypot(
        x_forward,
        y_left,
    )

    valid = (
        (x_forward > 0.0)
        & (range_m <= MAX_RANGE_M)
        & (
            np.abs(y_left)
            <= MAX_LATERAL_M
        )
    )

    x_forward = x_forward[valid]
    y_left = y_left[valid]
    z_up = z_up[valid]

    relative_speed = (
        relative_speed[valid]
    )

    # --------------------------------------------------------
    # Camera-relative vehicle coordinates
    # --------------------------------------------------------

    dx = (
        x_forward
        - CAMERA_X_M
    )

    dy = (
        y_left
        - CAMERA_Y_M
    )

    dz = (
        z_up
        - CAMERA_Z_M
    )

    # ========================================================
    # Practice 3.
    # Vehicle Frame → Camera Optical Frame
    #
    # Vehicle:
    # x forward, y left, z up
    #
    # Camera:
    # X right, Y down, Z forward
    # ========================================================

    ## TODO 3
    x_cam = None
    y_cam = None
    z_cam = None

    check_completed(
        "TODO 3: x_cam",
        x_cam,
    )

    check_completed(
        "TODO 3: y_cam",
        y_cam,
    )

    check_completed(
        "TODO 3: z_cam",
        z_cam,
    )

    in_front = (
        z_cam
        > MIN_CAMERA_DEPTH_M
    )

    x_cam = x_cam[in_front]
    y_cam = y_cam[in_front]
    z_cam = z_cam[in_front]

    x_forward = x_forward[in_front]
    y_left = y_left[in_front]
    z_up = z_up[in_front]

    relative_speed = (
        relative_speed[in_front]
    )

    if len(z_cam) == 0:

        return np.empty(
            (0, 6),
            dtype=np.float32,
        )

    fx = focal_length(
        image_width,
        CAMERA_FOV_DEG,
    )

    fy = fx

    cx = (
        image_width
        / 2.0
    )

    cy = (
        image_height
        / 2.0
    )

    # ========================================================
    # Practice 4. Perspective Projection
    #
    # u = fx * X/Z + cx
    # v = fy * Y/Z + cy
    # ========================================================

    ## TODO 4
    u = None
    v = None

    check_completed(
        "TODO 4: u",
        u,
    )

    check_completed(
        "TODO 4: v",
        v,
    )

    inside = (
        (u >= 0.0)
        & (u < image_width)
        & (v >= 0.0)
        & (v < image_height)
    )

    return np.column_stack(
        (
            u[inside],
            v[inside],
            x_forward[inside],
            y_left[inside],
            z_up[inside],
            relative_speed[inside],
        )
    ).astype(
        np.float32
    )


class RadarCameraProjection(Node):

    def __init__(self) -> None:

        super().__init__(
            "carla_radar_camera_projection"
        )

        self.latest_camera = None

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.on_camera,
            10,
        )

        self.create_subscription(
            PointCloud2,
            RADAR_TOPIC,
            self.on_radar,
            10,
        )

        self.projected_pub = (
            self.create_publisher(
                PointCloud2,
                PROJECTED_TOPIC,
                10,
            )
        )

        self.debug_pub = (
            self.create_publisher(
                Image,
                DEBUG_TOPIC,
                10,
            )
        )

        self.status_pub = (
            self.create_publisher(
                String,
                STATUS_TOPIC,
                10,
            )
        )

    def on_camera(
        self,
        msg: Image,
    ) -> None:

        bgr = image_to_bgr(
            msg
        )

        if bgr is None:
            return

        self.latest_camera = (
            stamp_seconds(msg),
            bgr,
            msg.header,
        )

    def on_radar(
        self,
        msg: PointCloud2,
    ) -> None:

        radar_points = (
            read_radar_points(
                msg
            )
        )

        radar_stamp = (
            stamp_seconds(msg)
        )

        if self.latest_camera is None:
            return

        (
            camera_stamp,
            camera,
            camera_header,
        ) = self.latest_camera

        sync_delta = (
            radar_stamp
            - camera_stamp
        )

        projected = (
            project_vehicle_points(
                radar_points,
                camera.shape[1],
                camera.shape[0],
            )
        )

        self.publish_cloud(
            msg,
            projected,
        )

        synchronized = (
            abs(sync_delta)
            <= MAX_SYNC_DELTA_SEC
        )

        status = {
            "raw_radar_points": (
                len(radar_points)
            ),
            "projected_points": (
                len(projected)
            ),
            "sync_delta_ms": (
                sync_delta
                * 1000.0
            ),
            "synchronized": (
                synchronized
            ),
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status
                )
            )
        )

        if not synchronized:
            return

        debug = camera.copy()

        for point in projected:

            cv2.circle(
                debug,
                (
                    int(point[0]),
                    int(point[1]),
                ),
                POINT_RADIUS_PX,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )

        if len(projected) > 0:

            nearest = projected[
                np.argmin(
                    projected[:, 2]
                )
            ]

            cv2.putText(
                debug,
                (
                    f"nearest={nearest[2]:.1f} m  "
                    f"rel_v={nearest[5]:+.1f} m/s"
                ),
                (20, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

        cv2.putText(
            debug,
            (
                f"radar={len(radar_points)}  "
                f"projected={len(projected)}"
            ),
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        output = Image()

        output.header = (
            camera_header
        )

        output.height = (
            debug.shape[0]
        )

        output.width = (
            debug.shape[1]
        )

        output.encoding = "bgr8"
        output.is_bigendian = False

        output.step = (
            debug.shape[1]
            * 3
        )

        output.data = (
            debug.tobytes()
        )

        self.debug_pub.publish(
            output
        )

    def publish_cloud(
        self,
        source: PointCloud2,
        projected: np.ndarray,
    ) -> None:

        names = [
            "u_px",
            "v_px",
            "x_forward_m",
            "y_left_m",
            "z_up_m",
            "relative_speed_mps",
        ]

        fields = [
            PointField(
                name=name,
                offset=index * 4,
                datatype=(
                    PointField.FLOAT32
                ),
                count=1,
            )
            for index, name
            in enumerate(names)
        ]

        header = source.header

        header.frame_id = (
            "vehicle"
        )

        cloud = (
            point_cloud2.create_cloud(
                header,
                fields,
                projected.tolist(),
            )
        )

        self.projected_pub.publish(
            cloud
        )


def main() -> None:

    rclpy.init()

    node = (
        RadarCameraProjection()
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# ============================================================
# Mini Experiment
#
# 1. CAMERA_FOV_DEG
#    90 → 70 → 110
#
# 2. MAX_RANGE_M
#    80 → 30
#
# Projection pixel 위치와 사용되는 Radar Point 수를 비교하세요.
# ============================================================
