#!/usr/bin/env python3
"""
Challenge 18: Goal Pose + GNSS + Global Path Planning

목표
----
1. 현재 knu_hero 위치를 Start로 설정합니다.
2. RViz에서 사용자가 Goal Pose를 지정합니다.
3. Start / Goal을 CARLA Driving Waypoint로 변환합니다.
4. CARLA/OpenDRIVE road topology를 이용해 Global Route를 생성합니다.
5. 생성된 Route를 /carla/path/global 로 Publish합니다.

Pipeline
--------
Current knu_hero
       │
       ├── GNSS
       │
       ▼
Start Waypoint
       │
       │       RViz Goal Pose
       │             ↓
       │       Goal Waypoint
       │             │
       └───────┬─────┘
               ↓
       Global Route Planner
               ↓
       /carla/path/global

Coordinate Contract
-------------------
CARLA world → ROS map

x_ros =  x_carla
y_ros = -y_carla
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import carla
import rclpy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from sensor_msgs.msg import (
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


# ============================================================
# Course Contract
# ============================================================

EXPECTED_MAP = "Town04"

EGO_ROLE_NAME = "knu_hero"

MAP_FRAME = "map"
GNSS_FRAME = "gnss"


GNSS_TOPIC = (
    "/carla/hero/gnss"
)

EGO_MAP_POSE_TOPIC = (
    "/carla/hero/map_pose"
)

LANE_NETWORK_TOPIC = (
    "/carla/map/lane_center_points"
)

GLOBAL_PATH_TOPIC = (
    "/carla/path/global"
)

SNAPPED_GOAL_TOPIC = (
    "/carla/goal_pose_snapped"
)

STATUS_TOPIC = (
    "/carla/path/global_status"
)


# RViz Goal Pose
GOAL_TOPIC = (
    "/goal_pose"
)


# ============================================================
# Parameters
# ============================================================

ROUTE_RESOLUTION_M = 2.0

LANE_NETWORK_RESOLUTION_M = 2.0

GNSS_SENSOR_TICK_SEC = 0.10

GNSS_X_M = 0.0
GNSS_Y_M = 0.0
GNSS_Z_M = 2.0


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
# Coordinate Conversion
# ============================================================

def carla_to_ros_xyz(
    location: carla.Location,
):

    return (
        float(location.x),
        float(-location.y),
        float(location.z),
    )


def ros_to_carla_location(
    x_ros: float,
    y_ros: float,
    z_ros: float = 0.0,
) -> carla.Location:

    return carla.Location(
        x=float(x_ros),
        y=float(-y_ros),
        z=float(z_ros),
    )


def carla_yaw_to_ros_yaw(
    yaw_deg: float,
) -> float:

    return -math.radians(
        float(yaw_deg)
    )


def yaw_to_quaternion(
    yaw_rad: float,
):

    half = (
        yaw_rad
        / 2.0
    )

    return (
        0.0,
        0.0,
        math.sin(half),
        math.cos(half),
    )


# ============================================================
# Time
# ============================================================

def time_message(
    seconds: float,
) -> Time:

    sec = int(seconds)

    return Time(
        sec=sec,
        nanosec=int(
            (seconds - sec)
            * 1_000_000_000
        ),
    )


# ============================================================
# CARLA Helpers
# ============================================================

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


def load_global_route_planner():

    candidates = [
        (
            Path.home()
            / "carla"
            / "PythonAPI"
            / "carla"
        ),
        (
            Path.home()
            / "carla"
            / "PythonAPI"
        ),
    ]

    for path in candidates:

        if (
            path.exists()
            and str(path)
            not in sys.path
        ):

            sys.path.insert(
                0,
                str(path),
            )

    from agents.navigation.global_route_planner import (
        GlobalRoutePlanner,
    )

    return GlobalRoutePlanner


# ============================================================
# ROS Pose Conversion
# ============================================================

def transform_to_pose(
    transform: carla.Transform,
    stamp,
) -> PoseStamped:

    x, y, z = (
        carla_to_ros_xyz(
            transform.location
        )
    )

    yaw = (
        carla_yaw_to_ros_yaw(
            transform.rotation.yaw
        )
    )

    (
        qx,
        qy,
        qz,
        qw,
    ) = yaw_to_quaternion(
        yaw
    )

    pose = PoseStamped()

    pose.header.stamp = stamp
    pose.header.frame_id = MAP_FRAME

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


# ============================================================
# Node
# ============================================================

class GoalPoseGlobalPath(Node):

    def __init__(self) -> None:

        super().__init__(
            "goal_pose_global_path"
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
                f"Town04가 필요합니다. "
                f"현재={self.carla_map.name}"
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

        # ----------------------------------------------------
        # Global Route Planner
        # ----------------------------------------------------

        GlobalRoutePlanner = (
            load_global_route_planner()
        )

        self.route_planner = (
            GlobalRoutePlanner(
                self.carla_map,
                ROUTE_RESOLUTION_M,
            )
        )

        self.current_route = []

        self.gnss_sensor = None

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.gnss_pub = (
            self.create_publisher(
                NavSatFix,
                GNSS_TOPIC,
                10,
            )
        )

        self.ego_pose_pub = (
            self.create_publisher(
                PoseStamped,
                EGO_MAP_POSE_TOPIC,
                10,
            )
        )

        self.lane_pub = (
            self.create_publisher(
                PointCloud2,
                LANE_NETWORK_TOPIC,
                1,
            )
        )

        self.path_pub = (
            self.create_publisher(
                RosPath,
                GLOBAL_PATH_TOPIC,
                10,
            )
        )

        self.goal_pub = (
            self.create_publisher(
                PoseStamped,
                SNAPPED_GOAL_TOPIC,
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

        # ----------------------------------------------------
        # Goal
        # ----------------------------------------------------

        self.create_subscription(
            PoseStamped,
            GOAL_TOPIC,
            self.on_goal,
            10,
        )

        # ----------------------------------------------------
        # GNSS
        # ----------------------------------------------------

        self.spawn_gnss()

        # CARLA lane network는 제공 코드로 시각화
        self.lane_points = (
            self.build_lane_network()
        )

        self.create_timer(
            1.0,
            self.publish_lane_network,
        )

        self.create_timer(
            0.10,
            self.publish_ego_pose,
        )

        # Global Path를 1 Hz로 재발행하여
        # Lab 19/20이 나중에 시작돼도 Route를 수신하도록 합니다.
        self.create_timer(
            1.0,
            self.publish_global_path,
        )

        self.get_logger().info(
            "Challenge 18 ready."
        )

        self.get_logger().info(
            "RViz에서 Goal Pose를 지정하세요."
        )

    # ========================================================
    # GNSS
    # ========================================================

    def spawn_gnss(
        self,
    ) -> None:

        blueprint = (
            self.world
            .get_blueprint_library()
            .find(
                "sensor.other.gnss"
            )
        )

        blueprint.set_attribute(
            "sensor_tick",
            str(
                GNSS_SENSOR_TICK_SEC
            ),
        )

        transform = (
            carla.Transform(
                carla.Location(
                    x=GNSS_X_M,
                    y=-GNSS_Y_M,
                    z=GNSS_Z_M,
                )
            )
        )

        self.gnss_sensor = (
            self.world.spawn_actor(
                blueprint,
                transform,
                attach_to=self.ego,
            )
        )

        self.gnss_sensor.listen(
            self.on_gnss
        )

    def on_gnss(
        self,
        measurement: carla.GnssMeasurement,
    ) -> None:

        msg = NavSatFix()

        msg.header = Header(
            stamp=time_message(
                measurement.timestamp
            ),
            frame_id=GNSS_FRAME,
        )

        msg.status.status = (
            NavSatStatus.STATUS_FIX
        )

        msg.status.service = (
            NavSatStatus.SERVICE_GPS
        )

        msg.latitude = float(
            measurement.latitude
        )

        msg.longitude = float(
            measurement.longitude
        )

        msg.altitude = float(
            measurement.altitude
        )

        self.gnss_pub.publish(
            msg
        )

    # ========================================================
    # Lane Network — provided
    # ========================================================

    def build_lane_network(
        self,
    ):

        waypoints = (
            self.carla_map
            .generate_waypoints(
                LANE_NETWORK_RESOLUTION_M
            )
        )

        points = []

        for waypoint in waypoints:

            points.append(
                carla_to_ros_xyz(
                    waypoint
                    .transform
                    .location
                )
            )

        return points

    def publish_lane_network(
        self,
    ) -> None:

        header = Header()

        header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        header.frame_id = MAP_FRAME

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
        ]

        cloud = (
            point_cloud2.create_cloud(
                header,
                fields,
                self.lane_points,
            )
        )

        self.lane_pub.publish(
            cloud
        )

    # ========================================================
    # Ego Pose
    # ========================================================

    def publish_ego_pose(
        self,
    ) -> None:

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose = transform_to_pose(
            self.ego.get_transform(),
            now,
        )

        self.ego_pose_pub.publish(
            pose
        )

    # ========================================================
    # Goal Callback
    # ========================================================

    def on_goal(
        self,
        msg: PoseStamped,
    ) -> None:

        self.get_logger().info(
            "Goal Pose received."
        )

        # ====================================================
        # TODO 1
        #
        # 현재 knu_hero 위치에서
        # Driving Waypoint를 찾으세요.
        #
        # Hint:
        #
        # self.carla_map.get_waypoint(
        #     ...,
        #     project_to_road=True,
        #     lane_type=carla.LaneType.Driving,
        # )
        # ====================================================

        ## TODO 1
        start_waypoint = None

        check_completed(
            "TODO 1: start_waypoint",
            start_waypoint,
        )

        # ====================================================
        # TODO 2
        #
        # RViz Goal Pose(map frame)를
        # CARLA Location으로 변환하세요.
        #
        # Hint:
        # ros_to_carla_location(x, y, z)
        # ====================================================

        ## TODO 2
        goal_location = None

        check_completed(
            "TODO 2: goal_location",
            goal_location,
        )

        # ====================================================
        # TODO 3
        #
        # Goal Location을 가장 가까운
        # Driving Waypoint로 Snap하세요.
        # ====================================================

        ## TODO 3
        goal_waypoint = None

        check_completed(
            "TODO 3: goal_waypoint",
            goal_waypoint,
        )

        # ====================================================
        # TODO 4
        #
        # GlobalRoutePlanner로
        # Start → Goal Route를 생성하세요.
        #
        # Hint:
        #
        # self.route_planner.trace_route(
        #     start_location,
        #     goal_location,
        # )
        # ====================================================

        ## TODO 4
        route = None

        check_completed(
            "TODO 4: route",
            route,
        )

        if len(route) == 0:

            self.get_logger().warning(
                "Route가 비어 있습니다."
            )

            return

        self.current_route = route

        self.publish_goal(
            goal_waypoint
        )

        self.publish_global_path()

        self.publish_status(
            start_waypoint,
            goal_waypoint,
        )

    # ========================================================
    # Goal
    # ========================================================

    def publish_goal(
        self,
        waypoint,
    ) -> None:

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.goal_pub.publish(
            transform_to_pose(
                waypoint.transform,
                now,
            )
        )

    # ========================================================
    # Global Path
    # ========================================================

    def publish_global_path(
        self,
    ) -> None:

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        path = RosPath()

        path.header.stamp = now
        path.header.frame_id = MAP_FRAME

        # ====================================================
        # TODO 5
        #
        # self.current_route의 각 waypoint를
        # PoseStamped로 변환해서
        # path.poses에 추가하세요.
        #
        # route element:
        # (waypoint, road_option)
        #
        # Helper:
        # transform_to_pose(...)
        # ====================================================

        ## TODO 5
        for waypoint, road_option in self.current_route:

            _ = (
                waypoint,
                road_option,
            )

            # 이 부분을 완성하세요.
            pass

        self.path_pub.publish(
            path
        )

    # ========================================================
    # Status
    # ========================================================

    def publish_status(
        self,
        start_waypoint,
        goal_waypoint,
    ) -> None:

        status = {
            "map": (
                self.carla_map.name
            ),
            "start_road_id": int(
                start_waypoint.road_id
            ),
            "start_lane_id": int(
                start_waypoint.lane_id
            ),
            "goal_road_id": int(
                goal_waypoint.road_id
            ),
            "goal_lane_id": int(
                goal_waypoint.lane_id
            ),
            "route_points": int(
                len(
                    self.current_route
                )
            ),
        }

        self.status_pub.publish(
            String(
                data=json.dumps(
                    status
                )
            )
        )

        self.get_logger().info(
            (
                "Global Path complete: "
                f"{len(self.current_route)} points"
            )
        )

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(
        self,
    ) -> bool:

        if (
            self.gnss_sensor
            is not None
            and self.gnss_sensor.is_alive
        ):

            try:
                self.gnss_sensor.stop()

            except Exception:
                pass

            try:
                self.gnss_sensor.destroy()

            except Exception:
                pass

        return super().destroy_node()


def main() -> None:

    rclpy.init()

    node = (
        GoalPoseGlobalPath()
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
# Challenge 18 — Success Criteria
#
# [ ] 현재 knu_hero → Start Waypoint
# [ ] RViz Goal → CARLA Goal Location
# [ ] Goal → Driving Waypoint Snap
# [ ] Start → Goal Route Planning
# [ ] /carla/path/global Publish
#
#
# RViz
# ----
# Fixed Frame = map
#
# PointCloud2:
# /carla/map/lane_center_points
#
# Pose:
# /carla/hero/map_pose
#
# Path:
# /carla/path/global
#
# Pose:
# /carla/goal_pose_snapped
#
#
# 필요한 경우 추가 Hint
# ----------------------
#
# GlobalRoutePlanner:
# self.route_planner.trace_route(start, goal)
#
# ROS Path:
# path.poses.append(pose)
# ============================================================
