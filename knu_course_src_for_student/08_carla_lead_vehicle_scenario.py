#!/usr/bin/env python3
"""
Practice 08: CARLA Lead Vehicle Scenario

목표
- Ego Vehicle 앞에 Lead Vehicle을 배치합니다.
- Vehicle의 Forward Vector를 이용해 상대 위치를 계산합니다.
- Camera / Radar에서 Lead Vehicle이 실제로 관측되는지 확인합니다.
- 초기 거리 변화에 따른 Sensor 결과를 비교합니다.

CARLA 기본 연결:
    127.0.0.1:2000
"""

from __future__ import annotations

import argparse

import carla


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. Lead Vehicle 초기 거리
#
# Ego Vehicle 기준 전방에 Lead Vehicle을 배치합니다.
# 기본 거리: 20 m
# ------------------------------------------------------------

## TODO 1
DEFAULT_DISTANCE_M = None


# ------------------------------------------------------------
# Practice 2. 최소 안전 Spawn 거리
#
# 너무 가까우면 차량이 겹치거나 Spawn이 실패할 수 있습니다.
# 이번 실습의 최소 거리: 8 m
# ------------------------------------------------------------

## TODO 2
MIN_DISTANCE_M = None


# ------------------------------------------------------------
# Practice 3. Lead Vehicle Blueprint
#
# CARLA Blueprint ID:
# vehicle.lincoln.mkz_2020
# ------------------------------------------------------------

## TODO 3
LEAD_VEHICLE_BLUEPRINT = None


def main() -> int:
    check_completed(
        "Practice 1: DEFAULT_DISTANCE_M",
        DEFAULT_DISTANCE_M,
    )

    check_completed(
        "Practice 2: MIN_DISTANCE_M",
        MIN_DISTANCE_M,
    )

    check_completed(
        "Practice 3: LEAD_VEHICLE_BLUEPRINT",
        LEAD_VEHICLE_BLUEPRINT,
    )

    parser = argparse.ArgumentParser()

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
        "--distance",
        type=float,
        default=DEFAULT_DISTANCE_M,
        help="Ego-to-lead nominal distance [m]",
    )

    args = parser.parse_args()

    if args.distance < MIN_DISTANCE_M:
        parser.error(
            f"--distance must be at least {MIN_DISTANCE_M:g} m"
        )

    # ------------------------------------------------------------
    # CARLA RPC
    # ------------------------------------------------------------
    client = carla.Client(
        args.host,
        args.port,
    )

    client.set_timeout(
        10.0
    )

    world = client.get_world()

    # ------------------------------------------------------------
    # Ego Vehicle 검색
    #
    # Lab 07에서 생성한 role_name="knu_hero" 차량을 찾습니다.
    # ------------------------------------------------------------
    heroes = [
        actor
        for actor in world.get_actors().filter("vehicle.*")
        if actor.attributes.get("role_name") == "knu_hero"
    ]

    if not heroes:
        print(
            "[FAIL] knu_hero가 없습니다."
        )
        print(
            "먼저 07_carla_camera_radar_bridge.py를 실행하세요."
        )
        return 1

    ego = heroes[0]

    # ------------------------------------------------------------
    # 이전 실습에서 남아 있는 knu_lead 제거
    # ------------------------------------------------------------
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "knu_lead":
            actor.destroy()

    # ------------------------------------------------------------
    # Ego Transform
    #
    # get_forward_vector()
    # → 현재 차량 Heading 방향의 단위 벡터
    #
    # Lead 위치:
    # Ego Location + Forward Vector × Distance
    # ------------------------------------------------------------
    ego_transform = ego.get_transform()

    forward = (
        ego_transform.get_forward_vector()
    )

    lead_location = (
        ego_transform.location
        + carla.Location(
            x=forward.x * args.distance,
            y=forward.y * args.distance,
            z=0.5,
        )
    )

    lead_transform = carla.Transform(
        lead_location,
        ego_transform.rotation,
    )

    # ------------------------------------------------------------
    # Lead Vehicle 생성
    # ------------------------------------------------------------
    blueprint = (
        world
        .get_blueprint_library()
        .find(LEAD_VEHICLE_BLUEPRINT)
    )

    blueprint.set_attribute(
        "role_name",
        "knu_lead",
    )

    lead = world.try_spawn_actor(
        blueprint,
        lead_transform,
    )

    if lead is None:
        print(
            "[FAIL] Lead Vehicle Spawn 위치가 사용 중입니다."
        )
        print(
            "예: --distance 25 로 다시 실행하세요."
        )
        return 1

    # 이번 실습에서는 Lead Vehicle을 정지 상태로 유지
    lead.set_simulate_physics(
        False
    )

    # ------------------------------------------------------------
    # Result
    # ------------------------------------------------------------
    actual_distance = (
        lead.get_location()
        .distance(
            ego.get_location()
        )
    )

    print()
    print("=" * 60)
    print("CARLA Lead Vehicle Scenario")
    print("=" * 60)

    print(
        "ego id          :",
        ego.id,
    )

    print(
        "lead id         :",
        lead.id,
    )

    print(
        "blueprint       :",
        LEAD_VEHICLE_BLUEPRINT,
    )

    print(
        "nominal distance:",
        f"{args.distance:.1f} m",
    )

    print(
        "actual distance :",
        f"{actual_distance:.1f} m",
    )

    print()
    print(
        "[PASS] stationary knu_lead spawned."
    )

    print(
        "Lead Vehicle은 CARLA World에 남아 있습니다."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )


# ------------------------------------------------------------
# 실행
#
# 전제:
# 07_carla_camera_radar_bridge.py가 실행 중이어야 합니다.
#
# python3 08_carla_lead_vehicle_scenario.py
#
#
# Camera 확인
# ros2 run rqt_image_view rqt_image_view
#
# Topic:
# /carla/hero/camera_front/image
#
#
# Radar 확인
# rviz2
#
# Fixed Frame:
#   vehicle
#
# PointCloud2:
#   /carla/hero/radar/point_cloud
#
#
# Mini Experiment — Lead Vehicle Distance
#
# python3 08_carla_lead_vehicle_scenario.py --distance 12
# python3 08_carla_lead_vehicle_scenario.py --distance 20
# python3 08_carla_lead_vehicle_scenario.py --distance 30
#
# 각 거리에서:
#
# 1. Camera에서 Lead Vehicle 크기 비교
# 2. RViz Radar Point 위치 비교
# 3. Lead Vehicle이 멀어질수록 어떤 Sensor 특성이 변하는지 확인
# ------------------------------------------------------------
