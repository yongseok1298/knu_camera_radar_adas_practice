#!/usr/bin/env python3
"""Lab 08: place a stationary lead vehicle in front of the KNU ego vehicle."""
from __future__ import annotations

import argparse

import carla


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--distance", type=float, default=20.0)
    args = parser.parse_args()
    if args.distance < 8.0:
        parser.error("--distance must be at least 8 m")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    heroes = [a for a in world.get_actors().filter("vehicle.*")
              if a.attributes.get("role_name") == "knu_hero"]
    if not heroes:
        print("[FAIL] run 07_carla_camera_radar_bridge.py first")
        return 1
    ego = heroes[0]
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "knu_lead":
            actor.destroy()

    ego_transform = ego.get_transform()
    forward = ego_transform.get_forward_vector()
    location = ego_transform.location + carla.Location(
        x=forward.x * args.distance, y=forward.y * args.distance, z=0.5)
    transform = carla.Transform(location, ego_transform.rotation)
    blueprint = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
    blueprint.set_attribute("role_name", "knu_lead")
    lead = world.try_spawn_actor(blueprint, transform)
    if lead is None:
        print("[FAIL] lead vehicle spawn point is occupied; try --distance 25")
        return 1
    lead.set_simulate_physics(False)
    print(f"[PASS] stationary knu_lead id={lead.id}, nominal distance={args.distance:.1f} m")
    print("The actor remains in CARLA. Reload the map to remove all lab actors.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

