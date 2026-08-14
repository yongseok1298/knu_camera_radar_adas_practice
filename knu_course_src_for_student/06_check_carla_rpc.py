#!/usr/bin/env python3
"""
Practice 06: CARLA RPC Connection

목표
- CARLA의 Server–Client 구조를 이해합니다.
- Python API로 CARLA Server에 RPC 연결합니다.
- Server Version / Map / Actor 정보를 확인합니다.
- Host / Port / Timeout 설정의 의미를 확인합니다.
"""

from __future__ import annotations

import argparse


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. CARLA RPC 주소
#
# carla.Client(host, port)
# → 실행 중인 CARLA Server와 연결
#
# Local Server 기본값:
#   host = 127.0.0.1
#   port = 2000
# ------------------------------------------------------------

## TODO 1
DEFAULT_HOST = None
DEFAULT_PORT = None


# ------------------------------------------------------------
# Practice 2. RPC Timeout
#
# client.set_timeout(seconds)
# → Server 응답을 기다릴 최대 시간
#
# 이번 실습 기본값: 5.0 sec
# ------------------------------------------------------------

## TODO 2
DEFAULT_TIMEOUT_SEC = None


def main() -> int:
    check_completed(
        "Practice 1: DEFAULT_HOST",
        DEFAULT_HOST,
    )
    check_completed(
        "Practice 1: DEFAULT_PORT",
        DEFAULT_PORT,
    )
    check_completed(
        "Practice 2: DEFAULT_TIMEOUT_SEC",
        DEFAULT_TIMEOUT_SEC,
    )

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
    )

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SEC,
    )

    args = parser.parse_args()

    # ------------------------------------------------------------
    # CARLA Python API
    # ------------------------------------------------------------
    try:
        import carla

    except ImportError:
        print(
            "[FAIL] CARLA Python API를 import할 수 없습니다."
        )
        return 1

    # ------------------------------------------------------------
    # Practice 3. CARLA Client 생성
    #
    # 사용할 함수:
    # carla.Client(host, port)
    #
    # 어떤 값으로 Server에 접속해야 하는지 연결하세요.
    # ------------------------------------------------------------

    ## TODO 3
    client_host = None
    client_port = None

    check_completed(
        "Practice 3: client_host",
        client_host,
    )

    check_completed(
        "Practice 3: client_port",
        client_port,
    )

    try:
        client = carla.Client(
            client_host,
            client_port,
        )

        # RPC 응답 최대 대기 시간
        client.set_timeout(
            args.timeout
        )

        # --------------------------------------------------------
        # CARLA Server 정보 요청
        # --------------------------------------------------------
        version = client.get_server_version()
        world = client.get_world()

        map_name = world.get_map().name
        actors = world.get_actors()

    except RuntimeError as error:
        print(
            f"[FAIL] CARLA RPC "
            f"{args.host}:{args.port}: {error}"
        )
        return 1

    # ------------------------------------------------------------
    # Result
    # ------------------------------------------------------------
    print()
    print("=" * 60)
    print("CARLA RPC Connection")
    print("=" * 60)

    print(
        "RPC address    :",
        f"{args.host}:{args.port}",
    )

    print(
        "timeout        :",
        f"{args.timeout:.1f} sec",
    )

    print(
        "server version :",
        version,
    )

    print(
        "map            :",
        map_name,
    )

    print(
        "actors         :",
        len(actors),
    )

    print()
    print(
        "[PASS] CARLA Server is reachable."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )


# ------------------------------------------------------------
# 실행
#
# 1. CARLA Server
#
# cd ~/carla
# ./CarlaUE4.sh
#
#
# 2. RPC 확인
#
# python3 06_check_carla_rpc.py
#
#
# Mini Experiment 1. 잘못된 Port
#
# python3 06_check_carla_rpc.py --port 2001 --timeout 2
#
# → Server에 연결되지 않을 때 어떤 Error가 발생하는지 확인
#
#
# Mini Experiment 2. 정상 Port
#
# python3 06_check_carla_rpc.py --port 2000 --timeout 2
#
# → 다시 정상적으로 연결되는지 확인
#
#
# Mini Experiment 3. Actor 수
#
# CARLA에서 Vehicle을 추가한 뒤 다시 실행하여
# actors 값이 변하는지 확인하세요.
# ------------------------------------------------------------
