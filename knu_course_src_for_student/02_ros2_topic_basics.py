#!/usr/bin/env python3
"""
Practice 02: ROS2 Topic Basics

목표
- ROS2 Publisher / Subscriber 구조를 이해합니다.
- Vehicle Speed Signal을 Topic으로 송수신합니다.
- Publish Period와 Frequency의 관계를 확인합니다.
- ROS2 CLI로 실제 통신 상태를 검증합니다.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def check_completed(name: str, value) -> None:
    if value is None:
        raise NotImplementedError(
            f"{name}가 아직 완성되지 않았습니다. 해당 ## TODO를 확인하세요."
        )


# ------------------------------------------------------------
# Practice 1. ROS2 Topic 이름
#
# Topic: Node 간 Message를 전달하는 통신 채널
# 이번 실습에서는 /vehicle/speed_mps 사용
# ------------------------------------------------------------

## TODO 1
TOPIC_NAME = None


# ------------------------------------------------------------
# Practice 2. Publish 주기
#
# create_timer(period_sec, callback)
#
# frequency [Hz] = 1 / period [sec]
# 0.2 sec → 5 Hz
# ------------------------------------------------------------

## TODO 2
PUBLISH_PERIOD_SEC = None


# ------------------------------------------------------------
# Practice 3. Vehicle Speed 증가량
#
# Callback이 실행될 때마다 +0.5 m/s
# ------------------------------------------------------------

## TODO 3
SPEED_STEP_MPS = None


SPEED_WRAP_MPS = 15.0


class SpeedPublisher(Node):
    """Vehicle Speed를 ROS2 Topic으로 전송하는 Node."""

    def __init__(self) -> None:
        check_completed("Practice 1: TOPIC_NAME", TOPIC_NAME)
        check_completed("Practice 2: PUBLISH_PERIOD_SEC", PUBLISH_PERIOD_SEC)
        check_completed("Practice 3: SPEED_STEP_MPS", SPEED_STEP_MPS)

        super().__init__("speed_publisher")

        # create_publisher(type, topic, queue_depth)
        self.publisher = self.create_publisher(
            Float32,
            TOPIC_NAME,
            10,
        )

        self.speed = 0.0

        # create_timer(period_sec, callback)
        self.create_timer(
            PUBLISH_PERIOD_SEC,
            self.publish_speed,
        )

        self.get_logger().info(
            f"Publisher started | "
            f"topic={TOPIC_NAME} | "
            f"expected={1.0 / PUBLISH_PERIOD_SEC:.1f} Hz"
        )

    def publish_speed(self) -> None:
        # Modulo 연산으로 15 m/s 이후 다시 0부터 반복
        self.speed = (
            self.speed + SPEED_STEP_MPS
        ) % SPEED_WRAP_MPS

        # Float32(data=value): ROS2 Float32 Message 생성
        message = Float32(data=self.speed)

        self.publisher.publish(message)

        self.get_logger().info(
            f"TX speed={self.speed:.1f} m/s"
        )


class SpeedSubscriber(Node):
    """Vehicle Speed Topic을 수신하는 Node."""

    def __init__(self) -> None:
        check_completed("Practice 1: TOPIC_NAME", TOPIC_NAME)

        super().__init__("speed_subscriber")

        # create_subscription(type, topic, callback, queue_depth)
        self.create_subscription(
            Float32,
            TOPIC_NAME,
            self.receive,
            10,
        )

        self.get_logger().info(
            f"Subscriber started | topic={TOPIC_NAME}"
        )

    def receive(self, message: Float32) -> None:
        # message.data: Float32 Message 내부의 실제 값
        self.get_logger().info(
            f"RX speed={message.data:.1f} m/s"
        )


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        choices=("pub", "sub"),
        required=True,
    )

    args = parser.parse_args()

    # ROS2 Python Client 초기화
    rclpy.init()

    node = (
        SpeedPublisher()
        if args.mode == "pub"
        else SpeedSubscriber()
    )

    try:
        # Timer / Subscription Callback을 계속 처리
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ------------------------------------------------------------
# 실행 방법
#
# Terminal 1
# python3 02_ros2_topic_basics.py --mode pub
#
# Terminal 2
# python3 02_ros2_topic_basics.py --mode sub
#
# Terminal 3
# ros2 node list
# ros2 topic list
# ros2 topic type /vehicle/speed_mps
# ros2 topic info /vehicle/speed_mps
# ros2 topic echo /vehicle/speed_mps
# ros2 topic hz /vehicle/speed_mps
#
# Mini Practice
# PUBLISH_PERIOD_SEC = 0.2 → 약 5 Hz
# PUBLISH_PERIOD_SEC = 0.1 → 약 10 Hz
# ros2 topic hz로 직접 비교하세요.
# ------------------------------------------------------------
