#!/usr/bin/env python3
"""
Practice 14: CARLA Live Camera Object Detection

목표
----
- CARLA Camera Image를 YOLO에 입력합니다.
- Object의 2D Bounding Box와 Confidence를 확인합니다.
- Detection 결과를 ROS2 vision_msgs로 Publish합니다.
- Confidence / Frame Skip에 따른 결과 변화를 확인합니다.

Pipeline
--------
CARLA Camera
    ↓
YOLO11n CARLA Detector
    ↓
2D Bounding Box
    ↓
Detection2DArray
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
import torch

from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from _course_common import course_data_root


def check_completed(
    name: str,
    value,
) -> None:

    if value is None:

        raise NotImplementedError(
            (
                f"{name}가 아직 완성되지 않았습니다. "
                "해당 ## TODO를 확인하세요."
            )
        )


# ============================================================
# Practice 1. ROS2 Camera Topic
#
# Lab 07에서 Front Camera가 Publish하는 Topic
# ============================================================

## TODO 1
CAMERA_TOPIC = None


DETECTION_TOPIC = (
    "/carla/object_detection_2d/bounding_box"
)

DEBUG_TOPIC = (
    "/carla/object_detection_2d/debug_image"
)


# ============================================================
# Practice 2. Detection Confidence
#
# Confidence Threshold
# → 낮을수록 더 많은 Object를 검출
# → 너무 낮으면 False Positive가 증가할 수 있음
#
# 기본값: 0.20
# ============================================================

## TODO 2
CONF_THRESHOLD = None


# ============================================================
# Practice 3. IoU Threshold
#
# NMS에서 겹치는 Bounding Box를 정리할 때 사용
#
# 기본값: 0.45
# ============================================================

## TODO 3
IOU_THRESHOLD = None


# ============================================================
# Practice 4. Frame Skip
#
# Camera = 약 20 Hz
#
# 1 → 모든 Frame 처리
# 2 → 약 절반의 Frame 처리
# 4 → 약 1/4 Frame 처리
#
# 기본값: 2
# ============================================================

## TODO 4
PROCESS_EVERY_N = None


MODEL_PATH = (
    course_data_root()
    / "models"
    / "yolo11n_carla.pt"
)

IMAGE_SIZE = 640


class CarlaYoloDetection(Node):

    def __init__(self) -> None:

        check_completed(
            "Practice 1: CAMERA_TOPIC",
            CAMERA_TOPIC,
        )

        check_completed(
            "Practice 2: CONF_THRESHOLD",
            CONF_THRESHOLD,
        )

        check_completed(
            "Practice 3: IOU_THRESHOLD",
            IOU_THRESHOLD,
        )

        check_completed(
            "Practice 4: PROCESS_EVERY_N",
            PROCESS_EVERY_N,
        )

        super().__init__(
            "carla_yolo_detection"
        )

        try:

            from ultralytics import YOLO

        except ImportError as error:

            raise RuntimeError(
                (
                    "Ultralytics is not available. "
                    "Run this Lab in Conda environment 'yolo'."
                )
            ) from error

        self.frame_count = 0

        # ----------------------------------------------------
        # GPU / CPU
        # ----------------------------------------------------

        if torch.cuda.is_available():

            self.device = 0

            self.get_logger().info(
                (
                    "GPU: "
                    f"{torch.cuda.get_device_name(0)}"
                )
            )

        else:

            self.device = "cpu"

            self.get_logger().warning(
                "CUDA unavailable - using CPU"
            )

        # ----------------------------------------------------
        # YOLO Model
        # ----------------------------------------------------

        self.model = YOLO(
            str(MODEL_PATH)
        )

        self.get_logger().info(
            f"model={MODEL_PATH.name}"
        )

        self.get_logger().info(
            f"classes={self.model.names}"
        )

        # ----------------------------------------------------
        # ROS2 I/O
        # ----------------------------------------------------

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.on_image,
            10,
        )

        self.detection_pub = (
            self.create_publisher(
                Detection2DArray,
                DETECTION_TOPIC,
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

    # ========================================================
    # ROS Image → BGR
    # ========================================================

    def image_to_bgr(
        self,
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

        if encoding == "rgb8":

            return cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR,
            )

        return image[
            :, :, :3
        ].copy()

    # ========================================================
    # Camera Callback
    # ========================================================

    def on_image(
        self,
        msg: Image,
    ) -> None:

        self.frame_count += 1

        # process_every_n:
        # 계산 부하를 줄이기 위한 Frame Skip
        if (
            self.frame_count
            % PROCESS_EVERY_N
            != 0
        ):
            return

        bgr = self.image_to_bgr(
            msg
        )

        if bgr is None:
            return

        # ----------------------------------------------------
        # YOLO Inference
        #
        # imgsz=640:
        # Ultralytics가 내부에서 letterbox resize를 수행합니다.
        # ----------------------------------------------------

        start = time.perf_counter()

        with torch.inference_mode():

            results = self.model.predict(
                bgr,
                imgsz=IMAGE_SIZE,
                conf=CONF_THRESHOLD,
                iou=IOU_THRESHOLD,
                device=self.device,
                verbose=False,
            )

        inference_ms = (
            time.perf_counter()
            - start
        ) * 1000.0

        output = Detection2DArray()

        output.header = (
            msg.header
        )

        debug = bgr.copy()

        object_count = 0

        if (
            results
            and results[0].boxes
            is not None
        ):

            result = results[0]

            boxes = (
                result.boxes.xyxy
                .detach()
                .float()
                .cpu()
                .numpy()
            )

            scores = (
                result.boxes.conf
                .detach()
                .float()
                .cpu()
                .numpy()
            )

            class_ids = (
                result.boxes.cls
                .detach()
                .int()
                .cpu()
                .numpy()
            )

            names = result.names

            for (
                box,
                score,
                class_id,
            ) in zip(
                boxes,
                scores,
                class_ids,
            ):

                x1, y1, x2, y2 = map(
                    float,
                    box,
                )

                width = max(
                    0.0,
                    x2 - x1,
                )

                height = max(
                    0.0,
                    y2 - y1,
                )

                center_x = (
                    x1
                    + 0.5 * width
                )

                center_y = (
                    y1
                    + 0.5 * height
                )

                class_name = str(
                    names.get(
                        int(class_id),
                        int(class_id),
                    )
                )

                # --------------------------------------------
                # vision_msgs/Detection2D
                # --------------------------------------------

                detection = Detection2D()

                detection.header = (
                    msg.header
                )

                bbox = BoundingBox2D()

                bbox.center.position.x = (
                    float(center_x)
                )

                bbox.center.position.y = (
                    float(center_y)
                )

                bbox.size_x = float(
                    width
                )

                bbox.size_y = float(
                    height
                )

                detection.bbox = (
                    bbox
                )

                hypothesis = (
                    ObjectHypothesisWithPose()
                )

                hypothesis.hypothesis.class_id = (
                    class_name
                )

                hypothesis.hypothesis.score = (
                    float(score)
                )

                detection.results.append(
                    hypothesis
                )

                output.detections.append(
                    detection
                )

                # --------------------------------------------
                # Debug Bounding Box
                # --------------------------------------------

                cv2.rectangle(
                    debug,
                    (
                        int(x1),
                        int(y1),
                    ),
                    (
                        int(x2),
                        int(y2),
                    ),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    debug,
                    (
                        f"{class_name} "
                        f"{float(score):.2f}"
                    ),
                    (
                        int(x1),
                        max(
                            20,
                            int(y1) - 6,
                        ),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                object_count += 1

        self.detection_pub.publish(
            output
        )

        # ----------------------------------------------------
        # Engineering Information
        # ----------------------------------------------------

        cv2.putText(
            debug,
            (
                f"objects={object_count}  "
                f"inference={inference_ms:.1f} ms"
            ),
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # ----------------------------------------------------
        # Debug Image
        # ----------------------------------------------------

        debug_msg = Image()

        debug_msg.header = (
            msg.header
        )

        debug_msg.height = int(
            debug.shape[0]
        )

        debug_msg.width = int(
            debug.shape[1]
        )

        debug_msg.encoding = "bgr8"

        debug_msg.is_bigendian = False

        debug_msg.step = (
            debug.shape[1]
            * 3
        )

        debug_msg.data = (
            debug.tobytes()
        )

        self.debug_pub.publish(
            debug_msg
        )


def main() -> None:

    rclpy.init()

    node = (
        CarlaYoloDetection()
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
# conda activate yolo
#
# python3 14_carla_yolo_detection.py
#
#
# 결과 확인
#
# ros2 run rqt_image_view rqt_image_view
#
# Topic:
# /carla/object_detection_2d/debug_image
#
#
# Detection Message
#
# ros2 topic echo \
# /carla/object_detection_2d/bounding_box \
# --once
#
#
# Mini Experiment 1
#
# CONF_THRESHOLD
# 0.20 → 0.50
#
# 검출되는 Object와 Confidence 변화를 비교합니다.
#
#
# Mini Experiment 2
#
# PROCESS_EVERY_N
# 1 → 2 → 4
#
# ros2 topic hz를 이용해 Detection Publish Rate를 비교합니다.
# ============================================================
