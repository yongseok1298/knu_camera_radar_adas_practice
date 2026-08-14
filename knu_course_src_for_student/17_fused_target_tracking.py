#!/usr/bin/env python3
"""
실습 17: 융합 객체 추적 및 MIO 선정

목표
----
- Camera-Radar 객체를 시간축으로 Tracking합니다.
- EMA로 객체 위치를 안정화합니다.
- 같은 Track의 Range 변화를 이용해 상대속도를 추정합니다.
- Ego lane의 대표 선행 객체(MIO)를 선정합니다.

MIO
---
Most Important Object

이번 실습에서는
ACC/AEB에 가장 중요한 Ego-lane 선행 객체를 의미합니다.

Relative Speed
--------------
순간 Radar Doppler 값을 그대로 사용하지 않고,

Range History
    ↓
ΔRange / Δt
    ↓
Median
    ↓
EMA

과정을 거쳐 제어용 상대속도를 추정합니다.

relative_speed < 0
→ Closing
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import (
    dataclass,
    field,
)

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import (
    Vector3Stamped,
)
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
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


# ============================================================
# Topics
# ============================================================

CAMERA_TOPIC = (
    "/carla/hero/camera_front/image"
)

FUSED_CANDIDATES_TOPIC = (
    "/adas/fused_candidates"
)

TRACKED_TARGETS_TOPIC = (
    "/adas/tracked_targets"
)

MIO_TOPIC = (
    "/adas/fused_target"
)

STATUS_TOPIC = (
    "/adas/tracking/status"
)

DEBUG_TOPIC = (
    "/adas/tracking/debug_image"
)


# ============================================================
# 실습 1. Tracking Parameters
#
# 기본값
# ------
# Position Association Gate : 2.0 m
# Position EMA Alpha        : 0.40
# Range-rate Window         : 5
# Range-rate EMA Alpha      : 0.35
# MIO Lateral Gate          : 2.0 m
# ============================================================

## TODO 1

ASSOCIATION_GATE_M = None

POSITION_EMA_ALPHA = None

RANGE_RATE_WINDOW = None

RANGE_RATE_EMA_ALPHA = None

MIO_LATERAL_GATE_M = None


MAX_TRACK_MISSES = 5

MIO_MAX_MISSES = 2

MIN_CONFIRM_HITS = 2

MIN_FORWARD_M = 0.5

MIN_RANGE_RATE_SAMPLES = 3


# ============================================================
# Visualization
# ============================================================

DEBUG_PERIOD_SEC = 0.10

IMAGE_QOS = QoSProfile(
    reliability=(
        ReliabilityPolicy.BEST_EFFORT
    ),
    history=(
        HistoryPolicy.KEEP_LAST
    ),
    depth=1,
)


# ============================================================
# Helpers
# ============================================================

def stamp_seconds(
    msg,
) -> float:

    stamp = (
        msg.header.stamp
    )

    return (
        float(stamp.sec)
        + float(stamp.nanosec)
        * 1e-9
    )


def image_to_bgr(
    msg: Image,
) -> np.ndarray | None:

    encoding = (
        msg.encoding.lower()
    )

    channels = (
        4
        if encoding in {
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


# ============================================================
# 실습 2. EMA
#
# value =
#     alpha * measurement
#     + (1-alpha) * previous
# ============================================================

def ema(
    previous: float,
    measurement: float,
    alpha: float,
) -> float:

    ## TODO 2
    value = None

    check_completed(
        "TODO 2: EMA",
        value,
    )

    return float(
        value
    )


# ============================================================
# 실습 3. Range-rate Estimation
#
# Range sample:
#
# t0, r0
# t1, r1
# t2, r2
#
# 각 구간:
#
# rate = (r1-r0) / (t1-t0)
#
# 여러 구간 rate의 median을 반환합니다.
# ============================================================

def estimate_range_rate(
    history,
) -> float | None:

    if (
        len(history)
        < MIN_RANGE_RATE_SAMPLES
    ):
        return None

    samples = list(
        history
    )

    rates = []

    for (
        previous,
        current,
    ) in zip(
        samples[:-1],
        samples[1:],
    ):

        t0, range0 = (
            previous
        )

        t1, range1 = (
            current
        )

        dt = (
            t1
            - t0
        )

        if dt <= 1e-3:
            continue

        # ================================================
        # TODO 3
        #
        # ΔRange / Δt
        # ================================================

        range_rate = None

        check_completed(
            "TODO 3: range_rate",
            range_rate,
        )

        rates.append(
            float(
                range_rate
            )
        )

    if len(rates) < 2:
        return None

    # 여러 구간의 대표값
    return float(
        np.median(
            np.asarray(
                rates,
                dtype=np.float64,
            )
        )
    )


def read_fused_candidates(
    msg: PointCloud2,
) -> np.ndarray:

    rows = []

    for point in point_cloud2.read_points(
        msg,
        field_names=(
            "x_forward_m",
            "y_left_m",
            "z_up_m",
            "relative_speed_mps",
            "detection_index",
            "detection_confidence",
            "u_px",
            "v_px",
        ),
        skip_nans=True,
    ):

        rows.append(
            [
                float(value)
                for value in point
            ]
        )

    if not rows:

        return np.empty(
            (0, 8),
            dtype=np.float32,
        )

    return np.asarray(
        rows,
        dtype=np.float32,
    )


# ============================================================
# Track
# ============================================================

@dataclass
class Track:

    track_id: int

    x_forward_m: float
    y_left_m: float
    z_up_m: float

    detection_confidence: float

    u_px: float
    v_px: float

    relative_speed_mps: float = 0.0

    raw_radar_velocity_mps: float = 0.0

    range_rate_ready: bool = False

    hits: int = 1
    misses: int = 0

    range_history: deque = field(
        default_factory=lambda:
            deque(
                maxlen=RANGE_RATE_WINDOW
            )
    )

    # ========================================================
    # 실습 4. Position Association Distance
    #
    # sqrt(dx² + dy²)
    # np.hypot(dx, dy)
    # ========================================================

    def distance_xy(
        self,
        measurement: np.ndarray,
    ) -> float:

        dx = (
            float(
                measurement[0]
            )
            - self.x_forward_m
        )

        dy = (
            float(
                measurement[1]
            )
            - self.y_left_m
        )

        ## TODO 4
        distance = None

        check_completed(
            "TODO 4: distance",
            distance,
        )

        return float(
            distance
        )

    def add_initial_range(
        self,
        timestamp: float,
        range_m: float,
    ) -> None:

        self.range_history.append(
            (
                timestamp,
                range_m,
            )
        )

    def update(
        self,
        measurement: np.ndarray,
        timestamp: float,
    ) -> None:

        measured_x = float(
            measurement[0]
        )

        measured_y = float(
            measurement[1]
        )

        measured_z = float(
            measurement[2]
        )

        raw_doppler = float(
            measurement[3]
        )

        confidence = float(
            measurement[5]
        )

        measured_u = float(
            measurement[6]
        )

        measured_v = float(
            measurement[7]
        )

        # ----------------------------------------------------
        # Range history
        # ----------------------------------------------------

        self.range_history.append(
            (
                timestamp,
                measured_x,
            )
        )

        range_rate = (
            estimate_range_rate(
                self.range_history
            )
        )

        # ----------------------------------------------------
        # Position smoothing
        # ----------------------------------------------------

        self.x_forward_m = ema(
            self.x_forward_m,
            measured_x,
            POSITION_EMA_ALPHA,
        )

        self.y_left_m = ema(
            self.y_left_m,
            measured_y,
            POSITION_EMA_ALPHA,
        )

        self.z_up_m = ema(
            self.z_up_m,
            measured_z,
            POSITION_EMA_ALPHA,
        )

        self.detection_confidence = ema(
            self.detection_confidence,
            confidence,
            POSITION_EMA_ALPHA,
        )

        self.u_px = ema(
            self.u_px,
            measured_u,
            POSITION_EMA_ALPHA,
        )

        self.v_px = ema(
            self.v_px,
            measured_v,
            POSITION_EMA_ALPHA,
        )

        # Raw Radar Doppler는 비교용
        self.raw_radar_velocity_mps = (
            raw_doppler
        )

        # ----------------------------------------------------
        # Range-rate smoothing
        # ----------------------------------------------------

        if range_rate is not None:

            if not self.range_rate_ready:

                self.relative_speed_mps = (
                    range_rate
                )

                self.range_rate_ready = (
                    True
                )

            else:

                self.relative_speed_mps = ema(
                    self.relative_speed_mps,
                    range_rate,
                    RANGE_RATE_EMA_ALPHA,
                )

        self.hits += 1

        self.misses = 0


# ============================================================
# Node
# ============================================================

class FusedTargetTracker(Node):

    def __init__(self) -> None:

        for name, value in (
            (
                "ASSOCIATION_GATE_M",
                ASSOCIATION_GATE_M,
            ),
            (
                "POSITION_EMA_ALPHA",
                POSITION_EMA_ALPHA,
            ),
            (
                "RANGE_RATE_WINDOW",
                RANGE_RATE_WINDOW,
            ),
            (
                "RANGE_RATE_EMA_ALPHA",
                RANGE_RATE_EMA_ALPHA,
            ),
            (
                "MIO_LATERAL_GATE_M",
                MIO_LATERAL_GATE_M,
            ),
        ):

            check_completed(
                f"TODO 1: {name}",
                value,
            )

        super().__init__(
            "fused_target_tracker"
        )

        self.tracks = []

        self.next_track_id = 1

        self.current_mio = None

        self.latest_camera = None

        self.latest_camera_header = None

        self.create_subscription(
            PointCloud2,
            FUSED_CANDIDATES_TOPIC,
            self.on_candidates,
            10,
        )

        self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.on_camera,
            IMAGE_QOS,
        )

        self.tracked_pub = (
            self.create_publisher(
                PointCloud2,
                TRACKED_TARGETS_TOPIC,
                10,
            )
        )

        self.mio_pub = (
            self.create_publisher(
                Vector3Stamped,
                MIO_TOPIC,
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

        self.debug_pub = (
            self.create_publisher(
                Image,
                DEBUG_TOPIC,
                IMAGE_QOS,
            )
        )

        self.create_timer(
            DEBUG_PERIOD_SEC,
            self.publish_debug,
        )

    # ========================================================
    # Camera
    # ========================================================

    def on_camera(
        self,
        msg: Image,
    ) -> None:

        image = image_to_bgr(
            msg
        )

        if image is None:
            return

        self.latest_camera = (
            image
        )

        self.latest_camera_header = (
            msg.header
        )

    # ========================================================
    # Tracking
    # ========================================================

    def on_candidates(
        self,
        msg: PointCloud2,
    ) -> None:

        timestamp = (
            stamp_seconds(
                msg
            )
        )

        measurements = (
            read_fused_candidates(
                msg
            )
        )

        for track in self.tracks:

            track.misses += 1

        unmatched = set(
            range(
                len(
                    measurements
                )
            )
        )

        # ----------------------------------------------------
        # Nearest-neighbor Association
        # ----------------------------------------------------

        for track in self.tracks:

            best_index = None

            best_distance = (
                float("inf")
            )

            for index in unmatched:

                distance = (
                    track.distance_xy(
                        measurements[
                            index
                        ]
                    )
                )

                if (
                    distance
                    < best_distance
                ):

                    best_distance = (
                        distance
                    )

                    best_index = (
                        index
                    )

            if (
                best_index is not None
                and best_distance
                <= ASSOCIATION_GATE_M
            ):

                track.update(
                    measurements[
                        best_index
                    ],
                    timestamp,
                )

                unmatched.remove(
                    best_index
                )

        # ----------------------------------------------------
        # New Track
        # ----------------------------------------------------

        for index in sorted(
            unmatched
        ):

            measurement = (
                measurements[
                    index
                ]
            )

            track = Track(
                track_id=(
                    self.next_track_id
                ),
                x_forward_m=float(
                    measurement[0]
                ),
                y_left_m=float(
                    measurement[1]
                ),
                z_up_m=float(
                    measurement[2]
                ),
                detection_confidence=float(
                    measurement[5]
                ),
                u_px=float(
                    measurement[6]
                ),
                v_px=float(
                    measurement[7]
                ),
                raw_radar_velocity_mps=float(
                    measurement[3]
                ),
            )

            track.add_initial_range(
                timestamp,
                float(
                    measurement[0]
                ),
            )

            self.tracks.append(
                track
            )

            self.next_track_id += 1

        self.tracks = [
            track
            for track in self.tracks
            if (
                track.misses
                <= MAX_TRACK_MISSES
            )
        ]

        # ----------------------------------------------------
        # 실습 5. MIO Selection
        # ----------------------------------------------------

        mio_candidates = [
            track
            for track in self.tracks
            if (
                track.hits
                >= MIN_CONFIRM_HITS
                and track.misses
                <= MIO_MAX_MISSES
                and track.x_forward_m
                > MIN_FORWARD_M
                and abs(
                    track.y_left_m
                )
                <= MIO_LATERAL_GATE_M
            )
        ]

        self.current_mio = (
            None
        )

        if mio_candidates:

            # ================================================
            # TODO 5
            #
            # x_forward_m이 가장 작은 Track을 선택하세요.
            # ================================================

            mio = None

            check_completed(
                "TODO 5: MIO",
                mio,
            )

            self.current_mio = (
                mio
            )

        self.publish_tracks(
            msg
        )

        if (
            self.current_mio
            is not None
        ):

            self.publish_mio(
                msg,
                self.current_mio,
            )

        self.publish_status(
            measurements
        )

    # ========================================================
    # Track Output
    # ========================================================

    def publish_tracks(
        self,
        source: PointCloud2,
    ) -> None:

        names = [
            "track_id",
            "x_forward_m",
            "y_left_m",
            "z_up_m",
            "relative_speed_mps",
            "detection_confidence",
            "u_px",
            "v_px",
            "hits",
            "misses",
            "raw_radar_velocity_mps",
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

        rows = []

        for track in self.tracks:

            rows.append(
                [
                    float(track.track_id),
                    float(track.x_forward_m),
                    float(track.y_left_m),
                    float(track.z_up_m),
                    float(
                        track.relative_speed_mps
                    ),
                    float(
                        track.detection_confidence
                    ),
                    float(track.u_px),
                    float(track.v_px),
                    float(track.hits),
                    float(track.misses),
                    float(
                        track.raw_radar_velocity_mps
                    ),
                ]
            )

        header = (
            source.header
        )

        header.frame_id = (
            "vehicle"
        )

        self.tracked_pub.publish(
            point_cloud2.create_cloud(
                header,
                fields,
                rows,
            )
        )

    # ========================================================
    # MIO Output
    # ========================================================

    def publish_mio(
        self,
        source: PointCloud2,
        mio: Track,
    ) -> None:

        output = Vector3Stamped()

        output.header = (
            source.header
        )

        output.header.frame_id = (
            "vehicle"
        )

        output.vector.x = float(
            mio.x_forward_m
        )

        output.vector.y = float(
            mio.y_left_m
        )

        # Track Range-rate
        output.vector.z = float(
            mio.relative_speed_mps
        )

        self.mio_pub.publish(
            output
        )

    # ========================================================
    # Status
    # ========================================================

    def publish_status(
        self,
        measurements,
    ) -> None:

        mio = (
            self.current_mio
        )

        status = {
            "measurements": int(
                len(
                    measurements
                )
            ),

            "active_tracks": int(
                len(
                    self.tracks
                )
            ),

            "relative_speed_source": (
                "tracked_range_rate"
            ),

            "mio_track_id": (
                None
                if mio is None
                else int(
                    mio.track_id
                )
            ),

            "mio_range_m": (
                None
                if mio is None
                else float(
                    mio.x_forward_m
                )
            ),

            "mio_relative_speed_mps": (
                None
                if mio is None
                else float(
                    mio.relative_speed_mps
                )
            ),

            "mio_raw_radar_velocity_mps": (
                None
                if mio is None
                else float(
                    mio.raw_radar_velocity_mps
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

    # ========================================================
    # Debug
    # ========================================================

    def publish_debug(
        self,
    ) -> None:

        if (
            self.latest_camera is None
            or self.latest_camera_header
            is None
        ):
            return

        debug = (
            self.latest_camera.copy()
        )

        for track in self.tracks:

            if (
                track.misses
                > MIO_MAX_MISSES
            ):
                continue

            cv2.putText(
                debug,
                f"T{track.track_id}",
                (
                    int(track.u_px) + 8,
                    int(track.v_px) + 20,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                2,
            )

        mio = (
            self.current_mio
        )

        if mio is not None:

            u = int(
                mio.u_px
            )

            v = int(
                mio.v_px
            )

            cv2.circle(
                debug,
                (
                    u,
                    v,
                ),
                12,
                (0, 255, 255),
                3,
            )

            cv2.putText(
                debug,
                (
                    f"MIO T{mio.track_id}  "
                    f"{mio.x_forward_m:.1f} m  "
                    f"{mio.relative_speed_mps:+.1f} m/s"
                ),
                (
                    20,
                    60,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 255),
                2,
            )

            cv2.putText(
                debug,
                (
                    "raw radar="
                    f"{mio.raw_radar_velocity_mps:+.1f} m/s"
                ),
                (
                    20,
                    85,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                2,
            )

        output = Image()

        output.header = (
            self.latest_camera_header
        )

        output.height = (
            debug.shape[0]
        )

        output.width = (
            debug.shape[1]
        )

        output.encoding = (
            "bgr8"
        )

        output.is_bigendian = (
            False
        )

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


def main() -> None:

    rclpy.init()

    node = (
        FusedTargetTracker()
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
# 확인
#
# /adas/tracking/status에서
#
# mio_relative_speed_mps
#     → Tracking 기반 Range-rate
#
# mio_raw_radar_velocity_mps
#     → CARLA Raw Radar Doppler
#
# 두 값을 비교합니다.
#
#
# Mini Experiment
# ---------------
#
# RANGE_RATE_WINDOW
# 3 → 5 → 7
#
# RANGE_RATE_EMA_ALPHA
# 0.2 → 0.35 → 0.7
# ============================================================
