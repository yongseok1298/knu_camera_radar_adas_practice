#!/usr/bin/env python3
"""
Lab 00: KNU Camera-Radar ADAS Course Environment Preflight

Reference Environment
---------------------
Ubuntu 22.04
Python 3.10
ROS2 Humble
CARLA 0.9.16

Execution Environments
----------------------
Host Python
    - Lab 00-13
    - RADIal
    - ROS2
    - CARLA

Conda environment: yolo
    - Lab 14
    - Ultralytics YOLO
    - PyTorch / CUDA
    - ROS2 vision_msgs

This program only checks the environment.
It does not install or modify packages.
"""

from __future__ import annotations

import importlib
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


# ============================================================
# Course Contract
# ============================================================

COURSE_ROOT = (
    Path.home()
    / "260818_0820_knu_course"
)

COURSE_DATA_ROOT = (
    COURSE_ROOT
    / "knu_course_data"
)

RADIAL_ROOT = (
    COURSE_DATA_ROOT
    / "RADIal_course"
)

CARLA_ROOT = (
    Path.home()
    / "carla"
)

EXPECTED_CARLA_VERSION = "0.9.16"

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000

MAIN_CLIP_START = 9015
MAIN_CLIP_END = 9142

EXPECTED_MAIN_CLIP_FRAMES = (
    MAIN_CLIP_END
    - MAIN_CLIP_START
    + 1
)

YOLO_ENV_NAME = "yolo"

YOLO_MODEL = (
    COURSE_DATA_ROOT
    / "models"
    / "yolo11n_carla.pt"
)


# ============================================================
# Result Counter
# ============================================================

PASS_COUNT = 0
FAIL_COUNT = 0
WARN_COUNT = 0


def passed(message: str) -> None:
    global PASS_COUNT
    PASS_COUNT += 1
    print(f"[PASS] {message}")


def failed(
    message: str,
    hint: str | None = None,
) -> None:
    global FAIL_COUNT
    FAIL_COUNT += 1

    print(f"[FAIL] {message}")

    if hint:
        print(f"       -> {hint}")


def warned(
    message: str,
    hint: str | None = None,
) -> None:
    global WARN_COUNT
    WARN_COUNT += 1

    print(f"[WARN] {message}")

    if hint:
        print(f"       -> {hint}")


def section(title: str) -> None:
    print()
    print("=" * 68)
    print(title)
    print("=" * 68)


# ============================================================
# Utility
# ============================================================

def package_version(module) -> str:
    return str(
        getattr(
            module,
            "__version__",
            "unknown",
        )
    )


def module_location(module) -> str:
    return str(
        getattr(
            module,
            "__file__",
            "unknown",
        )
    )


def sample_id_from_path(
    path: Path,
) -> int | None:
    """
    Course filenames:
        image_009015.jpg
        pcl_009015.npy
        fft_009015.npy
    """

    try:
        return int(
            path.stem.rsplit(
                "_",
                1,
            )[-1]
        )

    except (
        ValueError,
        IndexError,
    ):
        return None


def sample_ids_from_files(
    files,
) -> set[int]:

    ids: set[int] = set()

    for path in files:

        sample_id = (
            sample_id_from_path(
                path
            )
        )

        if sample_id is not None:
            ids.add(sample_id)

    return ids


def run_command(
    command: list[str],
    timeout: float = 15.0,
):

    try:
        return subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )

    except Exception:
        return None


def find_conda() -> str | None:

    conda = shutil.which(
        "conda"
    )

    if conda:
        return conda

    candidates = [
        (
            Path.home()
            / "anaconda3"
            / "bin"
            / "conda"
        ),
        (
            Path.home()
            / "miniconda3"
            / "bin"
            / "conda"
        ),
    ]

    for path in candidates:

        if path.is_file():
            return str(path)

    return None


# ============================================================
# 1. Basic Environment
# ============================================================

def check_basic_environment() -> None:

    section(
        "1. Basic Environment"
    )

    print(
        f"User        : {Path.home().name}"
    )

    print(
        f"Platform    : {platform.platform()}"
    )

    print(
        f"Python      : {sys.version.split()[0]}"
    )

    print(
        f"Executable  : {sys.executable}"
    )

    if sys.version_info[:2] == (3, 10):

        passed(
            "Python 3.10"
        )

    else:

        warned(
            (
                "Python version is "
                f"{sys.version_info.major}."
                f"{sys.version_info.minor}"
            ),
            (
                "Reference environment "
                "uses Python 3.10."
            ),
        )

    if COURSE_ROOT.is_dir():

        passed(
            f"Course root: {COURSE_ROOT}"
        )

    else:

        failed(
            f"Course root missing: {COURSE_ROOT}"
        )


# ============================================================
# 2. Course Data / RADIal
# ============================================================

def check_course_data() -> None:

    section(
        "2. Course Data / RADIal"
    )

    if not COURSE_DATA_ROOT.is_dir():

        failed(
            (
                "Course data root missing: "
                f"{COURSE_DATA_ROOT}"
            )
        )

        return

    passed(
        f"Course data root: {COURSE_DATA_ROOT}"
    )

    if not RADIAL_ROOT.is_dir():

        failed(
            (
                "RADIal_course missing: "
                f"{RADIAL_ROOT}"
            )
        )

        return

    passed(
        f"RADIal root: {RADIAL_ROOT}"
    )

    camera_dir = (
        RADIAL_ROOT
        / "camera"
    )

    pcl_dir = (
        RADIAL_ROOT
        / "radar_PCL"
    )

    fft_dir = (
        RADIAL_ROOT
        / "radar_FFT"
    )

    freespace_dir = (
        RADIAL_ROOT
        / "radar_Freespace"
    )

    camera_files = sorted(
        camera_dir.glob(
            "image_*.jpg"
        )
    )

    pcl_files = sorted(
        pcl_dir.glob(
            "pcl_*.npy"
        )
    )

    fft_files = sorted(
        fft_dir.glob(
            "fft_*.npy"
        )
    )

    freespace_files = sorted(
        freespace_dir.glob(
            "freespace_*.png"
        )
    )

    modalities = {
        "camera": (
            camera_dir,
            camera_files,
        ),
        "radar_PCL": (
            pcl_dir,
            pcl_files,
        ),
        "radar_FFT": (
            fft_dir,
            fft_files,
        ),
        "radar_Freespace": (
            freespace_dir,
            freespace_files,
        ),
    }

    for name, (
        directory,
        files,
    ) in modalities.items():

        if directory.is_dir():

            passed(
                f"{name}: {len(files)} files"
            )

        else:

            if name == "radar_Freespace":

                warned(
                    (
                        "Optional directory missing: "
                        f"{directory}"
                    )
                )

            else:

                failed(
                    (
                        f"{name} directory missing: "
                        f"{directory}"
                    )
                )

    # --------------------------------------------------------
    # Manifest
    # --------------------------------------------------------

    manifest_path = (
        RADIAL_ROOT
        / "subset_manifest.json"
    )

    if manifest_path.is_file():

        try:

            manifest = json.loads(
                manifest_path.read_text(
                    encoding="utf-8"
                )
            )

            passed(
                "RADIal subset_manifest.json"
            )

            if "sample_count" in manifest:

                print(
                    (
                        "       selected samples: "
                        f"{manifest['sample_count']}"
                    )
                )

        except Exception as error:

            failed(
                (
                    "subset_manifest.json "
                    f"cannot be read: {error}"
                )
            )

    else:

        failed(
            (
                "RADIal manifest missing: "
                f"{manifest_path}"
            )
        )

    # --------------------------------------------------------
    # Labels
    # --------------------------------------------------------

    labels_path = (
        RADIAL_ROOT
        / "labels.csv"
    )

    if labels_path.is_file():

        passed(
            "RADIal labels: labels.csv"
        )

    else:

        failed(
            (
                "RADIal labels missing: "
                f"{labels_path}"
            )
        )

    # --------------------------------------------------------
    # Main Clip 9015-9142
    # --------------------------------------------------------

    required_ids = set(
        range(
            MAIN_CLIP_START,
            MAIN_CLIP_END + 1,
        )
    )

    file_groups = {
        "camera": camera_files,
        "radar_PCL": pcl_files,
        "radar_FFT": fft_files,
    }

    clip_ok = True

    for name, files in file_groups.items():

        ids = (
            sample_ids_from_files(
                files
            )
        )

        available = (
            required_ids
            & ids
        )

        missing = sorted(
            required_ids
            - ids
        )

        if (
            len(available)
            == EXPECTED_MAIN_CLIP_FRAMES
        ):

            passed(
                (
                    f"{name} Main Clip "
                    f"{MAIN_CLIP_START}-"
                    f"{MAIN_CLIP_END}: "
                    f"{len(available)} frames"
                )
            )

        else:

            clip_ok = False

            failed(
                (
                    f"{name} Main Clip: "
                    f"{len(available)}/"
                    f"{EXPECTED_MAIN_CLIP_FRAMES}"
                ),
                (
                    "Missing sample IDs: "
                    f"{missing[:10]}"
                ),
            )

    if clip_ok:

        passed(
            (
                "Main Clip synchronized contract: "
                "camera + radar_PCL + radar_FFT "
                f"= {EXPECTED_MAIN_CLIP_FRAMES} frames"
            )
        )


# ============================================================
# 3. Radar-Camera Calibration
# ============================================================

def check_calibration() -> None:

    section(
        "3. Radar-Camera Calibration"
    )

    calibration_path = (
        RADIAL_ROOT
        / "calibration"
        / "radar_camera.json"
    )

    if not calibration_path.is_file():

        failed(
            (
                "Calibration missing: "
                f"{calibration_path}"
            )
        )

        return

    try:

        calibration = json.loads(
            calibration_path.read_text(
                encoding="utf-8"
            )
        )

    except Exception as error:

        failed(
            (
                "Calibration JSON cannot be read: "
                f"{error}"
            )
        )

        return

    required_keys = [
        "reference_image_width",
        "reference_image_height",
        "camera_matrix",
        "distortion_coefficients",
        "rotation_vector",
        "translation_vector",
    ]

    missing = [
        key
        for key
        in required_keys
        if key not in calibration
    ]

    if missing:

        failed(
            (
                "Calibration JSON missing keys: "
                f"{missing}"
            )
        )

    else:

        passed(
            (
                "Radar-Camera calibration: "
                f"{calibration_path}"
            )
        )


# ============================================================
# 4. Host Python / ROS2 Dependencies
# ============================================================

def check_python_dependencies() -> None:

    section(
        "4. Host Python / ROS2 Dependencies"
    )

    checks = [
        (
            "numpy",
            "NumPy",
        ),
        (
            "cv2",
            "OpenCV",
        ),
        (
            "matplotlib",
            "Matplotlib",
        ),
        (
            "rclpy",
            "rclpy",
        ),
        (
            "sensor_msgs_py",
            "sensor_msgs_py",
        ),
    ]

    for (
        import_name,
        display_name,
    ) in checks:

        try:

            module = (
                importlib.import_module(
                    import_name
                )
            )

            passed(
                (
                    f"{display_name}: "
                    f"{package_version(module)}"
                )
            )

            print(
                f"       {module_location(module)}"
            )

        except Exception as error:

            failed(
                (
                    f"{display_name} import: "
                    f"{error}"
                )
            )


# ============================================================
# 5. ROS2 Tools
# ============================================================

def ros_package_executable_exists(
    package: str,
    executable: str,
) -> bool:

    result = run_command(
        [
            "ros2",
            "pkg",
            "executables",
            package,
        ]
    )

    if (
        result is None
        or result.returncode != 0
    ):

        return False

    for line in result.stdout.splitlines():

        fields = (
            line.strip()
            .split()
        )

        if (
            len(fields) >= 2
            and fields[0] == package
            and fields[1] == executable
        ):

            return True

    return False


def ros_package_exists(
    package: str,
) -> bool:

    result = run_command(
        [
            "ros2",
            "pkg",
            "prefix",
            package,
        ]
    )

    return bool(
        result is not None
        and result.returncode == 0
    )


def check_ros2_tools() -> None:

    section(
        "5. ROS2 Humble Tools"
    )

    ros2_path = (
        shutil.which(
            "ros2"
        )
    )

    if ros2_path:

        passed(
            f"ROS2 CLI: {ros2_path}"
        )

    else:

        failed(
            "ROS2 CLI not found",
            (
                "source "
                "/opt/ros/humble/setup.bash"
            ),
        )

        return

    ros_distro = (
        os.environ.get(
            "ROS_DISTRO",
            "",
        )
    )

    if ros_distro == "humble":

        passed(
            "ROS_DISTRO=humble"
        )

    else:

        warned(
            (
                "ROS_DISTRO="
                f"{ros_distro or 'not set'}"
            ),
            (
                "source "
                "/opt/ros/humble/setup.bash"
            ),
        )

    if ros_package_executable_exists(
        "rviz2",
        "rviz2",
    ):

        passed(
            "RViz2"
        )

    else:

        failed(
            "RViz2 executable not found"
        )

    if ros_package_executable_exists(
        "rqt_image_view",
        "rqt_image_view",
    ):

        passed(
            "rqt_image_view"
        )

    else:

        failed(
            "rqt_image_view not found",
            (
                "sudo apt install "
                "ros-humble-rqt-image-view"
            ),
        )

    if ros_package_exists(
        "image_transport"
    ):

        passed(
            "image_transport"
        )

    else:

        failed(
            "image_transport missing"
        )

    if ros_package_exists(
        "compressed_image_transport"
    ):

        passed(
            "compressed_image_transport"
        )

    else:

        failed(
            "compressed_image_transport missing",
            (
                "sudo apt install "
                "ros-humble-image-transport-plugins"
            ),
        )

    if ros_package_exists(
        "vision_msgs"
    ):

        passed(
            "vision_msgs"
        )

    else:

        failed(
            "vision_msgs missing",
            (
                "sudo apt install "
                "ros-humble-vision-msgs"
            ),
        )


# ============================================================
# 6. CARLA
# ============================================================

def check_carla() -> None:

    section(
        "6. CARLA 0.9.16"
    )

    if CARLA_ROOT.is_dir():

        passed(
            f"CARLA runtime: {CARLA_ROOT}"
        )

    else:

        failed(
            (
                "CARLA runtime missing: "
                f"{CARLA_ROOT}"
            )
        )

    try:

        import carla

        passed(
            "CARLA Python API import"
        )

        print(
            (
                "       module: "
                f"{module_location(carla)}"
            )
        )

    except Exception as error:

        failed(
            (
                "CARLA Python API import failed: "
                f"{error}"
            ),
            (
                "python3 -m pip install "
                "--user --no-cache-dir "
                "--index-url https://pypi.org/simple "
                "carla==0.9.16"
            ),
        )

        return

    try:

        client = carla.Client(
            CARLA_HOST,
            CARLA_PORT,
        )

        client.set_timeout(
            2.0
        )

        client_version = str(
            client.get_client_version()
        )

        print(
            f"Client API : {client_version}"
        )

        if (
            client_version
            == EXPECTED_CARLA_VERSION
        ):

            passed(
                (
                    "CARLA client API "
                    f"{EXPECTED_CARLA_VERSION}"
                )
            )

        else:

            failed(
                (
                    "CARLA client API mismatch: "
                    f"{client_version}"
                )
            )

    except Exception as error:

        failed(
            (
                "CARLA client version check failed: "
                f"{error}"
            )
        )

        return

    # CARLA server is optional during Lab 00.
    try:

        server_version = str(
            client.get_server_version()
        )

        print(
            f"Server API : {server_version}"
        )

        if (
            server_version
            == EXPECTED_CARLA_VERSION
        ):

            passed(
                (
                    "CARLA server API "
                    f"{EXPECTED_CARLA_VERSION}"
                )
            )

        else:

            failed(
                (
                    "CARLA server API mismatch: "
                    f"{server_version}"
                )
            )

        if (
            client_version
            == server_version
        ):

            passed(
                "CARLA client/server versions match"
            )

        else:

            failed(
                (
                    "CARLA client/server mismatch: "
                    f"{client_version} != "
                    f"{server_version}"
                )
            )

    except Exception:

        warned(
            (
                "CARLA server is not running "
                f"at {CARLA_HOST}:"
                f"{CARLA_PORT}"
            ),
            (
                "This is OK for Lab 00. "
                "Start ~/carla/CarlaUE4.sh "
                "before CARLA labs."
            ),
        )


# ============================================================
# 7. YOLO Conda Environment
# ============================================================

def check_yolo_environment() -> None:

    section(
        "7. YOLO / Day 20 Environment"
    )

    conda = (
        find_conda()
    )

    if conda is None:

        failed(
            "Conda executable not found"
        )

        return

    passed(
        f"Conda: {conda}"
    )

    # --------------------------------------------------------
    # Environment existence
    # --------------------------------------------------------

    env_result = run_command(
        [
            conda,
            "env",
            "list",
            "--json",
        ]
    )

    if (
        env_result is None
        or env_result.returncode != 0
    ):

        failed(
            "Cannot query Conda environments"
        )

        return

    try:

        env_data = json.loads(
            env_result.stdout
        )

        env_paths = env_data.get(
            "envs",
            [],
        )

        yolo_env_path = next(
            (
                path
                for path
                in env_paths
                if Path(path).name
                == YOLO_ENV_NAME
            ),
            None,
        )

    except Exception:

        yolo_env_path = None

    if yolo_env_path is None:

        failed(
            (
                "Conda environment missing: "
                f"{YOLO_ENV_NAME}"
            )
        )

        return

    passed(
        (
            "Conda environment: "
            f"{YOLO_ENV_NAME}"
        )
    )

    print(
        f"       {yolo_env_path}"
    )

    # --------------------------------------------------------
    # Run dependency check INSIDE yolo environment
    # --------------------------------------------------------

    check_code = r'''
import json
import sys

result = {
    "python": sys.version.split()[0]
}

modules = [
    "numpy",
    "cv2",
    "torch",
    "torchvision",
    "ultralytics",
    "rclpy",
]

for name in modules:
    try:
        module = __import__(name)

        result[name] = {
            "ok": True,
            "version": str(
                getattr(
                    module,
                    "__version__",
                    "unknown"
                )
            ),
            "file": str(
                getattr(
                    module,
                    "__file__",
                    "unknown"
                )
            ),
        }

    except Exception as error:
        result[name] = {
            "ok": False,
            "error": str(error),
        }

try:
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    result["ros_messages"] = {
        "ok": True
    }

except Exception as error:

    result["ros_messages"] = {
        "ok": False,
        "error": str(error),
    }

try:
    import torch

    result["cuda"] = {
        "available": bool(
            torch.cuda.is_available()
        ),
        "version": str(
            torch.version.cuda
        ),
        "gpu": (
            torch.cuda.get_device_name(0)
            if torch.cuda.is_available()
            else None
        ),
    }

    if torch.cuda.is_available():

        x = torch.tensor(
            [1.0, 2.0],
            device="cuda",
        )

        y = x * 2.0

        torch.cuda.synchronize()

        result["cuda"]["operation"] = (
            y.device.type == "cuda"
        )

except Exception as error:

    result["cuda"] = {
        "available": False,
        "error": str(error),
    }

print(json.dumps(result))
'''

    result = run_command(
        [
            conda,
            "run",
            "--no-capture-output",
            "-n",
            YOLO_ENV_NAME,
            "python3",
            "-c",
            check_code,
        ],
        timeout=30.0,
    )

    if result is None:

        failed(
            "Cannot execute yolo environment"
        )

        return

    if result.returncode != 0:

        failed(
            "yolo environment dependency check failed"
        )

        if result.stderr.strip():
            print(
                result.stderr.strip()
            )

        return

    try:

        # Ultralytics may print a message before JSON.
        json_line = next(
            line
            for line
            in reversed(
                result.stdout.splitlines()
            )
            if line.strip().startswith("{")
        )

        data = json.loads(
            json_line
        )

    except Exception as error:

        failed(
            (
                "Cannot parse yolo environment "
                f"result: {error}"
            )
        )

        print(
            result.stdout
        )

        return

    # --------------------------------------------------------
    # Python
    # --------------------------------------------------------

    python_version = data.get(
        "python",
        "unknown",
    )

    if python_version.startswith(
        "3.10."
    ):

        passed(
            (
                "yolo Python: "
                f"{python_version}"
            )
        )

    else:

        warned(
            (
                "yolo Python: "
                f"{python_version}"
            ),
            (
                "Reference environment "
                "uses Python 3.10."
            ),
        )

    # --------------------------------------------------------
    # Python packages
    # --------------------------------------------------------

    display_names = {
        "numpy": "NumPy",
        "cv2": "OpenCV",
        "torch": "PyTorch",
        "torchvision": "TorchVision",
        "ultralytics": "Ultralytics",
        "rclpy": "rclpy",
    }

    for key, display_name in (
        display_names.items()
    ):

        item = data.get(
            key,
            {},
        )

        if item.get(
            "ok",
            False,
        ):

            passed(
                (
                    f"yolo {display_name}: "
                    f"{item.get('version', 'unknown')}"
                )
            )

            print(
                (
                    "       "
                    f"{item.get('file', 'unknown')}"
                )
            )

        else:

            failed(
                (
                    f"yolo {display_name}: "
                    f"{item.get('error', 'import failed')}"
                )
            )

    # --------------------------------------------------------
    # ROS2 messages inside Conda
    # --------------------------------------------------------

    ros_messages = data.get(
        "ros_messages",
        {},
    )

    if ros_messages.get(
        "ok",
        False,
    ):

        passed(
            "yolo ROS2 sensor_msgs / vision_msgs"
        )

    else:

        failed(
            (
                "yolo ROS2 messages: "
                f"{ros_messages.get('error')}"
            ),
            (
                "Run Lab 14 terminal with:\n"
                "          source /opt/ros/humble/setup.bash\n"
                "          conda activate yolo"
            ),
        )

    # --------------------------------------------------------
    # CUDA / GPU
    # --------------------------------------------------------

    cuda = data.get(
        "cuda",
        {},
    )

    if cuda.get(
        "available",
        False,
    ):

        passed(
            (
                "yolo CUDA available: "
                f"{cuda.get('version')}"
            )
        )

        passed(
            (
                "yolo GPU: "
                f"{cuda.get('gpu')}"
            )
        )

        if cuda.get(
            "operation",
            False,
        ):

            passed(
                "yolo CUDA tensor operation"
            )

        else:

            failed(
                "yolo CUDA tensor operation failed"
            )

    else:

        warned(
            "yolo CUDA unavailable",
            (
                "YOLO can run on CPU, "
                "but GPU is recommended."
            ),
        )

    # --------------------------------------------------------
    # Course YOLO weight
    # --------------------------------------------------------

    if YOLO_MODEL.is_file():

        size_mb = (
            YOLO_MODEL.stat().st_size
            / 1024.0
            / 1024.0
        )

        passed(
            (
                "Course YOLO model: "
                f"{YOLO_MODEL.name} "
                f"({size_mb:.1f} MB)"
            )
        )

    else:

        failed(
            (
                "Course YOLO model missing: "
                f"{YOLO_MODEL}"
            )
        )

    # --------------------------------------------------------
    # Legacy weight must not remain
    # --------------------------------------------------------

    legacy_model = (
        COURSE_DATA_ROOT
        / "models"
        / "carla_yolov8n.pt"
    )

    if legacy_model.exists():

        warned(
            (
                "Legacy YOLO model still exists: "
                f"{legacy_model.name}"
            ),
            (
                "Remove it to avoid model confusion."
            ),
        )

    else:

        passed(
            "Legacy carla_yolov8n.pt removed"
        )


# ============================================================
# 8. Course Interface Contract
# ============================================================

def check_course_contract() -> None:

    section(
        "8. Course Interface Contract"
    )

    contract = [
        (
            "CARLA host",
            "127.0.0.1",
        ),
        (
            "CARLA port",
            "2000",
        ),
        (
            "Ego role_name",
            "knu_hero",
        ),
        (
            "Lead role_name",
            "knu_lead",
        ),
        (
            "Vehicle frame",
            "vehicle",
        ),
        (
            "Vehicle axis",
            "x=forward, y=left, z=up",
        ),
        (
            "Global frame",
            "map",
        ),
        (
            "Camera topic",
            "/carla/hero/camera_front/image",
        ),
        (
            "Radar topic",
            "/carla/hero/radar/point_cloud",
        ),
        (
            "Detection topic",
            "/carla/object_detection_2d/bounding_box",
        ),
        (
            "Fused target",
            "/adas/fused_target",
        ),
        (
            "ADAS decision",
            "/adas/shadow_command",
        ),
        (
            "Lane result",
            "/carla/lane/center",
        ),
        (
            "Control status",
            "/vehicle/control/status",
        ),
        (
            "YOLO env",
            "yolo",
        ),
        (
            "YOLO model",
            "yolo11n_carla.pt",
        ),
    ]

    for key, value in contract:

        print(
            f"{key:17s}: {value}"
        )

    passed(
        "Course interface contract loaded"
    )


# ============================================================
# Summary
# ============================================================

def print_summary() -> None:

    section(
        "Environment Summary"
    )

    print(
        f"PASS : {PASS_COUNT}"
    )

    print(
        f"WARN : {WARN_COUNT}"
    )

    print(
        f"FAIL : {FAIL_COUNT}"
    )

    print()

    if FAIL_COUNT == 0:

        print(
            "[READY] Course environment is ready."
        )

        if WARN_COUNT > 0:

            print(
                (
                    "        WARN items are "
                    "informational or optional."
                )
            )

    else:

        print(
            (
                "[NOT READY] Fix FAIL items "
                "before the course."
            )
        )


# ============================================================
# Main
# ============================================================

def main() -> int:

    print()

    print(
        "KNU Camera-Radar ADAS Course"
    )

    print(
        "Environment Preflight"
    )

    print(
        (
            "Reference: Ubuntu 22.04 / "
            "ROS2 Humble / CARLA 0.9.16"
        )
    )

    check_basic_environment()

    check_course_data()

    check_calibration()

    check_python_dependencies()

    check_ros2_tools()

    check_carla()

    check_yolo_environment()

    check_course_contract()

    print_summary()

    return (
        0
        if FAIL_COUNT == 0
        else 1
    )


if __name__ == "__main__":

    raise SystemExit(
        main()
    )
