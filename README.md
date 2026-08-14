# 대구RISE 혁신아카데미 모빌리티 특화교육 — Module 3 실습

본 저장소는 다음 두 과목의 실습 자료를 제공합니다.

- **ADAS System Architecture**
- **센서 데이터 처리 및 융합 (Radar/Camera)**

실습의 목적은 단순한 예제 실행이 아니라, 교육생이 직접 **센서 입력 → 데이터 처리/융합 → 판단 → 차량 제어 → 결과 검증**의 연결 구조를 구현하고 확인하는 것입니다.

본 실습에서 생성한 인지·융합 결과와 시스템 인터페이스는 이후 **MATLAB/Simulink 기반 모델 설계 및 ADAS 제어 시스템 설계** 과정으로 이어지는 입력 기반으로 활용할 수 있도록 구성되어 있습니다.

> 19~20의 PID/Pure Pursuit는 폐루프 동작을 확인하기 위한 교육용 기준 제어기입니다. 이후 과정의 모델 기반 제어 설계를 대체하는 것이 목적이 아닙니다.

---

## 1. 실습 환경

기준 환경은 다음과 같습니다.

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- CARLA 0.9.16
- OpenCV / NumPy / Matplotlib
- Ultralytics YOLO는 별도 Conda 환경 `yolo`에서 사용

학생용 소스 경로:

```bash
~/260818_0820_knu_course/knu_course_src_for_student
```

실습 데이터 경로:

```bash
~/260818_0820_knu_course/knu_course_data
```

CARLA 실행 경로:

```bash
~/carla
```

YOLO 실습(14)은 별도 환경을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
conda activate yolo
cd ~/260818_0820_knu_course/knu_course_src_for_student
```

---

## 2. 실습 진행 방식

각 파일은 독립 실행형 Python 프로그램입니다. 별도의 ROS2 패키지 빌드는 필요하지 않습니다.

학생용 파일에는 일부 핵심 부분이 `## TODO`로 비어 있습니다.

```bash
grep -n "TODO" 19_path_tracking_pid_control.py
```

실습은 다음 순서로 진행합니다.

1. 입력 Topic과 좌표계 확인
2. 핵심 알고리즘 또는 파라미터 구현
3. ROS2 Topic으로 출력 확인
4. RViz2 또는 `rqt_image_view`에서 결과 확인
5. 파라미터를 변경해 결과 비교
6. 실패 조건 또는 degraded mode 확인

---

## 3. 전체 실습 흐름

### 8월 18일 — ADAS 시스템 구성 및 기본 검증

| 순서 | 파일 | 주요 내용 |
|---:|---|---|
| 00 | `00_check_environment.py` | 실습 환경 및 배포 파일 점검 |
| 01 | `01_inspect_radial_subset.py` | RADIal 데이터 구조, Radar PCL, 좌표계 확인 |
| 02 | `02_ros2_topic_basics.py` | ROS2 Publisher/Subscriber와 Topic 인터페이스 |
| 03 | `03_radial_pointcloud_replay.py` | Radar PointCloud2 재생 및 시각화 |
| 04A | `04a_radial_radar_camera_projection.py` | RADIal Radar point를 Camera image plane으로 투영 |
| 04B | `04b_radial_camera_radar_fusion.py` | Camera BBox와 Radar point 연관, MIO 후보 생성 |
| 05 | `05_radial_fft_processing.py` | Radar FFT complex tensor의 magnitude 처리 |
| 06 | `06_check_carla_rpc.py` | CARLA 0.9.16 RPC 연결 확인 |
| 07 | `07_carla_camera_radar_bridge.py` | CARLA Camera/Radar 센서 구성과 ROS2 Topic 출력 |
| 08 | `08_carla_lead_vehicle_scenario.py` | 선행 차량 검증 시나리오 구성 |
| 09 | `09_camera_radar_fusion.py` | 실시간 센서 상태·동기화 기반 융합 상태 확인 |
| 10 | `10_adas_decision_shadow.py` | TTC 기반 ACC/AEB 판단, Shadow output |
| 11 | `11_collect_system_evidence.py` | 실행 결과와 Pass/Fail/Not Run 증거 수집 |
| 12A | `12a_carla_lane_detection_canny.py` | Canny/Hough 기반 직선 차선 검출 |
| 12B | `12b_carla_lane_detection_polyfit.py` | BEV/Sliding Window/Polynomial 기반 곡선 차선 검출 |
| 13 | `13_path_tracking_control.py` | 차선 정보 기반 Pure Pursuit 주행 제어 확인 |

### 8월 20일 — Camera–Radar 융합 및 통합 ADAS 주행

| 순서 | 파일 | 주요 내용 |
|---:|---|---|
| 14 | `14_carla_yolo_detection.py` | CARLA 전방 Camera 기반 YOLO 객체 검출 |
| 15 | `15_carla_radar_camera_projection.py` | 실시간 Radar point의 Camera image plane 투영 |
| 16 | `16_carla_bbox_radar_association.py` | YOLO BBox–Radar point association |
| 17 | `17_fused_target_tracking.py` | 융합 객체 Tracking, MIO 선정, Range-rate 추정 |
| 18 | `18_goal_pose_global_path.py` | RViz Goal Pose와 CARLA/OpenDRIVE 기반 Global Path 생성 |
| 19 | `19_path_tracking_pid_control.py` | GNSS 기반 경로 추종, Pure Pursuit + PID 제어 |
| 20 | `20_camera_radar_gnss_adas_driving.py` | Camera–Radar–GNSS 기반 통합 ADAS 주행 |

실습의 최종 데이터 흐름은 다음과 같습니다.

```text
Camera ──> YOLO ───────────────┐
                               ├─> Association ─> Tracking/MIO ─> ACC/AEB
Radar ──> Projection ──────────┘                              │
                                                             ├─> PID ─> Throttle/Brake
Goal Pose ─> Global Path ─> GNSS ─> Pure Pursuit ────────────┘
                          │
Camera Lane Detection ────┴─> Local correction
```

---

## 4. 환경 점검

CARLA 실행 전:

```bash
python3 00_check_environment.py
```

CARLA 실행:

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low
```

CARLA 연결 확인:

```bash
cd ~/260818_0820_knu_course/knu_course_src_for_student
python3 06_check_carla_rpc.py
```

수업 후반 CARLA 실습은 **Town04**를 기준으로 진행합니다.

---

## 5. 주요 ROS2 인터페이스

| Topic | Type | 의미 |
|---|---|---|
| `/carla/hero/camera_front/image` | `sensor_msgs/Image` | 전방 Camera image |
| `/carla/hero/radar/point_cloud` | `sensor_msgs/PointCloud2` | vehicle frame Radar point cloud |
| `/carla/object_detection_2d/bounding_box` | `vision_msgs/Detection2DArray` | YOLO 2D detection 결과 |
| `/carla/radar/projected_points` | `sensor_msgs/PointCloud2` | Camera plane으로 투영된 Radar point |
| `/adas/fused_candidates` | `sensor_msgs/PointCloud2` | BBox–Radar association 결과 |
| `/adas/fused_target` | `geometry_msgs/Vector3Stamped` | MIO: x=forward range, y=y-left, z=tracked range-rate |
| `/adas/shadow_command` | `std_msgs/String` JSON | CRUISE/ACC/AEB 판단 결과 |
| `/carla/lane/center` | `geometry_msgs/PointStamped` | 차선 offset, heading error, confidence |
| `/carla/hero/gnss` | `sensor_msgs/NavSatFix` | Ego GNSS |
| `/carla/path/global` | `nav_msgs/Path` | Goal까지의 Global Path (`map` frame) |
| `/carla/path/tracking_target` | `geometry_msgs/PoseStamped` | Pure Pursuit lookahead target |
| `/vehicle/control/status` | `std_msgs/String` JSON | 통합 제어 상태 및 제어 명령 |

공통 좌표계:

```text
vehicle frame
x : forward
 y : left
 z : up

map frame
Global Path / Goal Pose 기준
```

`/adas/fused_target.vector.z`는 실습 17에서 동일 Track의 거리 변화를 이용해 추정한 상대속도입니다.

```text
z < 0 : Closing
z = 0 : Relative distance 유지
z > 0 : Separating
```

CARLA Radar의 순간 Doppler 값은 비교·진단용으로 확인할 수 있지만, 최종 ACC/AEB 제어 입력에는 Tracking 기반 Range-rate를 사용합니다.

---

## 6. 8월 20일 통합 실행 순서

통합 실습은 한 번에 모든 프로그램을 실행하지 않고 단계적으로 확인합니다.

### 1단계 — Sensor / Lane

```bash
python3 07_carla_camera_radar_bridge.py
python3 12b_carla_lane_detection_polyfit.py
```

### 2단계 — Camera–Radar 인지 및 융합

YOLO 터미널:

```bash
source /opt/ros/humble/setup.bash
conda activate yolo
python3 14_carla_yolo_detection.py
```

일반 ROS2 터미널:

```bash
python3 15_carla_radar_camera_projection.py
python3 16_carla_bbox_radar_association.py
python3 17_fused_target_tracking.py
python3 10_adas_decision_shadow.py
```

### 3단계 — Goal / Global Path

```bash
python3 18_goal_pose_global_path.py
```

RViz2에서 Goal Pose를 지정하고 `/carla/path/global`을 확인합니다.

### 4단계 — 통합 주행

먼저 제어 명령만 계산합니다.

```bash
python3 20_camera_radar_gnss_adas_driving.py
```

입력 Topic과 상태를 확인한 뒤 실제 CARLA 제어를 실행합니다.

```bash
python3 20_camera_radar_gnss_adas_driving.py --apply-control
```

> `13`, `19`, `20`은 동일 Ego 차량을 제어할 수 있으므로 실제 제어 모드에서는 동시에 실행하지 않습니다.

---

## 7. 확인해야 할 결과

### Camera–Radar

- Camera object BBox가 생성되는가?
- Radar point가 image plane으로 투영되는가?
- BBox 내부 Radar point가 해당 객체와 연관되는가?
- Track ID가 시간축에서 유지되는가?
- MIO의 range와 tracked range-rate가 안정적으로 출력되는가?

### ADAS 판단

- 선행 객체가 없으면 `CRUISE`
- 안전거리 감소 시 `ACC_HOLD` 또는 `ACC_BRAKE`
- TTC 임계조건에 도달하면 `AEB`
- 입력이 유효하지 않거나 오래되면 degraded/fallback 상태로 전환

### 경로 추종

- RViz Goal Pose로 Global Path가 생성되는가?
- GNSS 위치와 Global Path가 `map` frame에서 일치하는가?
- Pure Pursuit lookahead target이 차량 진행에 따라 이동하는가?
- PID가 목표속도를 추종하는가?
- Goal 접근 시 감속하고 Goal에서 정지하는가?

---

## 8. 실패 사례도 실습 결과입니다

본 실습은 정상 동작만 확인하는 것이 목적이 아닙니다.

다음과 같은 결과도 시스템 검증의 일부로 기록합니다.

- 직선 모델이 곡선 차선을 놓치는 경우
- 교차로/분기에서 차선 confidence가 낮아지는 경우
- Camera detection 또는 Radar association이 일시적으로 끊기는 경우
- Simulator Radar의 순간 Doppler 값이 불안정한 경우
- 센서 데이터가 stale 상태가 되어 fallback으로 전환되는 경우

각 실패 사례에서 **어떤 입력이 유효하지 않았는지, 시스템이 어떤 상태로 전환되는지, 제어가 어떻게 제한되는지**를 확인합니다.

---

## 9. 이후 MATLAB/Simulink 과정과의 연결

본 실습에서 확인하는 핵심은 다음 인터페이스입니다.

```text
Perception / Fusion
        ↓
MIO range / relative speed
        ↓
ADAS Decision
        ↓
Target speed / brake request
        ↓
Vehicle Control
```

19~20에서 사용하는 PID/Pure Pursuit는 위 인터페이스를 실제 CARLA 폐루프에서 검증하기 위한 기준 제어기입니다.

이후 MATLAB/Simulink 과정에서는 동일한 **입력–판단–제어 관계**를 모델 기반 설계 관점에서 확장하여 학습할 수 있습니다.
