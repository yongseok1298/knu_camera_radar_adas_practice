# KNU Camera–Radar ADAS 실습

이 폴더는 `00_...py`부터 번호 순서대로 진행한다. GitHub 예제와 같은 **독립 실행 Python 파일** 형식이며, 별도의 ROS2 패키지 빌드는 필요 없다.

## 전체 실습 흐름

| 순서 | 파일 | ECU/시스템 관점의 학습 결과 |
|---:|---|---|
| 00 | `00_check_environment.py` | 실행 환경과 배포물 수락 검사 |
| 01 | `01_inspect_radial_subset.py` | 센서 데이터 계약과 좌표계 확인 |
| 02 | `02_ros2_topic_basics.py` | ROS2 publisher–subscriber 계약 |
| 03 | `03_radial_pointcloud_replay.py` | Radar ECU의 PointCloud2 출력 |
| 04 | `04_radial_camera_radar_fusion.py` | RADIal 기반 camera–radar late fusion |
| 05 | `05_radial_fft_processing.py` | 복소 FFT 텐서의 magnitude 처리 |
| 06 | `06_check_carla_rpc.py` | Simulator ECU 연결 수락 검사 |
| 07 | `07_carla_camera_radar_bridge.py` | CARLA Sensor ECU와 ROS2 인터페이스 |
| 08 | `08_carla_lead_vehicle_scenario.py` | 검증 가능한 선행차 시나리오 설정 |
| 09 | `09_camera_radar_fusion.py` | 시간 동기화·카메라 상태·radar MIO 융합 |
| 10 | `10_adas_decision_shadow.py` | ACC/AEB 판단과 안전한 shadow output |
| 11 | `11_collect_system_evidence.py` | 요구사항 기반 Pass/Fail/Not Run 증거 |
| 12 | `12_carla_lane_detection.py` | RGB 차선 중심·요각 추정 |
| 13 | `13_path_tracking_control.py` | Pure Pursuit 차선 추종과 ADAS 종방향 판단 통합 |

## 0. 학생 PC 준비

기준 환경은 **Native Ubuntu 22.04 + ROS2 Humble + Python 3.10 + CARLA 0.9.16**이다. WSL에서는 00~05의 정적/데이터 실습은 가능하지만 CARLA GPU, RPC, 실제 센서 timing 검증을 대신하지 않는다.

USB에서 다음 두 폴더를 홈 디렉터리로 복사한다.

```bash
cp -a /media/$USER/USB_NAME/knu_course_data ~/260818_0820_knu_course/knu_course_data
cp -a /media/$USER/USB_NAME/src ~/260818_0820_knu_course/knu_course_src
cd ~/260818_0820_knu_course/knu_course_src
chmod +x [0-9][0-9]_*.py
```

데이터를 USB에서 직접 읽을 경우에만 다음 값을 지정한다.

```bash
export KNU_COURSE_DATA=/media/$USER/USB_NAME/knu_course_data
```

### ROS2 Humble 설치

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-numpy python3-matplotlib python3-opencv python3-pip
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

모든 새 터미널에서 ROS2를 source한 뒤 확인한다.

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 00_check_environment.py
```

예상 결과: CARLA Python API만 `[NOT RUN]`, 나머지는 `[PASS]`. 하나라도 `[FAIL]`이면 다음 실습으로 넘어가지 않는다.

## 1. RADIal 실습: 01~05

### 01 데이터와 좌표 계약

```bash
python3 01_inspect_radial_subset.py
```

확인할 값:

- 총 826 samples, split은 train 623 / validation 99 / test 104
- `radar_PCL`의 실습 출력 열: `x_forward, y_left, z_up, relative_speed`
- RADIal 원본의 right-positive 횡좌표를 ROS 차량 좌표의 left-positive로 변환

### 02 ROS2 토픽

터미널 A:

```bash
python3 02_ros2_topic_basics.py --mode pub
```

터미널 B:

```bash
python3 02_ros2_topic_basics.py --mode sub
```

예상 결과: B에서 `/vehicle/speed_mps` 값이 0.5 m/s 간격으로 수신된다.

### 03 Radar ECU replay

```bash
python3 03_radial_pointcloud_replay.py --hz 5
```

다른 터미널:

```bash
ros2 topic hz /radial/radar/points
ros2 topic echo /radial/radar/points --once
```

예상 결과: 약 5 Hz, `frame_id: vehicle`, `relative_speed_mps` 필드가 관찰된다. `--hz`는 강의용 합성 replay 속도이며 실제 RADIal 측정 timestamp가 아니다.

### 04 RADIal camera–radar fusion + 10 shadow 판단

터미널 A:

```bash
python3 04_radial_camera_radar_fusion.py
```

터미널 B:

```bash
python3 10_adas_decision_shadow.py
```

터미널 C:

```bash
python3 11_collect_system_evidence.py --source radial --duration 10
```

예상 결과: camera, radar, status, shadow가 모두 `[PASS]`. RADIal `labels.csv`의 bbox는 **사전 녹화된 카메라 검출기 proxy**이며 CARLA runtime ground truth로 사용하지 않는다. sample ID 결합이므로 `sync_delta_s=0.0`은 측정된 시간차가 아니다.

### 05 Radar FFT magnitude

```bash
python3 05_radial_fft_processing.py
```

예상 결과: 복소 텐서 shape/dtype과 magnitude 통계가 출력되고 `radial_fft_preview.png`가 생성된다. 캘리브레이션 메타데이터 없이 그림의 축을 실제 range/Doppler 값으로 부르지 않는다.

## 2. CARLA 준비와 실습: 06~11

### CARLA 압축 해제와 Python API

```bash
mkdir -p ~/carla
tar -xzf ~/260818_0820_knu_course/knu_course_data/carla/carla.tar.gz -C ~/carla
python3 -m pip install --user ~/carla/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl
```

터미널 A에서 Native CARLA를 실행한다.

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low
```

터미널 B:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 06_check_carla_rpc.py
```

`server version: 0.9.16`이 `[PASS]`여야 한다.

### 통합 시스템 실행

터미널 B — Sensor ECU:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 07_carla_camera_radar_bridge.py
```

터미널 C — 정지 선행차를 약 20 m 전방에 1회 배치:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 08_carla_lead_vehicle_scenario.py --distance 20
```

터미널 D — Fusion ECU:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 09_camera_radar_fusion.py
```

터미널 E — Decision ECU, actuator와 분리된 shadow mode:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 10_adas_decision_shadow.py
```

터미널 F — 검증 증거:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 11_collect_system_evidence.py --source carla --duration 10
```

정상 수락 기준:

- camera, radar, status, shadow: `[PASS]`
- 선행차가 radar FOV 안에 있으면 target: `[PASS]`
- `/adas/fusion/status` mode: 주로 `fused`
- `/adas/shadow_command`에 `shadow_mode: true`
- 어떤 스크립트도 CARLA throttle/brake actuator를 구독하거나 호출하지 않음

선행차가 `[NOT RUN]`이면 CARLA 카메라 화면에서 선행차 위치를 확인하고 `--distance 15` 또는 `--distance 25`로 다시 배치한다. 시스템 종료 후 CARLA map을 reload하면 강의 actor가 정리된다.

## 3. 고장 주입과 degraded mode

정상 Fusion ECU를 종료한 뒤 다음과 같이 다시 실행한다.

```bash
python3 09_camera_radar_fusion.py --force-camera-unavailable
ros2 topic echo /adas/fusion/status
```

예상 결과:

- radar target이 있으면 `radar_only_camera_unavailable`
- target이 없으면 `invalid_no_target`
- shadow node는 계속 실행되지만 actuator에는 연결되지 않음
- target이 0.5초 이상 끊기면 `FALLBACK_HOLD`

## ROS2 인터페이스 계약

| Topic | Type | 계약 |
|---|---|---|
| `/carla/hero/camera_front/image` | `sensor_msgs/Image` | BGRA8, camera optical frame |
| `/carla/hero/radar/point_cloud` | `sensor_msgs/PointCloud2` | x-forward, y-left, z-up, approaching velocity negative |
| `/adas/fused_target` | `geometry_msgs/Vector3Stamped` | x=range m, y=y-left m, z=relative speed m/s |
| `/adas/fusion/status` | `std_msgs/String` JSON | valid, mode, camera confidence, sync delta |
| `/adas/shadow_accel` | `geometry_msgs/AccelStamped` | linear.x=요구 종가속도, actuator 미연결 |
| `/adas/shadow_command` | `std_msgs/String` JSON | ACC/AEB mode, brake, TTC, `shadow_mode=true` |
| `/carla/lane/center` | `geometry_msgs/PointStamped` | x=차선 중심 y-left m, y=heading error rad, z=confidence |
| `/carla/lane/debug_image` | `sensor_msgs/Image` | 차선 검출 결과 BGR 이미지 |
| `/carla/hero/cmd_vel` | `geometry_msgs/Twist` | linear.x=목표속도 m/s, angular.z=조향각 rad |
| `/vehicle/control/status` | `std_msgs/String` JSON | 통합 제어 mode와 actuator 연결 여부 |

교육용 단일 파일 형식을 유지하기 위해 status와 상세 shadow command는 JSON 문자열을 사용한다. 양산 프로젝트에서는 버전이 명시된 custom message/IDL과 인터페이스 변경 관리를 적용해야 한다.

## 3일 권장 운영 순서

- 1일차: 00~05 — ROS2와 RADIal 데이터 계약, PCL, camera–radar association, FFT
- 2일차: 06~09 — CARLA sensor bridge, 시나리오, 동기화와 degraded fusion
- 3일차: 10~13 — ACC/AEB 판단, 차선 추종, 통합 제어, 고장 주입과 검증 보고

## 4. 자율주행 차선 추종 통합: 12~13

먼저 `07`, `09`, `10`을 각각 실행해 Camera/Radar와 ADAS 판단 topic을 준비한다.

터미널 G — RGB 차선 인식:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 12_carla_lane_detection.py
```

터미널 H — 기본 shadow mode 차선 추종:

```bash
cd ~/260818_0820_knu_course/knu_course_src
python3 13_path_tracking_control.py
ros2 topic echo /vehicle/control/status
```

정상 결과는 `LANE_TRACK+CRUISE`, `LANE_TRACK+ACC_*` 또는 `LANE_TRACK+AEB`이다. `actuator_connected`는 `false`이며 CARLA 차량은 움직이지 않는다. RViz2의 Image display에 `/carla/lane/debug_image`를 추가하면 검출 선을 확인할 수 있다.

모든 topic과 정지 선행차 위치를 확인한 뒤에만 실제 CARLA 제어를 실행한다.

```bash
python3 13_path_tracking_control.py --apply-control
```

실제 제어 동작:

- 차선 중심과 heading error로 Pure Pursuit 조향각 계산
- Camera–Radar 판단이 `ACC_BRAKE`이면 목표속도 감소
- `AEB`이면 throttle 0, brake 1.0
- 차선 검출이 0.5초 이상 끊기면 `LANE_FALLBACK`과 brake 0.4
- `Ctrl+C` 종료 시 brake 1.0

이 실습은 교육용 고전 영상처리이므로 역광·그림자·교차로에서 차선을 놓칠 수 있다. 이 실패 자체를 camera 한계와 degraded-mode 요구사항의 검증 사례로 기록한다.
