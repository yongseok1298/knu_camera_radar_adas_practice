# KNU Camera–Radar ADAS Practice

경북대학교 ADAS 실습용 학생 배포 저장소입니다.

기준 환경은 **Ubuntu 22.04 + ROS2 Humble + Python 3.10 + CARLA 0.9.16**이며, CARLA 기본 RPC는 `127.0.0.1:2000`을 사용합니다.

## 공통 인터페이스

- CARLA ego actor `role_name`: `knu_hero`
- lead actor `role_name`: `knu_lead`
- ego body frame: `vehicle` (`x` forward, `y` left, `z` up)
- front camera frame: `camera_front_optical`
- global route frame: `map`
- camera: `/carla/hero/camera_front/image`
- radar: `/carla/hero/radar/point_cloud`
- fused target: `/adas/fused_target`
- fusion status: `/adas/fusion/status`
- ADAS decision: `/adas/shadow_command`
- lane result: `/carla/lane/center`
- control status: `/vehicle/control/status`

## 폴더

`knu_course_src_for_student/`에 학생용 실습 코드를 번호 순서대로 제공합니다. 각 파일의 `## TODO`를 검색해 실습합니다.

- 00–13: ROS2, RADIal, CARLA Camera/Radar, fusion, ACC/AEB, lane detection, path tracking
- 14–17: 8/20 live camera–radar perception pipeline
- 18–19: CARLA waypoint map 및 global path planning
- 20: Camera–Radar–GNSS 기반 통합 ADAS driving

대용량 데이터셋, CARLA runtime 및 학습 weight는 Git 저장소에 포함하지 않고 강의용 `knu_course_data`로 별도 배포합니다.
