# ERP42 무조작 후진 보정 적용 및 A/B 검증 (2026-11-21)

## 요약

Gazebo Classic 11 / ROS 2 Humble / WSL Ubuntu-22.04 의 ERP42가 평지에서 명령 없이 후방으로 서서히 이동하는 현상을 재현했다. 기본 모델은 `/wheel_effort/commands` 발행자가 0개이고 측정 구동 `effort=0`인 상태에서도 후진했다.

- **기준 후진**: 약 -0.411 mm/s (약 12초에 -4.90 mm)
- **해결 적용**: effort 모드 기본 활성화 `erp42_control.idle_hold` 노드. ROS 2 `/odom`의 차체 위치/전방 속도를 이용하여 **작은 4바퀴 토크**로 정지 위치를 유지한다.
- **정지 유지 시험**: 별도 노드 시험에서 약 10초간 측정 위치 범위 16 µm, 실제 패키지 재빌드 후 기본 실행에서 약 10초간 관찰된 차체 X 차이는 약 23 µm. 보정이 수렴하면 로그상 속도도 거의 0에 가까웠다.
- **다른 주행 노드 우선**: 외부 `/wheel_effort/commands` 발행자가 탐지되면 보정은 명령을 멈춘다. 외부 발행자가 제거되면 현재 위치를 새 정지점으로 잡아 감속하고 정지시킨다.
- **범위**: effort 모드에만 적용. `drive_mode:=skid`는 그대로 사용하며 정지 보정 노드를 실행하지 않는다.
- **물리 원본 보존**: URDF/Xacro의 질량, 관성, 마찰, 스폰 높이는 **변경하지 않았다**. 효과를 보지 못한 수치 튜닝을 생산 설정에 반영하지 않았다.

## 반복 측정: 효과 없는 접근 분리

동일한 WSL + Gazebo를 각 시험마다 새로 시작하고 차체 x, pitch를 측정했다. 관찰 시간은 약 10~12초, 경과 시간은 wall-time 기준이다.

| 구성 | 후진 평균 속도(mm/s) | 판단 |
| --- | ---: | --- |
| 원본 baseline | -0.4111 | 무조작 후진 재현 |
| 조인트 damping=2 | -0.4159 | 개선 없음 |
| 조인트 damping=10 | -0.4172 | 개선 없음 |
| 조인트 friction=0.2 | -0.4115 | 개선 없음 |
| 스폰 높이 0.32→0.28 m | -0.4085 | 개선 없음 |
| damping=2, friction=0.2, spawn=0.28 m | -0.3996 | 미미한 차이 |
| 조향·구동 컨트롤러 비활성화 | -0.4175 | 컨트롤러 구동 명령이 원인 아님 |
| **정지 보정 컨트롤러** | **약 0 (µm 수준 변동)** | **정지 유지 확인** |

경미한 드리프트의 일차적인 원인은 Gazebo 접촉 수치해석·모델 충돌 기하에서 발생하는 미세 잔류 운동으로 추정되지만, 어떤 접촉·solver 항목이 초기 후방 힘을 발생시켰는지는 아직 확정하지 않았다. 추가적인 복잡한 접촉 메시 A/B는 수행하지 않았다.

## 실행 및 설정

Windows PowerShell에서 GUI를 열려면:

```powershell
wsl -d Ubuntu-22.04 -u root -- bash /root/erp42_ws/start_gazebo_gui.sh --rviz
```

ROS 2에서 직접 실행할 때:

```bash
source /opt/ros/humble/setup.bash
source /root/erp42_ws/install/setup.bash
ros2 launch erp42 erp42.launch.py gui:=false localization:=false
```

둘 다 effort 모드의 `idle_hold:=true`가 기본값이다. 원본 Gazebo 자유 구름을 관찰하려면 `idle_hold:=false` 또는 GUI 실행 옵션 `--no-idle-hold`를 사용한다. 차동구동 테스트에는 `drive_mode:=skid` 또는 GUI 스크립트 `--skid`를 사용한다.

```bash
ros2 node list | grep erp42_idle_hold
ros2 topic info /wheel_effort/commands -v
ros2 control list_controllers
```

기본 상태에서는 보정 노드가 `/wheel_effort/commands`에 유일한 발행자다. 주행 시에는 외부 구동 노드가 주기적으로 토크를 발행해야 한다. `ros2 topic pub --once`와 같은 한 번의 짧은 발행은 보정 노드의 publisher-ownership 전환을 보장하지 않으므로 권장하지 않는다.

### 제어 방식

파일: `erp42_control/erp42_control/idle_hold.py`.

- `/odom` 차체의 XY 위치·yaw·전방 속도를 읽는다. 최초 접촉 안정화 후 위치를 보정 목표로 저장한다.
- 오차를 차량 전방 축으로 투영하고 `tau = clamp(kp_position * position_error - kd_velocity * forward_speed, ±max_effort)`를 모든 바퀴에 발행한다.
- 기본 파라미터: `kp_position=20.0`, `kd_velocity=100.0`, `max_effort=4.0 N·m/wheel`; `odom` 타임아웃 0.4초.
- 외부 발행자 존재 시 제어권을 양보하고 내부 위치 기준점을 폐기한다. 외부 발행자 소멸 후 새로운 현재 위치를 목표로 잡는다.
- SIGINT 종료 전에 제어 토크를 0으로 내보내도록 ROS 문맥 종료 순서를 조정했다.

이는 **시뮬레이터의 정지 보정**이며, 실차용 안전 브레이크나 경사로 주차 기능을 대신하지 않는다. 센서·ROS 2 그래프 연결, 급정거, 높은 속도, 급경사에서 별도 안전 검증이 필요하다.

## 적용 위치

- `erp42_control/erp42_control/idle_hold.py`: 신규 정지 보정 ROS 2 노드
- `erp42_control/setup.py`: `ros2 run erp42_control idle_hold` 진입점
- `erp42/launch/erp42.launch.py`: effort 모드에서 조건부 자동 시작, `idle_hold` 인자
- `scripts/start_gazebo_gui_wsl.sh`: `--no-idle-hold` 옵션
- `scripts/rebuild_erp42_ros2_wsl.sh`: 변경 ROS 2 패키지 2개만 증분 빌드
- `README.md`, `GAZEBO_GUI_WSL.md`: 사용법
- `scripts/compare_idle_drift_wsl.py`, `scripts/test_idle_hold_takeover.py`: A/B 및 명령권 전환 검증 코드

WSL `/root/erp42_ws/install`에 실제 빌드 및 반영 완료. 원본 설치본 파일 백업:

- `/root/erp42_ws/backups/idle_drift_20261121/macros_erp42.xacro`
- `/root/erp42_ws/backups/idle_drift_20261121/erp42.launch.py`

## 검증 결과

1. ROS2 패키지 `erp42_control`, `erp42` **colcon 빌드 성공** (2 packages).
2. 기본 실행에서 `/erp42_idle_hold` 노드 자동 실행 및 `joint_state_broadcaster`, `steering`, `wheel_effort` 모두 active 확인.
3. `idle_hold:=false`에서 해당 노드가 없고 후방 이동 재현 확인.
4. `drive_mode:=skid`에서 해당 노드를 실행하지 않고 `skid_drive` 컨트롤러 active 확인.
5. 외부 토크 1 N·m ×4 를 3초간 발행하는 시험에서 보정 노드 로그의 `yielding` / `reacquiring hold point` 확인. 새로운 전진 위치 기준으로 감속·복귀 확인.
6. 기존 `scripts/test_humble_wsl.sh effort` 전체 smoke test 성공 (카메라 4대, IMU, TF, EKF, 주행 약 14.2m, 조향 목표 0.15rad 추종).
7. 기존 `scripts/test_humble_wsl.sh skid` 전체 smoke test 성공 (카메라 4대, IMU, TF, EKF, /cmd_vel 주행 약 3.0m).

테스트 종료 후 simulator는 종료했다. 앱에서 다시 실행하면 새 보정 노드가 기본으로 활성화된다.
