# ERP42 Gazebo Classic GUI (WSL2 Ubuntu-22.04)

ERP42의 ROS 2 Humble 빌드 워크스페이스는 `/root/erp42_ws`에 있다. 이 파일과 실행 스크립트는 해당 WSL 폴더에도 복사한다. 기존의 `install/`, `build/`, `log/`는 그대로 사용한다.

## Windows에서 실행

PowerShell 또는 Windows Terminal에서 다음 명령을 실행하면 Windows 데스크톱에 WSLg Gazebo 11 GUI가 열린다.

```powershell
wsl -d Ubuntu-22.04 -u root -- bash /root/erp42_ws/start_gazebo_gui.sh
```

Gazebo와 RViz2를 동시에 열어 차체 및 ROS 2 TF/센서 시각화를 확인하려면:

```powershell
wsl -d Ubuntu-22.04 -u root -- bash /root/erp42_ws/start_gazebo_gui.sh --rviz
```

`/cmd_vel`을 쓰는 차동구동 모드로 실행하려면 `--skid`를 추가한다. 기본 effort 모드에서는 `idle_hold:=true`로 무조작 시 수 mm 규모의 후방 이동을 막는 소형 토크 피드백이 시작된다. 물리 원본을 비교하려면 `--no-idle-hold`를 지정한다. 지속 주행 명령을 보내는 ROS 노드가 있으면 정지 보정은 제어권을 넘긴다. WSLg의 GPU 렌더링이 실패한다면 `--software-gl`을 추가하면 CPU 소프트웨어 렌더러로 전환된다. 실행 터미널에서 `Ctrl+C`를 누르면 ROS 2 launch가 정리된다.

## WSL2에서 실행

```bash
bash /root/erp42_ws/check_gazebo_gui.sh
bash /root/erp42_ws/start_gazebo_gui.sh --rviz
```

다른 WSL2 터미널에서 Gazebo가 동작 중인 경우:

```bash
source /opt/ros/humble/setup.bash
source /root/erp42_ws/install/setup.bash
ros2 topic list
ros2 control list_controllers
ros2 topic echo --once /odom
```

RViz2의 기본 구성이 `erp42` 패키지의 `config/erp42.rviz`를 사용하며, `/clock`을 구독하는 `use_sim_time` 환경에서 실행된다. Gazebo에서 차량을 움직이려면 기존 `ERP42-model/README.md`의 `/wheel_effort/commands` 및 `/steering/commands` 예시를 사용하거나 `--skid` 모드에서 `/cmd_vel`을 발행한다.

## 환경과 파일 위치

| 항목 | 경로 또는 설정 |
| --- | --- |
| WSL 배포판 | `Ubuntu-22.04` |
| ROS 2 | `/opt/ros/humble` |
| Gazebo Classic | `/usr/bin/gazebo` (11.10.2) |
| ERP42 workspace | `/root/erp42_ws` |
| Gazebo GUI 스크립트 | `/root/erp42_ws/start_gazebo_gui.sh` |
| 진단 스크립트 | `/root/erp42_ws/check_gazebo_gui.sh` |
| ERP42 설치 launch | `/root/erp42_ws/install/erp42/share/erp42/launch/erp42.launch.py` |
| Windows 소스 | `C:\Users\mango\OneDrive\바탕 화면\연구\ERP42-model` |

WSLg는 Windows와 Linux 창을 연결한다. `glxinfo -B`에서 NVIDIA 기반 `D3D12` 렌더러가 출력되면 하드웨어 경로가 활성화된 상태다. 소프트웨어 렌더러는 `llvmpipe`로 표시된다.

**2026-11-21 실행 검증:** `gui:=true` 실행에서 ERP42 모델 생성과 `/clock`, 4개 카메라 토픽, IMU, `/odom`, `/tf`가 확인됐고 `joint_state_broadcaster`, `steering`, `wheel_effort` 컨트롤러가 모두 활성화됐다. `--rviz` 실행에서는 `gzserver`, `gzclient`, `rviz2` 프로세스와 RViz2 OpenGL 4.2 초기화가 확인됐다. 화면 내 모델의 외관은 별도 육안 검증 대상이다.

### 문제 해결

- **GUI가 나타나지 않는 경우:** Windows에서 `wsl --list --verbose`로 Ubuntu-22.04가 WSL2인지 확인하고 WSL 터미널에서 `echo $DISPLAY`와 `glxinfo -B`를 점검한다.
- **OpenGL 오류나 빈 화면:** `--software-gl`을 사용한다. 소프트웨어 렌더링은 GPU 모드보다 느릴 수 있다.
- **기존 Gazebo 프로세스 경고:** 먼저 기존 시뮬레이션을 종료한 후 다시 시작한다. 서로 다른 두 서버가 같은 Gazebo 기본 포트를 사용하지 않도록 한다.
- **`erp42` 패키지를 찾을 수 없는 경우:** `source /opt/ros/humble/setup.bash`와 `source /root/erp42_ws/install/setup.bash`를 수행하고, 필요하면 Windows 소스의 `scripts/build_humble_wsl.sh`로 다시 빌드한다.
