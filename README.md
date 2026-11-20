# ERP42 · ROS 2 Humble

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11용 시뮬레이션 모델이다. 원본의 차체·바퀴 메시, 관절 배치, 조향 범위(±0.4 rad), 카메라 4대, IMU 및 5° 경사로를 사용한다. 기본 주행 입력은 바퀴별 토크(N·m)와 앞바퀴별 조향각(rad)이다. [원본 저장소](https://github.com/nabihandres/ERP42-model/tree/fcb5d4c1758638336f23295c3b23c525a8b7770f)

## Windows / WSL 2 환경

Windows의 PowerShell에서 이 저장소 디렉터리로 이동한 뒤 실행한다. 기존 Ubuntu 24.04/26.04 WSL 배포판과는 별개로 **Ubuntu 22.04**에 Humble을 설치한다.

```powershell
wsl --install -d Ubuntu-22.04
wsl -d Ubuntu-22.04 -u root -- bash scripts/bootstrap_humble_wsl.sh
wsl -d Ubuntu-22.04 -u root -- bash scripts/build_humble_wsl.sh
```

두 번째 스크립트는 빌드 파일을 WSL 내부의 `/root/erp42_ws`에 저장한다. 시뮬레이션은 다음 명령으로 실행할 수 있다. 이 명령은 화면을 표시하지 않는 환경에서 소프트웨어 OpenGL과 가상 디스플레이를 사용한다.

```powershell
wsl -d Ubuntu-22.04 -u root -- bash -lc "source /root/erp42_ws/install/setup.bash; LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a ros2 launch erp42 erp42.launch.py gui:=false localization:=true"
```

별도의 PowerShell 터미널을 같은 저장소 위치에서 열고 아래와 같이 실제 주행·토픽 수신을 검증한다.

```powershell
wsl -d Ubuntu-22.04 -u root -- bash -lc "source /root/erp42_ws/install/setup.bash; python3 scripts/smoke_humble.py --localization"
```

Gazebo 실행부터 종료까지 자동화된 검사는 한 터미널에서 실행할 수도 있다. 첫 번째는 토크·전륜 조향 모드, 두 번째는 `/cmd_vel` 차동구동 모드다. 실행 로그는 각각 `/tmp/erp42_effort_gazebo.log`, `/tmp/erp42_skid_gazebo.log`에 남는다.

```powershell
wsl -d Ubuntu-22.04 -u root -- bash scripts/test_humble_wsl.sh effort
wsl -d Ubuntu-22.04 -u root -- bash scripts/test_humble_wsl.sh skid
```

## 빌드

Humble과 ROS apt 저장소가 설정된 Ubuntu 터미널에서 실행한다. 아래 첫 줄은 **이 README가 있는 디렉터리에서** 실행한다. 워크스페이스와 빌드 결과는 Linux 홈 디렉터리에 둔다.

```bash
ERP42_SRC="$(pwd -P)"
source /opt/ros/humble/setup.bash
sudo apt install build-essential python3-colcon-common-extensions python3-rosdep
# rosdep을 처음 쓰는 시스템에서 한 번만 실행
test -e /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init
rosdep update --rosdistro humble

mkdir -p ~/erp42_ws
cd ~/erp42_ws
rosdep install --from-paths "$ERP42_SRC" --ignore-src --rosdistro humble -r -y
colcon build --base-paths "$ERP42_SRC" --packages-up-to erp42
source install/setup.bash
```

새 터미널에서는 `source ~/erp42_ws/install/setup.bash`를 먼저 실행한다. 원본에 동봉된 ROS 1 버전의 `gazebo_ros_pkgs`, `robot_localization`은 빌드에서 제외하고 Humble 배포 패키지를 사용한다.

## 시뮬레이션 실행

```bash
ros2 launch erp42 erp42.launch.py
```

기본 월드에는 평지와 경사로가 있다. Gazebo 창 없이 실행하려면 `gui:=false`, RViz2를 함께 열려면 `rviz:=true`, EKF를 켜려면 `localization:=true`를 추가한다. 원하는 Gazebo Classic 월드를 지정할 때는 `world:=/absolute/path/to/example.world`를 사용한다. 카메라 영상 렌더링에는 OpenGL 디스플레이가 필요하다. 디스플레이가 없는 Linux에서는 Xvfb를 설치하고 다음처럼 실행할 수 있다.

```bash
xvfb-run -a ros2 launch erp42 erp42.launch.py gui:=false localization:=true
```

별도 터미널에서 컨트롤러가 `active`인지 확인한다. 기본 모드에서는 `joint_state_broadcaster`, `wheel_effort`, `steering`이 활성화된다.

```bash
ros2 control list_controllers
```

### 바퀴 토크와 조향 명령

`/wheel_effort/commands`의 배열 순서는 원본 관절명 기준 오른쪽 앞·오른쪽 뒤·왼쪽 앞·왼쪽 뒤다. `/steering/commands`는 오른쪽 앞·왼쪽 앞 순서이며 허용 범위는 각각 −0.4~0.4 rad다. 원본은 차체 좌표계의 +Y 쪽 바퀴를 `right`로 명명한다.

```bash
ros2 topic pub --once /steering/commands std_msgs/msg/Float64MultiArray '{data: [0.15, 0.15]}'
ros2 topic pub --once /wheel_effort/commands std_msgs/msg/Float64MultiArray '{data: [12.0, 12.0, 12.0, 12.0]}'
```

그룹 컨트롤러는 마지막 명령을 유지한다. 토크를 해제하고 조향을 중앙에 놓으려면 다음 명령을 보낸다. 토크 0은 관성 주행을 허용한다.

```bash
ros2 topic pub --once /wheel_effort/commands std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.0]}'
ros2 topic pub --once /steering/commands std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0]}'
```

원본 제어 스크립트 이름도 사용할 수 있다. 다음은 토크와 조향을 동시에 10 Hz로 보내는 예다. 토크만 보내려면 `main.py`, 조향만 보내려면 `steer.py`를 실행한다. 각 노드는 시작할 때 파라미터를 읽는다.

```bash
ros2 run erp42_control motionERP42.py --ros-args -p torque:=12.0 -p angle:=0.15
```

### /cmd_vel 차동구동 모드

실행 중인 시뮬레이션을 종료하고 아래 모드로 다시 실행한다. 앞바퀴 조향을 중앙에 둔 상태에서 좌우 바퀴의 속도차로 선회한다. `/cmd_vel`은 선속도(m/s)와 각속도(rad/s)를 받으며, 마지막 명령에서 0.5초가 지나면 목표 속도를 0으로 설정한다.

```bash
ros2 launch erp42 erp42.launch.py drive_mode:=skid
# 다른 터미널에서
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.15}}'
```

## 센서와 위치 추정

| 토픽 | 데이터 |
| --- | --- |
| `/joint_states` | 바퀴 4개와 조향 관절 2개의 상태 |
| `/imu/data` | `structure` 프레임의 IMU, 20 Hz |
| `/erp42/{middle,left,right,back}_camera/image_raw` | 800×800 RGB 영상, 카메라별 설정 30 Hz |
| `/erp42/{middle,left,right,back}_camera/camera_info` | 각 카메라의 내부 파라미터와 optical frame |
| `/odom` | Gazebo 차체의 실제 자세와 차체 좌표계 속도, 20 Hz. 기본 모드에서는 `odom → structure` TF도 발행 |
| `/odom_skid` | 차동구동 모드의 바퀴 회전 기반 오도메트리 |
| `/noisy_odom` | EKF 옵션에서 횡방향 위치에 ±0.5 m 균등 잡음을 더한 오도메트리, 5 Hz |
| `/odometry/filtered` | EKF 옵션에서 IMU와 `/noisy_odom`을 융합한 평면 추정값. 이 모드에서는 EKF가 `odom → structure` TF를 단독 발행 |

```bash
ros2 topic echo --once /imu/data
ros2 topic hz /erp42/middle_camera/image_raw
ros2 run tf2_ros tf2_echo odom structure
```

모델과 조향 관절만 살펴보려면 `ros2 launch erp42 rviz.launch.xml`을 실행한다. 시뮬레이션 중 그래프는 `ros2 launch erp42 plots.launch.xml`로 열 수 있다. EKF만 별도로 시작할 때는 `ros2 launch odometry_localization ekf_localization.launch.xml`을 사용한다.

### 통합 동작 검사

시뮬레이터 실행 후 별도 Ubuntu 터미널에서 `python3 scripts/smoke_humble.py --localization`을 실행한다(저장소 최상위 폴더 기준). 이 검사는 4개 카메라의 이미지·보정 정보, IMU, 관절 상태, odom/TF, EKF 토픽, 주행 변위 및 조향각을 확인한다. 차동구동을 검사할 때는 시뮬레이션을 `drive_mode:=skid`로 다시 실행한 뒤 `python3 scripts/smoke_humble.py --drive-mode skid`를 사용한다.
