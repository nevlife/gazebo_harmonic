# Hunter Gazebo Harmonic Simulation

Gazebo Harmonic(gz-sim 8) + ROS 2 Jazzy 환경에서 동작하는 Hunter 로봇 시뮬레이션 패키지입니다.

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `gazebo_harmonic` | 시뮬레이션 환경, Launch 파일, URDF/SDF, 컨트롤러 설정, `gps_covariance_relay`/`vehicle_speed_publisher` 노드 |
| `hunter_base` | Hunter 로봇 기본 URDF 모델(링크/휠/박스 분리 구성) 및 STL 메쉬 리소스 |
| `external/RGLGazeboPlugin` | GPU 가속 LiDAR 시뮬레이션 플러그인 (Gaussian noise 추가) |
| `scout_base` | Scout 2.0 모델과 원본 DAE 메쉬 |
| `limo_base` | LIMO 4륜 차동 모델 |

## 시스템 요구사항

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim 8)
- NVIDIA GPU (CUDA 지원) + 드라이버 — RGL 사용 시 필수

## 시스템 구성

### URDF 구조

`hunter_gazebo.xacro`가 최상위 파일로, 두 패키지의 xacro를 조합합니다.

```
hunter_gazebo.xacro (robot name: hunter2)
├── gazebo_harmonic/urdf/
│   ├── hunter_base.xacro       # 마찰 계수, 재질, IMU/GPS 센서 플러그인
│   ├── gazebo_control.xacro    # Ackermann 조향, JointState, OdometryPublisher
│   ├── velodyne_VLP32C_gazebo.xacro  # RGL LiDAR 센서 설정
│   └── camera_gazebo.xacro          # 카메라 센서 플러그인
└── hunter_base/urdf/
    ├── base_link.xacro
    ├── front_box_link.xacro / rear_box_link.xacro
    ├── front_wheels.xacro / rear_wheels.xacro
    ├── velodyne_VLP32C.xacro   # LiDAR 링크/조인트 정의
    └── camera.xacro            # 카메라 링크/조인트 정의
```

> `for_rviz` 플래그로 Gazebo/rviz2 간 비주얼을 분기합니다. logo 메쉬만 Gazebo 전용이며, body/shadow/lidar 메쉬는 양쪽 모두 표시됩니다.

### 조향 시스템

Hunter 로봇은 **Ackermann Steering** 구조를 사용합니다.

- **구동 방식**: 후륜 구동(RWD), 전륜 조향
- **플러그인**: `gz-sim-ackermann-steering-system`
- **물리 파라미터**

| 항목 | 값 |
|---|---|
| 휠베이스 | 0.65142 m |
| 휠 간격 (kingpin_width) | 0.585 m |
| 휠 반경 | 0.165 m |
| 조향 제한 | ±0.461 rad |
| 속도 제한 | ±10 m/s |
| 가속도 제한 | ±5 m/s² |

### 센서

| 센서 | 모델 | 토픽 | 주파수 | 노이즈 |
|---|---|---|---|---|
| LiDAR | Velodyne VLP-32C (Ultra Puck) | `/velodyne_points` | 10 Hz | Gaussian distance stddev_base 0.008 m |
| IMU | Generic IMU | `/imu` | 100 Hz | Gaussian (축별 상이, 아래 표) |
| GPS | Generic NavSat | `/gps/fix` | 10 Hz | Gaussian stddev 0.00000045 deg (~0.05 m) |
| Camera | Generic Camera | `/camera/raw` | 15 Hz | - |

#### LiDAR 상세 (RGL GPU LiDAR)

[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)을 사용한 하드웨어 가속 레이트레이싱 기반 LiDAR 시뮬레이션입니다. Gazebo 기본 `gpu_lidar` 대비 빠른 처리량을 제공합니다.

| 항목 | 값 |
|---|---|
| 플러그인 | `rgl::RGLServerPluginInstance` (custom sensor) |
| 패턴 | Ultra Puck 프리셋 (VLP-32C 실제 채널 분포) |
| 수직 채널 | 32채널, 비균일 분포 |
| 수평 FOV | 360° |
| 측정 범위 | 0.1 m ~ 200 m |
| 노이즈 | Gaussian distance (mean: 0, stddev_base: 0.008 m, stddev_rise_per_meter: 0) |
| 출력 frame_id | `velodyne_sensor` |

> 라이다 파라미터는 `src/gazebo_harmonic/urdf/velodyne_VLP32C_gazebo.xacro`에서 수정합니다.
> `pattern_preset`을 `pattern_uniform`으로 교체하면 수평 샘플 수, 수직 채널 수, 각도 범위를 직접 지정할 수 있습니다 (주석 처리된 예시 참고).

#### IMU 노이즈 파라미터

| 축 | 각속도 stddev (rad/s) | 선가속도 stddev (m/s²) |
|---|---|---|
| X | 0.012432 | 0.264649 |
| Y | 0.013663 | 0.406186 |
| Z | 0.211278 | 0.153983 |

#### Camera 상세

| 항목 | 값 |
|---|---|
| 해상도 | 1280 × 720 (HD) |
| FOV | 2.0944 rad (≈120° 수평) |
| 업데이트 주파수 | 15 Hz |
| 클리핑 | 0.1 m ~ 100 m |
| 마운트 위치 | front_box 링크, LiDAR 전방 하부 |

> 카메라 파라미터는 `src/gazebo_harmonic/urdf/camera_gazebo.xacro`에서 수정합니다.

#### GPS 파이프라인

```
Gazebo /gps  →  ros_gz_bridge  →  /gps/raw  →  gps_covariance_relay  →  /gps/fix
```

`gps_covariance_relay` 노드는 공분산 정보가 없는 raw NavSatFix 메시지에 대각 공분산 행렬을 추가하여 표준 포맷으로 변환합니다.
- 수평/수직 분산: 0.0025 m² (= stddev 0.05 m, 파라미터로 조정 가능)
- `COVARIANCE_TYPE_DIAGONAL_KNOWN`, `STATUS_FIX`, `SERVICE_GPS` 설정

#### 차량 속도 토픽

`vehicle_speed_publisher` 노드는 `/odometry/wheel`의 `twist.twist.linear.x`를 추출해 `/vehicle/speed` (`std_msgs/Float64`)로 publish 합니다.

### 오도메트리 및 TF

| 토픽 | 설명 | TF |
|---|---|---|
| `/odometry/wheel` | Ackermann 플러그인 오도메트리 (`odom` → `base_link`), 50 Hz | `/tf_wheel` 발행 |
| `/odometry/ground_truth` | 물리 엔진 ground truth (`world` → `base_link_ground_truth`), 50 Hz | `/tf_gt` 발행 |
| `/joint_states` | 모든 조인트 상태, 50 Hz | - |

> Ackermann 플러그인의 TF는 `/tf_wheel`로, OdometryPublisher의 TF는 `/tf_gt`로 분리되어 발행됩니다 (메인 `/tf`와 충돌 방지). 필요한 쪽을 `/tf`로 remap 해서 사용하세요.

### 정적 TF (launch에서 발행)

Gazebo 센서 frame 이름은 `<model>/<link>/<sensor>` 패턴입니다. URDF 링크와 매칭하기 위해 launch에서 다음 static TF를 발행합니다:

| Parent | Child | Offset (xyz) |
|---|---|---|
| `velodyne_sensor` | `hunter2/base_link/velodyne_sensor` | 0, 0, 0 |
| `base_link` | `hunter2/base_link/imu_sensor` | 0, 0, 0.05 |
| `base_link` | `hunter2/base_link/navsat_sensor` | 0, 0, 0.1 |
| `camera_link` | `hunter2/base_link/camera_sensor` | 0, 0, 0 |

> RGL plugin은 `<frame>velodyne_sensor</frame>`를 직접 지정하므로 `/velodyne_points`의 `frame_id`는 항상 `velodyne_sensor`입니다.

## ROS-Gazebo 브리지 토픽

| Gazebo 토픽 | ROS 토픽 | 메시지 타입 |
|---|---|---|
| `/imu` | `/imu` | `sensor_msgs/Imu` (`gz.msgs.IMU`) |
| `/gps` | `/gps/raw` (remap) | `sensor_msgs/NavSatFix` (`gz.msgs.NavSat`) |
| `/velodyne_points` | `/velodyne_points` | `sensor_msgs/PointCloud2` (`gz.msgs.PointCloudPacked`) |
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` (`gz.msgs.Twist`) |
| `/clock` | `/clock` | `rosgraph_msgs/Clock` (`gz.msgs.Clock`, gz→ros only) |
| `/odometry/wheel` | `/odometry/wheel` | `nav_msgs/Odometry` (`gz.msgs.Odometry`) |
| `/odometry/ground_truth` | `/odometry/ground_truth` | `nav_msgs/Odometry` (`gz.msgs.Odometry`) |
| `/joint_states` | `/joint_states` | `sensor_msgs/JointState` (`gz.msgs.Model`) |
| `/camera/raw` | `/camera/raw` | `sensor_msgs/Image` (`gz.msgs.Image`, gz→ros only) |

## 의존성

### 1. Gazebo Harmonic 설치

ROS 2 Jazzy 패키지(`ros-jazzy-ros-gz`)는 기본적으로 Gazebo Harmonic(gz-sim 8)을 사용합니다. 별도 Gazebo 설치는 필요 없으며, 직접 설치하려면 [Gazebo Harmonic 공식 설치 가이드](https://gazebosim.org/docs/harmonic/install_ubuntu/)를 참고하세요.

### 2. ROS 패키지 설치

```bash
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-ros
```

### 3. rosdep 의존성 설치

```bash
cd ~/dev/hunter_gazebo
rosdep install --from-paths src external --ignore-src -r -y
```

## 빌드

`RGLGazeboPlugin`은 `external/`에 있지만 colcon이 함께 빌드합니다 (별도 standalone build 불필요). RobotecGPULidar 라이브러리(`.so`)는 첫 빌드 시 자동 다운로드됩니다.

```bash
cd ~/dev/hunter_gazebo
colcon build
source install/setup.bash
```

빌드 결과:
- `install/RGLGazeboPlugin/RGLServerPlugin/` — 서버 플러그인 (`libRGLServerPluginInstance.so`, `libRGLServerPluginManager.so`)
- `install/RGLGazeboPlugin/RGLVisualize/` — GUI 플러그인
- launch 파일이 `GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_GUI_PLUGIN_PATH`, `RGL_PATTERNS_DIR`을 자동 설정하므로 별도 export 불필요

> **경고**: `RGLGazeboPlugin` 패키지명이 ROS 명명 규칙(소문자/`_`)과 어긋나서 colcon이 경고를 출력합니다. 빌드 자체에는 영향 없음.

## 실행

### Empty World (GPS 포함, 기본)

```bash
ros2 launch gazebo_harmonic hunter_sim_start.launch.py
```

### Baylands World

```bash
ros2 launch gazebo_harmonic hunter_simple_baylands.launch.py
```

### Scout 2.0

```bash
ros2 launch gazebo_harmonic scout_sim_start.launch.py
```

주요 토픽은 `/cmd_vel`, `/scout/odometry`, `/scout/joint_states`,
`/scout/scan`, `/scout/camera/*`입니다.

### LIMO

```bash
ros2 launch gazebo_harmonic limo_sim_start.launch.py
```

주요 토픽은 `/cmd_vel`, `/limo/odometry`, `/limo/joint_states`,
`/limo/scan`, `/limo/imu`, `/limo/camera/*`입니다.

### 키보드 원격 제어

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### rviz2 시각화

시뮬레이션 launch와 동일한 환경(source된 셸)에서 실행:

```bash
source ~/dev/hunter_gazebo/install/setup.bash
rviz2
```

> Fixed Frame을 `base_link` 또는 `velodyne_sensor`로 설정하세요.

## 월드 파일

| 파일 | 설명 | 물리 step | 물리 엔진 |
|---|---|---|---|
| `empty_with_gps.sdf` | 빈 평지 + GPS용 구면 좌표계 (기본) | 1 ms | DART (default) |
| `simple_baylands.sdf` | Baylands 지형 (Fuel 모델 자동 다운로드) | 5 ms (200 Hz) | ODE |

### GPS 기준 좌표

| 월드 | 위도 | 경도 | 비고 |
|---|---|---|---|
| empty_with_gps | 37.2397°N | 126.7736°E | 한국 |
| simple_baylands | 37.4122°N | -121.9989°W | 미국 캘리포니아 |

## 플러그인 목록

### 월드 플러그인 (SDF)

**`empty_with_gps.sdf`**

| 플러그인 | 역할 |
|---|---|
| `gz-sim-physics-system` | 물리 엔진 (DART) |
| `gz-sim-sensors-system` | 센서 처리 (ogre2 렌더러) |
| `gz-sim-user-commands-system` | GUI 명령 처리 |
| `gz-sim-scene-broadcaster-system` | 씬 브로드캐스트 |
| `rgl::RGLServerPluginManager` | RGL GPU LiDAR 씬 동기화 (lidar_link 내부 entity 무시) |

**`simple_baylands.sdf`** — 위 외에 추가:

| 플러그인 | 역할 |
|---|---|
| `gz-sim-contact-system` | 접촉 감지 |
| `gz-sim-imu-system` (world) | IMU 시스템 |
| `gz-sim-air-pressure-system` | 기압 센서 |
| `gz-sim-apply-link-wrench-system` | 외력/토크 적용 |
| `gz-sim-navsat-system` (world) | GPS 시스템 |

### 로봇 플러그인 (URDF/Xacro)

| 플러그인 | 역할 |
|---|---|
| `rgl::RGLServerPluginInstance` | GPU 가속 LiDAR 센서 (Ultra Puck 프리셋) |
| `gz-sim-ackermann-steering-system` | 조향 + 구동 제어, `/odometry/wheel` 발행 |
| `gz-sim-joint-state-publisher-system` | `/joint_states` 발행 |
| `gz-sim-odometry-publisher-system` | ground truth 오도메트리 발행 |
| `gz-sim-imu-system` | IMU 센서 처리 |
| `gz-sim-navsat-system` | GPS 센서 처리 |
| `gz-sim-sensors-system` (camera) | 카메라 센서 처리 |

