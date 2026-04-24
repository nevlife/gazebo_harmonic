# Hunter Gazebo Fortress Simulation

Gazebo Fortress(Ignition Gazebo 6) + ROS 2 Humble 환경에서 동작하는 Hunter 로봇 시뮬레이션 패키지입니다.

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `gazebo_harmonic` | 시뮬레이션 환경, Launch 파일, URDF/SDF, 컨트롤러 설정, `gps_covariance_relay` 노드 |
| `hunter_base` | Hunter 로봇 기본 URDF 모델(링크/휠/박스 분리 구성) 및 STL 메쉬 리소스 |
| `external/RGLGazeboPlugin` | GPU 가속 LiDAR 시뮬레이션 플러그인 (Gaussian noise 추가) |

## 시스템 요구사항

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- NVIDIA GPU (CUDA 지원) + 드라이버

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
- **플러그인**: `ignition-gazebo-ackermann-steering-system`
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
| LiDAR | Velodyne VLP-32C (Ultra Puck) | `/velodyne_points` | 10 Hz | Gaussian distance stddev 0.03 m |
| IMU | Generic IMU | `/imu` | 100 Hz | Gaussian (축별 상이) |
| GPS | Generic NavSat | `/gps/fix` | 10 Hz | Gaussian stddev 0.00000045 deg (~0.05 m) |
| Camera | Generic Camera | `/camera/raw` | 15 Hz | - |

#### LiDAR 상세 (RGL GPU LiDAR)

[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)을 사용한 하드웨어 가속 레이트레이싱 기반 LiDAR 시뮬레이션입니다. Gazebo 기본 `gpu_lidar` 대비 약 4배 성능 향상을 제공합니다.

| 항목 | 값 |
|---|---|
| 플러그인 | RGLServerPluginInstance (custom sensor) |
| 패턴 | Ultra Puck 프리셋 (VLP-32C 실제 채널 분포) |
| 수직 채널 | 32채널, 비균일 분포 (-25° ~ +15°) |
| 수평 FOV | 360° |
| 측정 범위 | 0.1 m ~ 200 m |
| 노이즈 | Gaussian distance (mean: 0, stddev_base: 0.03 m) |

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
| FOV | 120° (수평) |
| 업데이트 주파수 | 15 Hz |
| 클리핑 | 0.1 m ~ 100 m |
| 마운트 위치 | front_box 링크, LiDAR 전방 하부 |

> 카메라 파라미터는 `src/gazebo_harmonic/urdf/camera_gazebo.xacro`에서 수정합니다.

#### GPS 파이프라인

```
Gazebo /gps  →  ros_gz_bridge  →  /gps/raw  →  gps_covariance_relay  →  /gps/fix
```

`gps_covariance_relay` 노드는 공분산 정보가 없는 raw NavSatFix 메시지에 대각 공분산 행렬을 추가하여 표준 포맷으로 변환합니다.
- 수평/수직 분산: 0.0025 m² (= stddev 0.05 m)
- `COVARIANCE_TYPE_DIAGONAL_KNOWN`, `STATUS_FIX` 설정

### 오도메트리 및 TF

| 토픽 | 설명 | TF |
|---|---|---|
| `/odom` | Ackermann 플러그인 기반 오도메트리 (`odom` → `base_link`) | `/tf_gz` 발행 |
| `/odometry/ground_truth` | 물리 엔진 기반 ground truth (`world` → `base_link`) | `/tf` 발행 |
| `/joint_states` | 모든 조인트 상태, 50 Hz | - |

> Ackermann 플러그인의 TF는 `/tf_gz`로, OdometryPublisher의 TF는 `/tf`로 분리되어 발행됩니다.

## ROS-Gazebo 브리지 토픽

| Gazebo 토픽 | ROS 토픽 | 메시지 타입 |
|---|---|---|
| `/imu` | `/imu` | `sensor_msgs/Imu` |
| `/gps` | `/gps/raw` | `sensor_msgs/NavSatFix` |
| `/velodyne_points` | `/velodyne_points` | `sensor_msgs/PointCloud2` |
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` |
| `/clock` | `/clock` | `rosgraph_msgs/Clock` |
| `/odometry/wheel` | `/odometry/wheel` | `nav_msgs/Odometry` |
| `/odometry/ground_truth` | `/odometry/ground_truth` | `nav_msgs/Odometry` |
| `/joint_states` | `/joint_states` | `sensor_msgs/JointState` |
| `/camera/raw` | `/camera/raw` | `sensor_msgs/Image` |

## 의존성

### 1. Gazebo Fortress 설치

[Gazebo Fortress 공식 설치 가이드](https://gazebosim.org/docs/fortress/install_ubuntu/)를 참고하여 설치합니다.

### 2. ROS 패키지 설치

```bash
sudo apt install \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard
```

> `ros-humble-ros-gz-sim`은 ROS 2 Humble에서 기본적으로 Gazebo Fortress(Ignition 6)에 연결됩니다.

### 3. rosdep 의존성 설치

```bash
cd ~/dev/hunter_gazebo
rosdep install --from-paths src --ignore-src -r -y
```

## 빌드

### 1. RGL Gazebo Plugin (GPU LiDAR)

NVIDIA GPU 가속 LiDAR 시뮬레이션을 위한 [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) (Fortress 브랜치)을 사용합니다. CUDA 지원 GPU와 NVIDIA 드라이버가 필요합니다.

```bash
cd ~/dev/hunter_gazebo/external/RGLGazeboPlugin
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install && make -j$(nproc) && make install
```

빌드 결과물은 `external/RGLGazeboPlugin/install/`에 생성됩니다. launch 파일에서 환경변수(`IGN_GAZEBO_SYSTEM_PLUGIN_PATH`, `IGN_GUI_PLUGIN_PATH`, `RGL_PATTERNS_DIR`)를 자동 설정하므로 별도 export는 필요 없습니다.

### 2. 워크스페이스 빌드

```bash
cd ~/dev/hunter_gazebo
colcon build
source install/setup.bash
```

## 실행

### Empty World (GPS 포함, 전체 기능)

```bash
ros2 launch gazebo_harmonic hunter_sim_start.launch.py
```

### Baylands World

```bash
ros2 launch gazebo_harmonic hunter_simple_baylands.launch.py
```

### 키보드 원격 제어

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### rviz2 시각화

시뮬레이션 launch와 동일한 터미널(source된 환경)에서 실행하거나, 별도 터미널에서 source 후 실행합니다.

```bash
source ~/dev/hunter_gazebo/install/setup.bash
rviz2
```

> Fixed Frame을 `base_link` 또는 `velodyne_sensor`로 설정하세요.

## 월드 파일

| 파일 | 설명 | 물리 step |
|---|---|---|
| `empty_with_gps.sdf` | 빈 평지 + GPS용 구면 좌표계 설정 (기본) | 1 ms |
| `simple_baylands.sdf` | Baylands 지형 환경 (Fuel 모델 자동 다운로드) | 5 ms |

> 두 월드 모두 DART 물리 엔진을 사용합니다 (Fortress 기본).

### GPS 기준 좌표

| 월드 | 위도 | 경도 | 비고 |
|---|---|---|---|
| empty_with_gps | 37.2397°N | 126.7736°E | 한국 |
| simple_baylands | 37.4122°N | -121.9989°W | 미국 캘리포니아 |

## 플러그인 목록

### 월드 플러그인 (SDF)

| 플러그인 | 역할 |
|---|---|
| `ignition-gazebo-physics-system` | 물리 엔진 (DART) |
| `ignition-gazebo-sensors-system` | 센서 처리 (ogre2 렌더러) |
| `ignition-gazebo-user-commands-system` | GUI 명령 처리 |
| `ignition-gazebo-scene-broadcaster-system` | 씬 브로드캐스트 |
| `rgl::RGLServerPluginManager` | RGL GPU LiDAR 씬 동기화 |

### 로봇 플러그인 (URDF/Xacro)

| 플러그인 | 역할 |
|---|---|
| `rgl::RGLServerPluginInstance` | GPU 가속 LiDAR 센서 (Ultra Puck 프리셋) |
| `ignition-gazebo-ackermann-steering-system` | 조향 + 구동 제어, `/odom` 발행 |
| `ignition-gazebo-joint-state-publisher-system` | `/joint_states` 발행 |
| `ignition-gazebo-odometry-publisher-system` | ground truth 오도메트리 발행 |
| `ignition-gazebo-imu-system` | IMU 센서 처리 |
| `ignition-gazebo-navsat-system` | GPS 센서 처리 |
| `ignition-gazebo-sensors-system` (camera) | 카메라 센서 처리 |
