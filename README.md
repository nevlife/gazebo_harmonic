# Hunter Gazebo Harmonic Simulation

Gazebo Harmonic 환경에서 동작하도록 개발된 Hunter 로봇 시뮬레이션 패키지입니다. Ackermann Steering 구조를 지원하며, ROS 2와 Gazebo Harmonic 간의 원활한 연동을 위해 `ros2_control` 및 `gz_ros2_control`을 활용합니다.

## 시스템 구성 및 사양

### 1. 조향 시스템 (Steering System)
Hunter 로봇은 **Ackermann Steering** 기하학 구조를 기반으로 합니다.
- **구동 방식**: 후륜 구동 (Rear-Wheel Drive), 전륜 조향 (Front-Wheel Steering)
- **컨트롤러**: `ackermann_steering_controller` (ros2_control)


### 2. 센서 (Sensors)
시뮬레이션 모델에는 자율 주행 및 내비게이션 연구를 위한 다양한 센서가 포함되어 있습니다.

| 센서 타입 | 모델명 | 토픽 (Topic) | 설명 |
|---|---|---|---|
| **LiDAR** | Velodyne VLP-32C | `/velodyne_points` | 32채널, Gaussian Noise 적용 (stddev: 0.008) |
| **IMU** | Generic IMU | `/imu` | 100Hz 업데이트, **오차 미적용 (Ideal)** |
| **GPS** | Generic NavSat | `/gps` | 10Hz 업데이트, **오차 미적용 (Ideal)** |

> **참고**: IMU 및 GPS 센서는 현재 노이즈나 오차가 적용되지 않은 이상적인(Ideal) 상태의 데이터를 제공합니다. 센서 오차 시뮬레이션이 필요한 경우 URDF의 관련 설정을 수정해야 합니다.

## 주요 패키지

- **gazebo_harmonic**: Gazebo Harmonic 전용 시뮬레이션 환경, Launch 파일, URDF 설정 및 컨트롤러 구성을 포함합니다.
- **hunter_base**: Hunter 로봇의 기본 URDF 모델 및 메쉬 리소스를 제공합니다.

## 실행 방법

### 1. 시뮬레이션 시작
Gazebo Harmonic 환경에서 Hunter 로봇을 실행합니다:

```bash
ros2 launch gazebo_harmonic hunter_sim_start.launch.py
```

제공되는 다른 환경들:
- **Empty World**: `ros2 launch gazebo_harmonic hunter_sim_start.launch.py`
- **Baylands World**: `ros2 launch gazebo_harmonic hunter_simple_baylands.launch.py`

### 2. 로봇 제어 (Teleop)
키보드를 사용하여 로봇을 조종할 수 있습니다:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/ackermann_steering_controller/reference \
    -p stamped:=true \
    -p frame_id:=base_link
```