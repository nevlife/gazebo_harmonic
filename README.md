# Hunter Gazebo Harmonic Simulation

Gazebo Harmonic 환경에서 동작하도록 개발된 Hunter 로봇 시뮬레이션 패키지입니다. Ackermann Steering 구조를 지원하며, ROS 2와 Gazebo Harmonic 간의 원활한 연동을 위해 `ros2_control` 및 `gz_ros2_control`을 활용합니다.

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

## 시스템 구성
- **제어**: `ackermann_steering_controller`를 통해 조향과 주행을 제어합니다.
- **URDF**: Gazebo Harmonic에 최적화된 `ros2_control` 설정을 포함하고 있습니다 (`src/gazebo_harmonic/urdf/ros2_control.xacro`).
- **파라미터**: `src/gazebo_harmonic/config/ackermann_like_controller.yaml`에서 물리 파라미터(Wheelbase 등)를 확인할 수 있습니다.