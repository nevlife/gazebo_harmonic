```bash
ros2 launch gazebo_harmonic hunter_sim_start.launch.py
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r /cmd_vel:=/ackermann_steering_controller/reference \
    -p stamped:=true \
    -p frame_id:=base_link
```