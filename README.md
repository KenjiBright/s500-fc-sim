# S500 Flight Controller Simulator

Mô phỏng flight control cho quadcopter S500 với ROS2 + Gazebo Humble.

## Thông số Drone
- **Frame:** DJI Phantom 3 inspired S500 (500mm wheelbase)
- **Motor:** DJI 2312 800KV + 10" propeller
- **ESC:** Mamba F55/F65
- **Battery:** Lipo 4S 75C Ovonic
- **Total Mass:** ~1.1 kg
- **Max Thrust:** ~10-12 kg
- **Thrust-to-Weight Ratio:** ~10x

## Cấu trúc Project

```
s500-fc-sim/
├── urdf/                 # Robot URDF model
├── launch/              # ROS2 launch files
├── worlds/              # Gazebo world files
├── config/              # Configuration files
├── src/
│   └── flight_controller/    # Flight controller ROS2 package
└── models/              # Custom Gazebo models
```

## Yêu cầu

- ROS2 Humble
- Gazebo Humble
- C++17 compiler
- Build tools: `colcon`, `cmake`

## Setup

### 1. Tạo ROS2 Workspace

```bash
mkdir -p ~/s500_ws/src
cd ~/s500_ws/src
git clone https://github.com/KenjiBright/s500-fc-sim.git
cd ~/s500_ws
```

### 2. Build Project

```bash
colcon build
source install/setup.bash
```

### 3. Chạy Simulation

```bash
ros2 launch flight_controller gazebo_sim.launch.py
```

## Tính năng

- [x] URDF model S500 quadcopter
- [x] Gazebo physics simulation
- [x] IMU sensor simulation
- [x] Motor/ESC simulation
- [ ] PID Flight Controller
- [ ] RC Command Input
- [ ] UART Interface
- [ ] PID Auto-tuning

## Kiến trúc Hệ thống

```
┌────────────────────┐
│   Gazebo Sim       │
│  (Sensor Data)     │
└────────┬───────────┘
         │ ROS2 Topics
         │
┌────────▼───────────┐
│  Flight Control    │
│     Node           │
│  (PID Control)     │
└────────┬───────────┘
         │ ROS2 Topics
         │
┌────────▼───────────┐
│  Motor/ESC         │
│     Sim            │
└────────────────────┘
```

## Tham khảo

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/)
- [Betaflight PID Tuning](https://github.com/betaflight/betaflight/wiki)

---

**Tác giả:** KenjiBright
**Ngày tạo:** 2026-05-31
