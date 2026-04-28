# FAHIM_Autonomous_Mobile_Robot
Autonomous Mobile Robot Using Multi-Sensor Fusion and Deep Learning for Dynamic Object Avoidance. Made it by Salman Rahban and Ali Khuzam

FAHIM is a ROS 2 (Humble) differential-drive robot running on a **Jetson Orin Nano** with an **Arduino Nano** motor controller. It uses LiDAR scan-matching for odometry and RTAB-Map for 2D SLAM and localization, with Nav2 for autonomous navigation and YOLOv11 for real-time object detection.

---

## Hardware

| Component | Details |
|---|---|
| Compute | Jetson Orin Nano |
| Motor controller | Arduino Nano + BTS7960 drivers |
| Drive | 2× JGB37-520 motors, differential drive |
| LiDAR | RPLidar A2 M12 (`/dev/rplidar`) |
| Camera | Intel RealSense D435i |
| IMU | MPU6050 on I2C bus 7 |
| Caster | Front passive caster |

**Chassis:** 278 × 176 × 156 mm | **Wheel radius:** 42.5 mm | **Wheel separation:** 235 mm

---

## Package Overview

| Package | Purpose |
|---|---|
| `my_robot_description` | URDF/Xacro model, RViz config, sensor frames |
| `my_robot_hardware` | Arduino bridge (motors/encoders) + MPU6050 IMU node |
| `my_robot_bringup` | Launch files, EKF/Nav2/RTAB-Map configs |
| `my_robot_vision` | YOLOv11 object detection node |

---

## Architecture

```
RPLidar ──→ /scan ──→ rf2o_laser_odometry ──→ /odom_lidar ─┐
                                                             ├──→ EKF ──→ /odometry/filtered ──→ RTAB-Map
Arduino encoders ──→ /odom ─────────────────────────────────┘            (map → odom TF)
                                                                                │
                                                                                ▼
                                                                             Nav2
                                                                                │
                                                              /goal_pose ──────→ bt_navigator
                                                                                │
                                                              controller_server ──→ /cmd_vel_smoothed
                                                                                │
                                                              velocity_smoother ──→ /cmd_vel ──→ Arduino

RealSense D435i ──→ /camera/color + /camera/aligned_depth ──→ yolo_node ──→ /camera/yolo/*
```

**Odometry strategy:** rf2o LiDAR scan-matching is the primary odometry source, providing absolute x, y, yaw at ~8 Hz. Wheel odometry (`/odom`) is available but currently disabled in the EKF because the Arduino publishes at only ~5 Hz with gaps up to 1.2 s — re-enable once that serial issue is fixed.

---

## Prerequisites

### Install once on the Jetson

```bash
# Core ROS 2 dependencies
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-nav2-bringup \
  ros-humble-rplidar-ros \
  ros-humble-realsense2-camera \
  python3-smbus2 \
  python3-serial

# YOLOv11
pip3 install ultralytics --break-system-packages

# rf2o LiDAR odometry (build from source)
cd ~/FAHIM/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/FAHIM && colcon build --packages-select rf2o_laser_odometry
```

### Device symlinks

Create udev rules so device names are stable across reboots:

```bash
# /dev/arduino  →  Arduino Nano USB serial
# /dev/rplidar  →  RPLidar A2
```

### I2C permissions (MPU6050)

```bash
sudo usermod -aG i2c $USER   # logout/login after this
```

---

## Build

```bash
cd ~/FAHIM
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Robot

Source the workspace in every terminal before running anything:

```bash
source ~/FAHIM/install/setup.bash
```

---

### Terminal 1 — Hardware Bringup (always required)

Starts: `robot_state_publisher`, `joint_state_publisher`, `arduino_bridge`, RPLidar, rf2o LiDAR odometry, RealSense D435i, and the EKF.

```bash
ros2 launch my_robot_bringup robot.launch.py
```

Optional arguments:

```bash
ros2 launch my_robot_bringup robot.launch.py \
  use_ekf:=true \
  use_lidar:=true \
  use_camera:=true
```

---

### Terminal 2A — SLAM (build a new map)

```bash
ros2 launch my_robot_bringup slam.launch.py
```

Useful variants:

```bash
# Start fresh — wipe the existing RTAB-Map database
ros2 launch my_robot_bringup slam.launch.py delete_db:=true

# Open the RTAB-Map GUI alongside SLAM
ros2 launch my_robot_bringup slam.launch.py use_rtabmap_viz:=true

# Custom database location
ros2 launch my_robot_bringup slam.launch.py database_path:=/path/to/my.db
```

**Save the map** while SLAM is active (run in a separate terminal):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/FAHIM/fahim_map \
  --ros-args -p map_subscribe_transient_local:=true
```

This creates `fahim_map.yaml` + `fahim_map.pgm` in `~/FAHIM/`.

---

### Terminal 2B — Localization (use an existing map)

Use this **instead of** Terminal 2A when you already have a saved map and want the robot to navigate a known environment.

```bash
ros2 launch my_robot_bringup slam.launch.py localization:=true
```

---

### Terminal 3 — Autonomous Navigation (Nav2)

Requires **Terminal 1** (hardware) + **Terminal 2B** (localization) to already be running.

```bash
ros2 launch my_robot_bringup nav2.launch.py
```

With a custom map:

```bash
ros2 launch my_robot_bringup nav2.launch.py map_path:=/path/to/your_map.yaml
```

**Send a navigation goal from the command line:**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

Or use the **2D Nav Goal** tool in RViz (click + drag to set position and heading).

---

### Terminal 4 — RViz Visualization

```bash
ros2 launch my_robot_bringup rviz.launch.py
```

The RViz config shows: robot model, LiDAR scan, static map, global/local costmaps, global/local planned paths, robot footprint, YOLO annotated image, YOLO 3D markers, and filtered odometry arrows.

---

### Terminal 5 — Object Detection (YOLO)

Requires the camera running (Terminal 1 with `use_camera:=true`). The model is auto-downloaded on first run.

```bash
ros2 launch my_robot_vision yolo.launch.py
```

With custom options:

```bash
ros2 launch my_robot_vision yolo.launch.py \
  model:=yolo11s.pt \
  conf_threshold:=0.5 \
  iou_threshold:=0.45 \
  process_every_n:=3 \
  max_depth_m:=4.0 \
  device:=cuda:0
```

**YOLO output topics:**

| Topic | Type | Description |
|---|---|---|
| `/camera/yolo/image_annotated/compressed` | `CompressedImage` | JPEG frames with bounding boxes |
| `/camera/yolo/detections` | `Detection2DArray` | 2D bounding boxes + class + confidence |
| `/camera/yolo/detections_3d` | `Detection3DArray` | 3D poses in `camera_color_optical_frame` |
| `/camera/yolo/markers` | `MarkerArray` | RViz spheres + distance labels |

In RViz, add an **Image** display pointed at `/camera/yolo/image_annotated/compressed` to see annotated frames.

---

### Optional — IMU Node (MPU6050)

The IMU node is not launched by default. Run manually if you want raw IMU data:

```bash
ros2 run my_robot_hardware mpu6050_node \
  --ros-args -p i2c_bus:=7 -p rate_hz:=50.0
```

Keep the robot perfectly still during the ~2.5 s gyro calibration at startup.

---

## Typical 3-Terminal Workflow

```
T1: ros2 launch my_robot_bringup robot.launch.py
T2: ros2 launch my_robot_bringup slam.launch.py localization:=true
T3: ros2 launch my_robot_bringup nav2.launch.py
```

Then optionally in T4/T5:

```
T4: ros2 launch my_robot_bringup rviz.launch.py
T5: ros2 launch my_robot_vision yolo.launch.py
```

---

## Useful Diagnostic Commands

```bash
# Verify the TF tree is complete (map → odom → base_footprint → base_link → sensors)
ros2 run tf2_tools view_frames

# Monitor filtered odometry
ros2 topic echo /odometry/filtered

# Check LiDAR odometry rate (should be ~8-10 Hz)
ros2 topic hz /odom_lidar

# Check Arduino odometry rate (should be ~50 Hz; investigate if lower)
ros2 topic hz /odom

# List all active nodes
ros2 node list

# Check EKF diagnostics
ros2 topic echo /diagnostics

# Inspect Nav2 lifecycle state
ros2 lifecycle get /bt_navigator

# Manual motor test (drive forward 0.2 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

---

## Tuning Guide

### EKF (`config/ekf.yaml`)

- `sensor_timeout` — set to 1.5 s to accommodate rf2o at ~8 Hz; lower once Arduino rate is fixed
- `odom0_config` — the 15-element boolean array selects which states (x, y, yaw, vx, vyaw) are fused from `/odom_lidar`
- To re-enable wheel odometry, add an `odom1: /odom` block with appropriate config once the Arduino serial issue is resolved

### Nav2 (`config/nav2_params.yaml`)

- `desired_linear_vel` — cruise speed (default 0.4 m/s, max 0.5 m/s)
- `rotate_to_heading_min_angle` — rotate in-place before driving if heading error exceeds this (default 45°)
- `rotate_to_heading_angular_vel` — in-place rotation speed (default 1.0 rad/s); raise if motors stall
- `xy_goal_tolerance` / `yaw_goal_tolerance` — how close is "close enough" to the goal
- `inflation_radius` — obstacle padding radius (default 0.30 m); lower for tighter spaces

### Arduino Bridge (`robot.launch.py` wheel_params)

- `left_motor_scale` / `right_motor_scale` — correct straight-line drift (robot veers right → lower `left_motor_scale`)
- `min_motor_rpm` — minimum RPM to overcome motor stiction (default 50); raise if in-place rotation stalls
- `left_odom_scale` / `right_odom_scale` — correct measured distance error (if robot reports 1.03 m for 1 m actual, set scale to 0.971)

### YOLO (`yolo.launch.py`)

- `process_every_n` — skip frames to reduce GPU load; 2 = process every other frame
- `publish_width` — downscale annotated image before JPEG encoding (default 640 px wide)
- `model` — swap to `yolo11s.pt` or `yolo11m.pt` for better accuracy at the cost of latency

---

## Key Topics Reference

| Topic | Direction | Description |
|---|---|---|
| `/scan` | published | RPLidar raw laser scan |
| `/odom` | published | Wheel encoder odometry (Arduino) |
| `/odom_lidar` | published | LiDAR scan-matching odometry (rf2o) |
| `/odometry/filtered` | published | EKF fused odometry |
| `/cmd_vel` | subscribed | Motor velocity commands |
| `/cmd_vel_smoothed` | internal | Between Nav2 controller and velocity smoother |
| `/map` | published | RTAB-Map occupancy grid |
| `/robot_description` | published | URDF for RViz and TF |
| `/camera/color/image_raw` | published | RealSense RGB stream |
| `/camera/aligned_depth_to_color/image_raw` | published | RealSense aligned depth |
| `/goal_pose` | subscribed | Nav2 navigation goal (from RViz or CLI) |
| `/plan` | published | Global path |
| `/local_plan` | published | Local trajectory |
| `/imu/data_raw` | published | MPU6050 raw IMU (if node running) |

---

