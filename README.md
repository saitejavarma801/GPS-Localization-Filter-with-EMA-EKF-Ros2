# GPS EMA Filter — ROS2 Package

A ROS2 package implementing a multi-stage GPS filtering and heading fusion pipeline for robust outdoor autonomous navigation.

---

## Overview

Raw GPS signals are noisy, jumpy, and unreliable at low speeds. This package chains four nodes to produce clean, validated position and fused heading estimates suitable for navigation stacks like Nav2 or robot_localization EKF.

```
/fix  (raw GPS)
  └─► [ema_filter_node]       → /gps/filtered
        └─► [motion_filter_node]  → /gps/validated
              ├─► [heading_kalman_fusion] + /imu → /imu/fused
              └─► [mag_declination_updater]      → sets param on navsat nodes
```

---

## Nodes

### `ema_filter_node`
Subscribes to raw `/fix` and applies an **adaptive EMA (Exponential Moving Average)** filter.

- Alpha adapts based on HDOP and estimated speed
  - High HDOP → α = 0.05 (heavy smoothing)
  - Stationary → α = 0.1
  - Moving → α = 0.4
- Publishes smoothed fix to `/gps/filtered`
- Publishes HDOP + speed debug info to `/localization/status`

### `motion_filter_node`
Validates EMA-filtered GPS using **motion consistency checks**.

- Rejects fixes with speed > 5 m/s (outlier spike)
- Rejects fixes where GPS shows motion but odometry (`/odom`) says robot is stationary
- Rejects fixes with acceleration > 2 m/s²
- Rejects fixes with heading change > 45° between consecutive samples
- Publishes validated fixes to `/gps/validated`

### `heading_kalman_fusion`
Fuses **IMU yaw** and **GPS-derived heading** using a 1D Kalman filter.

- Prediction step driven by IMU at high rate
- GPS update applied only when speed > 0.5 m/s (heading is unreliable at rest)
- Handles angle wraparound correctly using `(angle + π) % 2π - π`
- Publishes fused IMU to `/imu/fused`

### `mag_declination_updater`
One-shot node that auto-computes and applies **magnetic declination**.

- Waits for first valid `/gps/validated` fix
- Uses the `geomag` WMM library to compute declination at the robot's location
- Calls `set_parameters` service on `/navsat_raw` and `/navsat_filtered` nodes
- Shuts down cleanly after applying

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/fix` | `sensor_msgs/NavSatFix` | Input (raw GPS) |
| `/imu` | `sensor_msgs/Imu` | Input (raw IMU) |
| `/odom` | `nav_msgs/Odometry` | Input (wheel odometry) |
| `/gps/filtered` | `sensor_msgs/NavSatFix` | EMA output |
| `/gps/validated` | `sensor_msgs/NavSatFix` | Motion-validated output |
| `/imu/fused` | `sensor_msgs/Imu` | Heading-fused IMU output |
| `/localization/status` | `std_msgs/String` | Debug JSON (HDOP, speed) |

---

## Dependencies

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>rcl_interfaces</depend>
```

Python:
```
geomag
```

Install geomag:
```bash
pip install geomag
```

---

## Build & Run

```bash

git clone https://github.com/saitejavarma801/gps_fil_ws

# Build
cd ~/your_ws
colcon build --packages-select gps_ema_filter
source install/setup.bash

# Launch all nodes
ros2 launch gps_ema_filter gps_with_ema.launch.py
```

---


## Part Of

OutNav — a ROS2-based waypoint-driven autonomous outdoor navigation stack.


# MIT License

Copyright (c) 2026 SAI TEJA VARMA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

