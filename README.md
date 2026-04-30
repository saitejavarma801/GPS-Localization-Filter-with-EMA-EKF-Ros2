# GPS-Localization-Filter-with-EMA-EKF-Ros2
A robust multi-stage GPS filtering and sensor fusion pipeline for outdoor autonomous navigation. This package combines adaptive EMA smoothing, motion consistency validation, and EKF-based sensor fusion to produce reliable position and heading estimates for robotic systems.

# Overview

Raw GPS signals are inherently noisy, prone to spikes, and unreliable at low speeds.

This package improves GPS reliability through a layered filtering architecture:

  1.Noise reduction using adaptive EMA
  
  2.Outlier rejection using motion consistency checks
  
  3.Accurate heading estimation using EKF fusion (IMU + GPS)
  
  4.Automatic magnetic declination correction



# Processing Pipeline

    /fix (raw GPS)
           └─► [ema_filter_node]             → /gps/filtered
           └─► [motion_filter_node]          → /gps/validated
              ├─► [ekf_fusion_node] + /imu   → /imu/fused
              └─► [mag_declination_updater]

----
# System Architecture

The system is structured into modular ROS 2 nodes, each responsible for a specific stage of filtering and validation.

# Nodes
# Ema filter Node
Subscribes to raw /fix and applies an adaptive EMA (Exponential Moving Average) filter.

Alpha adapts based on HDOP and estimated speed

1.High HDOP → α = 0.05 (heavy smoothing)

2.Stationary → α = 0.1

3.Moving → α = 0.4


4.Publishes smoothed fix to /gps/filtered

5.Publishes HDOP + speed debug info to /localization/status

----
# Motion Filter Node
Validates EMA-filtered GPS using motion consistency checks.

1.Rejects fixes with speed > 5 m/s (outlier spike)

2.Rejects fixes where GPS shows motion but odometry (/odom) says robot is stationary

3.Rejects fixes with acceleration > 2 m/s²

4.Rejects fixes with heading change > 45° between consecutive samples

5.Publishes validated fixes to /gps/validated

# Heading Kalman Fusion
Fuses IMU yaw and GPS-derived heading using a 1D Kalman filter.

1.Prediction step driven by IMU at high rate
2.GPS update applied only when speed > 0.5 m/s (heading is unreliable at rest)
3.Handles angle wraparound correctly using (angle + π) % 2π - π
4.Publishes fused IMU to /imu/fused

# Mag Declination Updater
One-shot node that auto-computes and applies magnetic declination.

1.Waits for first valid /gps/validated fix
2.Uses the geomag WMM library to compute declination at the robot's location
3.Calls set_parameters service on /navsat_raw and /navsat_filtered nodes
4.Shuts down cleanly after applying


Topics
TopicTypeDirection/fixsensor_msgs/NavSatFixInput (raw GPS)/imusensor_msgs/ImuInput (raw IMU)/odomnav_msgs/OdometryInput (wheel odometry)/gps/filteredsensor_msgs/NavSatFixEMA output/gps/validatedsensor_msgs/NavSatFixMotion-validated output/imu/fusedsensor_msgs/ImuHeading-fused IMU output/localization/statusstd_msgs/StringDebug JSON (HDOP, speed)




# Installation
     git clone https://github.com/saitejavarma/gps_fil_Ws

     cd gps_fil_ws
     colcon build
     source install/setup.bash


# Launch
    ros2 launch gps_ema_filter gps_with_ema.launch.py





# Key Advantages


1.Robust against GPS noise and spikes


2.Reliable performance at low speeds


3.Improved heading accuracy through sensor fusion


4.Modular and lightweight design





# Part of
OutNav — a ROS 2-based outdoor autonomous navigation system for Mobile Robot.


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

