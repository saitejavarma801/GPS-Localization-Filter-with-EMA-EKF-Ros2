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


# System Architecture

The system is structured into modular ROS 2 nodes, each responsible for a specific stage of filtering and validation.

# Nodes
ema_filter_node — Adaptive GPS Smoothing
Applies an adaptive Exponential Moving Average (EMA) filter to raw GPS data.
# Key Features:


Dynamic smoothing factor (α):


High HDOP → α = 0.05


Stationary → α = 0.1


Moving → α = 0.4




Reduces jitter while preserving responsiveness


Publishes filtered GPS data


# Topics:


Subscribes: /fix


Publishes:


/gps/filtered


/localization/status (HDOP, speed debug info)





# Motion Filter Node — Motion Consistency Validation

Validates filtered GPS data using physical motion constraints.

# Validation Rules:


Rejects speed spikes greater than 5 m/s


Rejects GPS motion when odometry indicates stationary


Rejects acceleration greater than 2 m/s²


Rejects heading changes greater than 45°


# Purpose:


Eliminates GPS outliers and multipath errors


Ensures only reliable data is used downstream


# Topics:


Subscribes: /gps/filtered, /odom


Publishes: /gps/validated



# Ekf fusion_node — Sensor Fusion (IMU + GPS)
Fuses IMU orientation and GPS-derived heading using an Extended Kalman Filter (EKF).
# Key Features:


Combines high-frequency IMU data with GPS corrections


GPS updates applied only when speed is greater than 0.5 m/s


Handles angle wrapping correctly


Produces stable, drift-resistant heading estimates


# Topics:


Subscribes: /imu, /gps/validated


Publishes: /imu/fused



# Mag declination updater — Magnetic Declination Correction
Automatically computes and applies magnetic declination based on location.
# Key Features:


Uses the geomag library (WMM model)


Updates parameters in NavSat nodes


Executes once and exits cleanly




# Installation
     git clone https://github.com/saitejavarma/gps_fil_Ws

     cd gps_fil_ws
     colcon build
     source install/setup.bash


# Launch
    ros2 launch gps_ema_filter gps_with_ema.launch.py





# Key Advantages


Robust against GPS noise and spikes


Reliable performance at low speeds


Improved heading accuracy through sensor fusion


Modular and lightweight design





Part Of
OutdoorNav — a ROS 2-based outdoor autonomous navigation system.


