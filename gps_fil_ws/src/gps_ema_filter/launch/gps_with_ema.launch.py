#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    return LaunchDescription([

        # ---------------- GPS ----------------
        Node(
            package='gps_reader',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{'port': '/dev/ttyTHS1', 'baudrate': 9600}]  #  FIXED
        ),

        # ---------------- EMA FILTER ----------------
        Node(
            package='gps_ema_filter',
            executable='ema_filter_node',
            name='gps_ema_filter',
            output='screen'
        ),

        # ---------------- MOTION FILTER ----------------
        Node(
            package='gps_ema_filter',
            executable='motion_filter_node',
            name='gps_motion_filter',
            output='screen'
        ),

        # ---------------- HEADING KALMAN ----------------
        Node(
            package='gps_ema_filter',
            executable='heading_kalman_fusion',
            name='heading_kalman_fusion',
            output='screen'
        ),
	# ----------------MAG_DECLINATION_UPDATER----------
	Node(
    	    package='gps_ema_filter',
    	    executable='mag_declination_updater',
    	    name='mag_declination_updater',
    	    output='screen'
	),
        # =========================================================
        #  RAW GPS NAVSAT (ANALYSIS ONLY)
        # =========================================================
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_raw',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'delay': 3.0,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'broadcast_utm_transform': False,
                'publish_filtered_gps': False,
                'use_odometry_yaw': False,
                'wait_for_datum': False
            }],
            remappings=[
                ('gps/fix', '/fix'),
                ('imu/data', '/imu/fused'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/raw_gps/odom')
            ]
        ),

        # =========================================================
        #  FILTERED GPS NAVSAT (USED BY EKF)
        # =========================================================
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_filtered',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'delay': 3.0,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'broadcast_utm_transform': False,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'wait_for_datum': False
            }],
            remappings=[
                ('gps/fix', '/gps/validated'),
                ('imu/data', '/imu/fused'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/filtered_gps/odom')
            ]
        ),

        # =========================================================
        #  EKF 
        # =========================================================
        TimerAction(
            period=5.0,  # FIXED (wait for navsat)
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=['/home/orin/gps_filter_ws/src/gps_ema_filter/config/ekf.yaml']
                )
            ]
        ),
    ])
