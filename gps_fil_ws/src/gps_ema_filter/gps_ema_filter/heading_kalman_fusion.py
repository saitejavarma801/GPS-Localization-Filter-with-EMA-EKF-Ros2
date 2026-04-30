#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import math


class HeadingKalman(Node):

    def __init__(self):
        super().__init__('heading_kalman_fusion')

        self.x = 0.0
        self.P = 1.0

        self.Q = 0.005
        self.R_gps = 1.5
        self.R_imu = 0.05

        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None

        self.last_imu = None

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/validated', self.gps_callback, 10)

        self.pub = self.create_publisher(Imu, '/imu/fused', 10)

    def imu_callback(self, msg):

        q = msg.orientation

        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        imu_yaw = math.atan2(siny, cosy)

        # Prediction
        self.P += self.Q

        # Update (IMU)
        K = self.P / (self.P + self.R_imu)
        diff = (imu_yaw - self.x + math.pi) % (2 * math.pi) - math.pi

        self.x = self.x + K * diff
        self.P = (1 - K) * self.P

        self.last_imu = msg
        self.publish_fused()

    def gps_callback(self, msg):

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is None:
            self.prev_lat = msg.latitude
            self.prev_lon = msg.longitude
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0:
            return

        dy = (msg.latitude - self.prev_lat) * 111000
        dx = (msg.longitude - self.prev_lon) * 111000 * math.cos(math.radians(msg.latitude))

        speed = math.sqrt(dx*dx + dy*dy) / dt

        if speed > 0.5:
            gps_heading = math.atan2(dy, dx)

            K = self.P / (self.P + self.R_gps)

            innovation = (gps_heading - self.x + math.pi) % (2 * math.pi) - math.pi
            self.x = self.x + K * innovation
            self.P = (1 - K) * self.P

        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
        self.prev_time = now

    def publish_fused(self):

        if self.last_imu is None:
            return

        yaw = self.x

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        fused = Imu()
        fused.header = self.last_imu.header
        fused.orientation.z = qz
        fused.orientation.w = qw
        fused.angular_velocity = self.last_imu.angular_velocity
        fused.linear_acceleration = self.last_imu.linear_acceleration

        self.pub.publish(fused)


def main():
    rclpy.init()
    node = HeadingKalman()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
