#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math


class GPSMotionFilter(Node):

    def __init__(self):
        super().__init__('gps_motion_filter')

        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.prev_speed = 0.0
        self.prev_heading = None
        self.odom_speed = 0.0
        self.received_odom = False

        self.create_subscription(NavSatFix, '/gps/filtered', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(NavSatFix, '/gps/validated', 10)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_speed = math.sqrt(vx*vx + vy*vy)
        self.received_odom = True

    def gps_callback(self, msg):

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt <= 0:
                return

            dy = (msg.latitude - self.prev_lat) * 111000
            dx = (msg.longitude - self.prev_lon) * 111000 * math.cos(math.radians(msg.latitude))

            speed = math.sqrt(dx*dx + dy*dy) / dt
            heading = math.atan2(dy, dx)

            if speed > 5.0:
                return

            if self.received_odom and self.odom_speed < 0.05 and speed > 0.5:
                return

            acc = abs(speed - self.prev_speed) / dt
            if acc > 2.0:
                return

            if self.prev_heading is not None:
                diff = (heading - self.prev_heading + math.pi) % (2 * math.pi) - math.pi
                if abs(diff) > math.radians(45):
                    return

            self.prev_speed = speed
            self.prev_heading = heading

        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
        self.prev_time = now

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GPSMotionFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
