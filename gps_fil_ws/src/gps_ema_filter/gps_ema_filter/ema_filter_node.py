#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json
import math


class GPS_EMA_Filter(Node):

    def __init__(self):
        super().__init__('gps_ema_filter')

        self.filtered_lat = None
        self.filtered_lon = None
        self.filtered_alt = None

        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.last_speed = 0.0

        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.filtered_pub = self.create_publisher(NavSatFix, '/gps/filtered', 10)
        self.status_pub = self.create_publisher(String, '/localization/status', 10)

        self.get_logger().info("Advanced GPS EMA Filter Started")

    def gps_callback(self, msg):

        if msg.status.status < 0:
            return

        # HDOP estimation
        if msg.position_covariance_type == 1:
            variance = msg.position_covariance[0]
            hdop = math.sqrt(variance)
        else:
            hdop = 10.0

        now = self.get_clock().now().nanoseconds * 1e-9

        # Speed estimation
        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                dy = (msg.latitude - self.prev_lat) * 111000
                dx = (msg.longitude - self.prev_lon) * 111000 * math.cos(math.radians(msg.latitude))
                self.last_speed = math.sqrt(dx*dx + dy*dy) / dt

        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
        self.prev_time = now

        # Adaptive alpha
        if hdop > 3:
            alpha = 0.05
        elif self.last_speed < 0.2:
            alpha = 0.1
        else:
            alpha = 0.4

        if self.filtered_lat is None:
            self.filtered_lat = msg.latitude
            self.filtered_lon = msg.longitude
            self.filtered_alt = msg.altitude
        else:
            self.filtered_lat = alpha * msg.latitude + (1-alpha) * self.filtered_lat
            self.filtered_lon = alpha * msg.longitude + (1-alpha) * self.filtered_lon
            self.filtered_alt = alpha * msg.altitude + (1-alpha) * self.filtered_alt

        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude = self.filtered_lat
        out.longitude = self.filtered_lon
        out.altitude = self.filtered_alt
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type

        self.filtered_pub.publish(out)

        # Status debug
        status = {
            "hdop": round(hdop, 2),
            "speed": round(self.last_speed, 2)
        }
        s = String()
        s.data = json.dumps(status)
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = GPS_EMA_Filter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
