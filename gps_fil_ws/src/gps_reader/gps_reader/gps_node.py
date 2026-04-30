#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2
import threading


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 9600)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        self.num_sats = 0
        self.fix_quality = 0
        self.altitude_m = 0.0
        self.hdop = 0.0

        self.heading_deg = 0.0
        self.prev_lat = None
        self.prev_lon = None

        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Opened {port} @ {baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            raise

        # Threaded read loop
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    def _serial_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.read_until(b'\n')
                line = raw.decode("ascii", errors="ignore").strip()

                if not line.startswith("$"):
                    continue

                try:
                    msg = pynmea2.parse(line)
                except:
                    continue

                # ---------------- GGA ----------------
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self.num_sats = int(msg.num_sats or 0)
                    self.fix_quality = int(msg.gps_qual or 0)
                    self.altitude_m = float(msg.altitude or 0.0)
                    self.hdop = float(msg.horizontal_dil or 0.0)

                # ---------------- RMC ----------------
                if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":

                    lat = msg.latitude
                    lon = msg.longitude

                    # Heading
                    if msg.true_course:
                        self.heading_deg = float(msg.true_course)
                    elif self.prev_lat is not None:
                        self.heading_deg = self._bearing(
                            self.prev_lat, self.prev_lon, lat, lon
                        )

                    self.prev_lat = lat
                    self.prev_lon = lon

                    # ---------------- NavSatFix ----------------
                    fix = NavSatFix()
                    fix.header.stamp = self.get_clock().now().to_msg()
                    fix.header.frame_id = "gps"

                    fix.latitude = lat
                    fix.longitude = lon
                    fix.altitude = self.altitude_m

                    # ✅ Always initialize covariance (CRITICAL FIX)
                    fix.position_covariance = [0.0] * 9
                    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    # ✅ Proper fix status
                    if self.fix_quality > 0:
                        fix.status.status = NavSatStatus.STATUS_FIX
                    else:
                        fix.status.status = NavSatStatus.STATUS_NO_FIX

                    fix.status.service = NavSatStatus.SERVICE_GPS

                    # ✅ Improve covariance using HDOP
                    if self.hdop > 0:
                        horizontal_accuracy = self.hdop * 2.5  # meters
                        vertical_accuracy = self.hdop * 4.0    # worse than horizontal

                        fix.position_covariance = [
                            horizontal_accuracy**2, 0.0, 0.0,
                            0.0, horizontal_accuracy**2, 0.0,
                            0.0, 0.0, vertical_accuracy**2
                        ]

                        fix.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                        )

                    self.fix_pub.publish(fix)

            except Exception as e:
                self.get_logger().error(f"Serial loop error: {e}")

    def _bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)

        x = math.sin(dlon) * math.cos(phi2)
        y = (
            math.cos(phi1) * math.sin(phi2)
            - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
        )

        return (math.degrees(math.atan2(x, y)) + 360) % 360


def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
