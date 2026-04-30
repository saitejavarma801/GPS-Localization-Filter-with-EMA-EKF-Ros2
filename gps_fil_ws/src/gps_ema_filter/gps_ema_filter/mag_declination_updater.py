#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import NavSatFix
import geomag


class MagDeclinationUpdater(Node):
    """
    One-shot node:
    - Waits for first valid GPS fix (/gps/validated)
    - Computes magnetic declination using WMM (geomag)
    - Updates navsat nodes dynamically
    - Shuts down cleanly
    """

    NAVSAT_NODES = ['/navsat_raw', '/navsat_filtered']
    PARAM_NAME = 'magnetic_declination_radians'

    def __init__(self):
        super().__init__('mag_declination_updater')

        self._done = False

        self._sub = self.create_subscription(
            NavSatFix,
            '/gps/validated',   #use filtered GPS
            self._gps_callback,
            10
        )

        self.get_logger().info(
            "Waiting for valid GPS fix to compute magnetic declination..."
        )

    # ------------------------------------------------------------------
    def _gps_callback(self, msg: NavSatFix):

        if self._done:
            return

        # Wait for valid fix
        if msg.status.status < 0:
            return

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        # ---------------- COMPUTE DECLINATION ----------------
        try:
            decl_deg = geomag.declination(lat, lon, alt)
        except Exception as e:
            self.get_logger().error(f"geomag failed: {e} → using 0.0")
            decl_deg = 0.0

        decl_rad = math.radians(decl_deg)

        self.get_logger().info(
            f" GPS Fix: lat={lat:.5f}, lon={lon:.5f}, alt={alt:.1f} m"
        )
        self.get_logger().info(
            f" Declination: {decl_deg:+.4f}° → {decl_rad:+.6f} rad"
        )

        # ---------------- APPLY TO NAVSAT NODES ----------------
        all_ok = True

        for node_name in self.NAVSAT_NODES:
            ok = self._set_param(node_name, decl_rad)
            all_ok = all_ok and ok

        if all_ok:
            self.get_logger().info(" All navsat nodes updated successfully.")
        else:
            self.get_logger().warn(
                " Some navsat nodes not reachable — using default (0.0). "
                "Ensure navsat nodes started before this node."
            )

        # Done → shutdown
        self._done = True
        self.get_logger().info(" Shutting down declination updater node...")
        rclpy.shutdown()

    # ------------------------------------------------------------------
    def _set_param(self, node_name: str, value: float) -> bool:

        svc_name = f'{node_name}/set_parameters'
        client = self.create_client(SetParameters, svc_name)

        self.get_logger().info(f" Connecting to {svc_name} ...")

        #  Retry logic (important fix)
        for _ in range(5):
            if client.wait_for_service(timeout_sec=2.0):
                break
        else:
            self.get_logger().warn(f" Service unavailable: {svc_name}")
            return False

        req = SetParameters.Request()

        param = Parameter(
            self.PARAM_NAME,
            Parameter.Type.DOUBLE,
            value
        )

        req.parameters = [param.to_parameter_msg()]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().warn(f" No response from {svc_name}")
            return False

        result = future.result().results[0]

        if result.successful:
            self.get_logger().info(
                f" {node_name}: {self.PARAM_NAME} = {value:+.6f} rad"
            )
            return True
        else:
            self.get_logger().warn(
                f" {node_name} rejected parameter: {result.reason}"
            )
            return False


# -----------------------------------------------------------------------
def main():
    rclpy.init()
    node = MagDeclinationUpdater()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
