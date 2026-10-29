#!/usr/bin/env python3
"""Live view of the GPS accuracy, in metres, for demos.

Once per second, prints the distance between the true antenna position
(Gazebo ground truth) and:
  - the standalone GPS  (/gps/raw)
  - the RTK GPS         (/gps/fix, the position used by the robot)

  ros2 run agri_ugv_description gps_monitor.py
  ros2 param set /rtk_gnss rtk_enabled false   # in another terminal: RTK off
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformListener

ORIGIN_LAT, ORIGIN_LON = 34.74, 10.76
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3
_s = math.sin(math.radians(ORIGIN_LAT))
_w = math.sqrt(1.0 - WGS84_E2 * _s * _s)
M_N = math.radians(1.0) * WGS84_A * (1.0 - WGS84_E2) / _w ** 3
M_E = math.radians(1.0) * WGS84_A / _w * math.cos(math.radians(ORIGIN_LAT))


def rotate(q, v):
    u = np.array([q.x, q.y, q.z])
    return v + 2 * np.cross(u, np.cross(u, v) + q.w * v)


def fmt(d):
    return f'{100 * d:6.1f} cm' if d < 1.0 else f'{d:6.2f} m '


class GpsMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.antenna = None
        self.gt = self.raw = self.fix = None
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(NavSatFix, '/gps/raw', lambda m: setattr(self, 'raw', m), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/fix', lambda m: setattr(self, 'fix', m), qos_profile_sensor_data)
        self.create_timer(1.0, self.show)
        print('  standalone GPS | RTK GPS (/gps/fix) | mode')

    def error(self, f):
        p = self.gt.pose.pose
        true = np.array([p.position.x, p.position.y]) + rotate(p.orientation, self.antenna)[:2]
        meas = np.array([(f.longitude - ORIGIN_LON) * M_E, (f.latitude - ORIGIN_LAT) * M_N])
        return float(np.linalg.norm(meas - true))

    def show(self):
        if self.antenna is None:
            if not self.tf.can_transform('base_footprint', 'gps_link', rclpy.time.Time()):
                return
            t = self.tf.lookup_transform('base_footprint', 'gps_link', rclpy.time.Time()).transform.translation
            self.antenna = np.array([t.x, t.y, t.z])
        if self.gt is None or self.raw is None or self.fix is None:
            print('  waiting for /ground_truth/odom, /gps/raw, /gps/fix ...')
            return
        mode = 'RTK' if self.fix.status.status == NavSatStatus.STATUS_GBAS_FIX else 'standalone (RTK off)'
        print(f'  {fmt(self.error(self.raw))}      | {fmt(self.error(self.fix))}          | {mode}', flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(GpsMonitor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
