#!/usr/bin/env python3
"""RTK GNSS: robot GPS corrected by a base station on a known point.

Gazebo's NavSat sensors give the true antenna positions plus receiver noise.
A real GPS also suffers from satellite and atmosphere errors of a few metres,
which change slowly and are the same for receivers a few km apart. This node
models that error and the RTK correction that removes it:

  atmosphere   e(t): common error (metres, East/North/Up), slow and smooth
  base station  measures  known point + e(t)  ->  correction = measured - known
                (sent at 1 Hz, like an RTK radio link)
  robot         measures  true position + e(t)  ->  minus latest correction

Topics:
  in   /gps/navsat         robot receiver (Gazebo)
  in   /rtk_base/navsat    base station receiver (Gazebo)
  out  /gps/raw            standalone GPS: true + e(t) + code noise (~2 m error)
  out  /rtk_base/corrections   geometry_msgs/Vector3Stamped, metres E/N/U
  out  /gps/fix            RTK position (~2 cm), status GBAS_FIX; falls back to
                           the standalone position (status FIX) when RTK is off
                           or the corrections are older than max_correction_age

Runtime switch (demo):  ros2 param set /rtk_gnss rtk_enabled false
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def metres_per_degree(lat_deg):
    """(north, east) metres per degree of latitude / longitude on WGS84."""
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    m = WGS84_A * (1.0 - WGS84_E2) / w ** 3       # meridian radius
    n = WGS84_A / w                                # prime vertical radius
    return math.radians(1.0) * m, math.radians(1.0) * n * math.cos(math.radians(lat_deg))


class Atmosphere:
    """Common GNSS error e(t) in metres (E, N, U): a bias plus slow sinusoids
    (periods of 5-20 min). Smooth, like real ionosphere/troposphere/orbit errors."""

    def __init__(self, seed):
        rng = np.random.default_rng(seed)
        a = rng.uniform(-math.pi, math.pi)
        mag = rng.uniform(1.5, 2.0)
        self.bias = np.array([mag * math.cos(a), mag * math.sin(a), rng.uniform(2.0, 3.0)])
        self.amp = np.vstack([rng.uniform(0.15, 0.3, (2, 3)), rng.uniform(0.5, 1.0, (1, 3))])
        self.period = rng.uniform(300.0, 1200.0, (3, 3))
        self.phase = rng.uniform(0.0, 2 * math.pi, (3, 3))

    def error(self, t):
        return self.bias + np.sum(self.amp * np.sin(2 * math.pi * t / self.period + self.phase), axis=1)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class RtkGnss(Node):
    def __init__(self):
        super().__init__('rtk_gnss')
        self.declare_parameter('rtk_enabled', True)
        self.code_noise = self.declare_parameter('standalone_noise', 0.3).value    # m, 1-sigma
        self.max_age = self.declare_parameter('max_correction_age', 5.0).value     # s
        self.corr_period = self.declare_parameter('correction_period', 1.0).value  # s
        self.survey_n = self.declare_parameter('survey_samples', 30).value
        self.atmo = Atmosphere(self.declare_parameter('seed', 3).value)
        self.rng = np.random.default_rng()

        self.known = None          # base station known coordinates (lat, lon, alt)
        self.survey = []
        self.corr = None           # latest correction, metres E/N/U
        self.corr_time = None
        self.last_corr_sent = -1e9

        self.raw_pub = self.create_publisher(NavSatFix, '/gps/raw', qos_profile_sensor_data)
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
        self.corr_pub = self.create_publisher(Vector3Stamped, '/rtk_base/corrections', 10)
        self.create_subscription(NavSatFix, '/rtk_base/navsat', self.on_base, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/navsat', self.on_robot, qos_profile_sensor_data)

    @staticmethod
    def shifted(fix, enu):
        """Copy of a NavSatFix moved by enu metres."""
        m_n, m_e = metres_per_degree(fix.latitude)
        out = NavSatFix()
        out.header = fix.header
        out.latitude = fix.latitude + enu[1] / m_n
        out.longitude = fix.longitude + enu[0] / m_e
        out.altitude = fix.altitude + enu[2]
        return out

    # ---------------------------------------------------------------- base station
    def on_base(self, msg):
        if self.known is None:
            # survey-in: the base sits on a point with known coordinates; averaging
            # its receiver for a few seconds gives them to the millimetre
            self.survey.append((msg.latitude, msg.longitude, msg.altitude))
            if len(self.survey) >= self.survey_n:
                self.known = np.mean(self.survey, axis=0)
                self.get_logger().info(
                    f'RTK base station on known point lat {self.known[0]:.8f} '
                    f'lon {self.known[1]:.8f} alt {self.known[2]:.3f} m')
            return
        t = stamp_s(msg)
        if t - self.last_corr_sent < self.corr_period:
            return
        self.last_corr_sent = t
        measured = self.shifted(msg, self.atmo.error(t))
        m_n, m_e = metres_per_degree(self.known[0])
        self.corr = np.array([(measured.longitude - self.known[1]) * m_e,
                              (measured.latitude - self.known[0]) * m_n,
                              measured.altitude - self.known[2]])
        self.corr_time = t
        c = Vector3Stamped()
        c.header = msg.header
        c.vector.x, c.vector.y, c.vector.z = (float(v) for v in self.corr)
        self.corr_pub.publish(c)

    # ---------------------------------------------------------------- robot receiver
    def on_robot(self, msg):
        t = stamp_s(msg)
        e = self.atmo.error(t)

        noise = np.r_[self.rng.normal(0.0, self.code_noise, 2), self.rng.normal(0.0, 2 * self.code_noise)]
        raw = self.shifted(msg, e + noise)
        raw.status.status = NavSatStatus.STATUS_FIX
        raw.status.service = NavSatStatus.SERVICE_GPS
        sh = math.hypot(np.linalg.norm(self.atmo.bias[:2]), self.code_noise)
        raw.position_covariance = [sh ** 2, 0.0, 0.0, 0.0, sh ** 2, 0.0, 0.0, 0.0, (2 * sh) ** 2]
        raw.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.raw_pub.publish(raw)

        rtk_ok = (self.get_parameter('rtk_enabled').value and self.corr is not None
                  and t - self.corr_time <= self.max_age)
        if not rtk_ok:
            self.fix_pub.publish(raw)
            return
        fix = self.shifted(msg, e - self.corr)
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance = [0.02 ** 2, 0.0, 0.0, 0.0, 0.02 ** 2, 0.0, 0.0, 0.0, 0.04 ** 2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(fix)


def main():
    rclpy.init()
    try:
        rclpy.spin(RtkGnss())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # normal stop (Ctrl+C / launch shutdown)


if __name__ == '__main__':
    main()
