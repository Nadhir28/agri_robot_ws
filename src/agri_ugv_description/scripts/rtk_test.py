#!/usr/bin/env python3
"""RTK GNSS test for agri_ugv in the olive_orchard world.

Checks (against Gazebo ground truth):
  - base station: its receiver is on the known point of the world file;
    corrections are sent at ~1 Hz and contain the metre-level common error
  - standalone GPS (/gps/raw) is wrong by metres, as a real single receiver
  - RTK GPS (/gps/fix) is within a few centimetres, at rest and while driving,
    with status GBAS_FIX (RTK) and a matching covariance
  - switching RTK off falls back to the standalone position (status FIX),
    switching it on again restores centimetre accuracy

Writes the driving trace (truth, standalone, RTK) to
~/.ros/agri_ugv_tests/rtk_trace.csv for the report figure.

Run with the simulation up:
  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true
  ros2 run agri_ugv_description rtk_test.py
Exit code 0 if every check passes.
"""
import bisect
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformListener

ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = 34.74, 10.76, 20.0
BASE_ANTENNA_H = 1.645           # antenna height above the base station origin (model.sdf)
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def metres_per_degree(lat_deg):
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    return (math.radians(1.0) * WGS84_A * (1.0 - WGS84_E2) / w ** 3,
            math.radians(1.0) * WGS84_A / w * math.cos(math.radians(lat_deg)))


M_N, M_E = metres_per_degree(ORIGIN_LAT)


def fix_to_xyz(f):
    """NavSatFix -> world x (east), y (north), z (up) in metres."""
    return np.array([(f.longitude - ORIGIN_LON) * M_E, (f.latitude - ORIGIN_LAT) * M_N,
                     f.altitude - ORIGIN_ALT])


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rotate(q, v):
    """Rotate vector v by quaternion q (x, y, z, w)."""
    u = np.array([q.x, q.y, q.z])
    return v + 2 * np.cross(u, np.cross(u, v) + q.w * v)


class RtkTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_rtk_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        pkg = get_package_share_directory('agri_ugv_description')
        with open(os.path.join(pkg, 'worlds', 'olive_orchard.json')) as f:
            self.meta = json.load(f)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.gt = []                      # (t, pose)
        self.fix, self.raw, self.base, self.corr = [], [], [], []
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.create_subscription(NavSatFix, '/gps/fix', self.fix.append, qos)
        self.create_subscription(NavSatFix, '/gps/raw', self.raw.append, qos)
        self.create_subscription(NavSatFix, '/rtk_base/navsat', self.base.append, qos)
        self.create_subscription(Vector3Stamped, '/rtk_base/corrections', self.corr.append, 10)
        self.param_cli = self.create_client(SetParameters, '/rtk_gnss/set_parameters')
        self.antenna = None               # gps_link position in base_footprint
        self.results = []

    def on_gt(self, msg):
        self.gt.append((stamp_s(msg), msg.pose.pose))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_seconds, cmd=None):
        t_end, next_pub = self.now() + sim_seconds, 0.0
        while rclpy.ok() and self.now() < t_end:
            if cmd is not None and self.now() >= next_pub:
                self.pub.publish(cmd)
                next_pub = self.now() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def check(self, name, ok, detail):
        self.results.append((name, bool(ok), detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name}: {detail}')

    def true_antenna(self, t):
        """Ground-truth antenna position at time t (interpolated between gt samples)."""
        times = [g[0] for g in self.gt]
        i = bisect.bisect_left(times, t)
        if i == 0 or i >= len(times):
            return None
        (t0, p0), (t1, p1) = self.gt[i - 1], self.gt[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        pos = [np.array([p.position.x, p.position.y, p.position.z]) + rotate(p.orientation, self.antenna)
               for p in (p0, p1)]
        return (1 - a) * pos[0] + a * pos[1]

    def errors(self, fixes):
        """Horizontal and vertical errors of fixes with respect to the true antenna."""
        h, v = [], []
        for f in fixes:
            true = self.true_antenna(stamp_s(f))
            if true is None:
                continue
            d = fix_to_xyz(f) - true
            h.append(math.hypot(d[0], d[1]))
            v.append(abs(d[2]))
        return np.array(h), np.array(v)

    def set_rtk(self, enabled):
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='rtk_enabled', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        fut = self.param_cli.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done() and time.monotonic() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done() and fut.result().results[0].successful

    def window(self, seconds, cmd=None):
        """Collect fresh messages for `seconds` of sim time."""
        for lst in (self.fix, self.raw, self.base, self.corr):
            lst.clear()
        self.spin_for(seconds, cmd)

    def run(self):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < 60:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.antenna is None and self.tf.can_transform('base_footprint', 'gps_link', rclpy.time.Time()):
                tr = self.tf.lookup_transform('base_footprint', 'gps_link', rclpy.time.Time()).transform.translation
                self.antenna = np.array([tr.x, tr.y, tr.z])
            if self.antenna is not None and self.gt and self.fix and self.raw and self.corr:
                break
        got = {'ground truth': bool(self.gt), 'tf gps_link': self.antenna is not None, '/gps/fix': bool(self.fix),
               '/gps/raw': bool(self.raw), '/rtk_base/navsat': bool(self.base), '/rtk_base/corrections': bool(self.corr)}
        missing = [k for k, ok in got.items() if not ok]
        self.check('rtk topics publishing', not missing,
                   'gps/fix, gps/raw, rtk_base/navsat, rtk_base/corrections received' if not missing
                   else f'missing: {missing}')
        if missing or not self.param_cli.wait_for_service(timeout_sec=10.0):
            return False

        # ---- base station on its known point, corrections at ~1 Hz (robot at rest)
        self.window(6.0)
        b = self.meta['rtk_base']
        base_true = np.array([b['x'], b['y'], b['z'] + BASE_ANTENNA_H])
        base_meas = np.mean([fix_to_xyz(f) for f in self.base], axis=0)
        d = base_meas - base_true
        self.check('base station receiver on the known point', math.hypot(d[0], d[1]) < 0.03 and abs(d[2]) < 0.05,
                   f'measured ({base_meas[0]:.3f}, {base_meas[1]:.3f}, {base_meas[2]:.3f}) m, '
                   f'world file ({base_true[0]:.3f}, {base_true[1]:.3f}, {base_true[2]:.3f}) m')
        ts = [stamp_s(c) for c in self.corr]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else 0.0
        self.check('corrections sent at ~1 Hz', 0.8 < rate < 1.2, f'{rate:.2f} Hz ({len(ts)} in 6 s sim time)')
        c = self.corr[-1].vector
        ch = math.hypot(c.x, c.y)
        self.check('corrections carry the common GPS error', ch > 1.0,
                   f'horizontal {ch:.2f} m (E {c.x:+.2f}, N {c.y:+.2f}), vertical {c.z:+.2f} m')

        # ---- at rest: standalone vs RTK
        self.window(5.0)
        raw_h, _ = self.errors(self.raw)
        fix_h, fix_v = self.errors(self.fix)
        status = {f.status.status for f in self.fix}
        self.check('standalone gps error is metre-level', raw_h.mean() > 1.0,
                   f'mean {raw_h.mean():.2f} m over {len(raw_h)} fixes')
        self.check('rtk fix status', status == {NavSatStatus.STATUS_GBAS_FIX},
                   f'statuses {sorted(status)} (2 = GBAS_FIX / RTK)')
        self.check('rtk accuracy at rest', np.sqrt(np.mean(fix_h ** 2)) < 0.03 and np.sqrt(np.mean(fix_v ** 2)) < 0.06,
                   f'horizontal RMS {100 * np.sqrt(np.mean(fix_h ** 2)):.1f} cm, '
                   f'vertical RMS {100 * np.sqrt(np.mean(fix_v ** 2)):.1f} cm over {len(fix_h)} fixes')
        cov = self.fix[-1].position_covariance
        self.check('rtk covariance', abs(math.sqrt(cov[0]) - 0.02) < 1e-6,
                   f'reported horizontal sigma {100 * math.sqrt(cov[0]):.1f} cm')

        # ---- driving along the path between rows 2 and 3, gentle curve
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = 0.5, 0.05
        self.window(12.0, cmd)
        self.spin_for(1.0, Twist())
        raw_h, _ = self.errors(self.raw)
        fix_h, _ = self.errors(self.fix)
        p0, p1 = self.true_antenna(stamp_s(self.fix[0])), self.true_antenna(stamp_s(self.fix[-1]))
        dist = math.hypot(*(p1 - p0)[:2])
        self.check('rtk accuracy while driving',
                   dist > 4.0 and np.sqrt(np.mean(fix_h ** 2)) < 0.03 and fix_h.max() < 0.08,
                   f'{dist:.1f} m driven, RMS {100 * np.sqrt(np.mean(fix_h ** 2)):.1f} cm, '
                   f'max {100 * fix_h.max():.1f} cm (standalone RMS {np.sqrt(np.mean(raw_h ** 2)):.2f} m)')
        self.write_trace()

        # ---- RTK switched off, then on again
        ok_off = self.set_rtk(False)
        self.spin_for(1.0)
        self.window(3.0)
        off_h, _ = self.errors(self.fix)
        status = {f.status.status for f in self.fix}
        self.check('rtk off -> standalone fallback',
                   ok_off and status == {NavSatStatus.STATUS_FIX} and off_h.mean() > 1.0,
                   f'statuses {sorted(status)} (0 = FIX), mean error {off_h.mean():.2f} m')
        ok_on = self.set_rtk(True)
        self.spin_for(1.0)
        self.window(3.0)
        on_h, _ = self.errors(self.fix)
        status = {f.status.status for f in self.fix}
        self.check('rtk on again -> centimetre accuracy',
                   ok_on and status == {NavSatStatus.STATUS_GBAS_FIX} and np.sqrt(np.mean(on_h ** 2)) < 0.03,
                   f'statuses {sorted(status)}, RMS {100 * np.sqrt(np.mean(on_h ** 2)):.1f} cm')

        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)

    def write_trace(self):
        path = os.path.expanduser('~/.ros/agri_ugv_tests/rtk_trace.csv')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        raw = {round(stamp_s(f), 3): fix_to_xyz(f) for f in self.raw}
        with open(path, 'w') as out:
            out.write('t,true_x,true_y,raw_x,raw_y,rtk_x,rtk_y\n')
            for f in self.fix:
                t = round(stamp_s(f), 3)
                true = self.true_antenna(stamp_s(f))
                if true is None or t not in raw:
                    continue
                r, k = raw[t], fix_to_xyz(f)
                out.write(f'{t:.3f},{true[0]:.4f},{true[1]:.4f},{r[0]:.4f},{r[1]:.4f},{k[0]:.4f},{k[1]:.4f}\n')
        self.get_logger().info(f'trace written to {path}')


def main():
    rclpy.init()
    node = RtkTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
