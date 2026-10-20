#!/usr/bin/env python3
"""Automated drive/rotation test for agri_ugv in Gazebo.

Publishes /cmd_vel phase by phase (timed on simulation time) and compares the
motion measured by Gazebo's ground-truth pose (/ground_truth/odom) and the
wheel odometry (/odom) with what was commanded.

Run with the simulation already up:
  ros2 run agri_ugv_description motion_test.py
Exit code 0 if every check passes.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

LIN_ACC = 1.0   # must match max_linear_acceleration in agri_ugv.gazebo.xacro
ANG_ACC = 3.0   # must match max_angular_acceleration


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    """Pose tracker with unwrapped yaw."""

    def __init__(self):
        self.msg = None
        self.yaw = 0.0
        self._last_raw = None

    def update(self, msg):
        raw = yaw_of(msg.pose.pose.orientation)
        if self._last_raw is not None:
            self.yaw += wrap(raw - self._last_raw)
        else:
            self.yaw = raw
        self._last_raw = raw
        self.msg = msg

    def snapshot(self):
        p = self.msg.pose.pose.position
        t = self.msg.twist.twist
        return dict(x=p.x, y=p.y, z=p.z, yaw=self.yaw, v=t.linear.x, w=t.angular.z)


class MotionTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_motion_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gt = Track()
        self.odom = Track()
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt.update, 50)
        self.create_subscription(Odometry, '/odom', self.odom.update, 50)
        self.results = []

    # ---------------------------------------------------------------- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_ready(self, timeout_wall=60.0):
        import time
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout_wall:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gt.msg and self.odom.msg and self.now() > 0.0:
                return True
        return False

    def drive(self, v, w, duration):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        t_end = self.now() + duration
        next_pub = 0.0
        while rclpy.ok() and self.now() < t_end:
            if self.now() >= next_pub:
                self.pub.publish(cmd)
                next_pub = self.now() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name}: {detail}')

    @staticmethod
    def ramp_distance(v, T, a):
        """Distance covered when accelerating from 0 to v at rate a, over T."""
        t_ramp = abs(v) / a
        if T <= t_ramp:
            return math.copysign(0.5 * a * T * T, v)
        return v * T - math.copysign(0.5 * a * t_ramp * t_ramp, v)

    def phase(self, name, v, w, T, settle=2.0):
        g0, o0 = self.gt.snapshot(), self.odom.snapshot()
        self.drive(v, w, T)
        g1, o1 = self.gt.snapshot(), self.odom.snapshot()
        self.drive(0.0, 0.0, settle)
        g2 = self.gt.snapshot()

        # displacement expressed in the start heading frame
        dx, dy = g1['x'] - g0['x'], g1['y'] - g0['y']
        c, s = math.cos(g0['yaw']), math.sin(g0['yaw'])
        fwd, lat = c * dx + s * dy, -s * dx + c * dy
        dyaw = g1['yaw'] - g0['yaw']
        odx, ody = o1['x'] - o0['x'], o1['y'] - o0['y']
        oc, os_ = math.cos(o0['yaw']), math.sin(o0['yaw'])
        o_fwd = oc * odx + os_ * ody
        o_dyaw = o1['yaw'] - o0['yaw']
        dist = math.hypot(dx, dy)
        info = dict(name=name, v=v, w=w, T=T, fwd=fwd, lat=lat, dist=dist, dyaw=dyaw,
                    o_fwd=o_fwd, o_dyaw=o_dyaw, v_end=g1['v'], w_end=g1['w'],
                    v_settle=g2['v'], w_settle=g2['w'], z=g1['z'],
                    drift_after_stop=math.hypot(g2['x'] - g1['x'], g2['y'] - g1['y']))
        self.get_logger().info(
            f'{name}: fwd={fwd:+.3f} m lat={lat:+.3f} m dyaw={math.degrees(dyaw):+.1f} deg | '
            f'odom fwd={o_fwd:+.3f} dyaw={math.degrees(o_dyaw):+.1f} deg | '
            f'v_end={g1["v"]:+.3f} w_end={g1["w"]:+.3f} | '
            f'after stop v={g2["v"]:+.3f} w={g2["w"]:+.3f}')
        return info

    # ---------------------------------------------------------------- tests
    def run(self):
        if not self.wait_ready():
            self.check('topics available', False, 'no /odom or /ground_truth/odom received')
            return False
        self.drive(0.0, 0.0, 2.0)  # let the robot settle on the ground
        z0 = self.gt.snapshot()['z']
        self.check('robot resting on ground', abs(z0) < 0.02, f'base_footprint z={z0:.4f} m')

        # 1. Forward
        r = self.phase('forward', 0.5, 0.0, 4.0)
        exp = self.ramp_distance(0.5, 4.0, LIN_ACC)
        self.check('forward distance', abs(r['fwd'] - exp) < 0.10 * exp,
                   f'{r["fwd"]:.3f} m (expected {exp:.3f} m +-10%)')
        self.check('forward straight', abs(r['lat']) < 0.05 and abs(math.degrees(r['dyaw'])) < 2.0,
                   f'lateral {r["lat"]:+.3f} m, yaw {math.degrees(r["dyaw"]):+.2f} deg')
        self.check('forward speed', abs(r['v_end'] - 0.5) < 0.05, f'{r["v_end"]:.3f} m/s')

        # 2. Stop
        self.check('stop after forward', abs(r['v_settle']) < 0.02 and abs(r['w_settle']) < 0.02,
                   f'v={r["v_settle"]:+.4f} m/s w={r["w_settle"]:+.4f} rad/s, '
                   f'coast {r["drift_after_stop"]:.3f} m')

        # 3. Backward
        r = self.phase('backward', -0.5, 0.0, 4.0)
        exp = self.ramp_distance(-0.5, 4.0, LIN_ACC)
        self.check('backward distance', abs(r['fwd'] - exp) < 0.10 * abs(exp),
                   f'{r["fwd"]:.3f} m (expected {exp:.3f} m +-10%)')
        self.check('backward straight', abs(r['lat']) < 0.05 and abs(math.degrees(r['dyaw'])) < 2.0,
                   f'lateral {r["lat"]:+.3f} m, yaw {math.degrees(r["dyaw"]):+.2f} deg')
        self.check('stop after backward', abs(r['v_settle']) < 0.02 and abs(r['w_settle']) < 0.02,
                   f'v={r["v_settle"]:+.4f} m/s w={r["w_settle"]:+.4f} rad/s')

        # 4. Arc turns
        for name, w in (('arc left', 0.5), ('arc right', -0.5)):
            r = self.phase(name, 0.5, w, 4.0)
            exp = self.ramp_distance(w, 4.0, ANG_ACC)
            self.check(f'{name} yaw', math.copysign(1, r['dyaw']) == math.copysign(1, w)
                       and abs(r['dyaw'] - exp) < 0.15 * abs(exp),
                       f'{math.degrees(r["dyaw"]):+.1f} deg (expected {math.degrees(exp):+.1f} +-15%)')
            self.check(f'{name} moves forward', r['dist'] > 1.0, f'path chord {r["dist"]:.3f} m')

        # 5. Rotation in place, one full turn each way
        for name, w in (('spin left', 1.0), ('spin right', -1.0)):
            T = 2 * math.pi / abs(w) + abs(w) / (2 * ANG_ACC)
            r = self.phase(name, 0.0, w, T)
            exp = math.copysign(2 * math.pi, w)
            self.check(f'{name} yaw', abs(r['dyaw'] - exp) < 0.10 * abs(exp),
                       f'{math.degrees(r["dyaw"]):+.1f} deg (expected {math.degrees(exp):+.1f} +-10%), '
                       f'odom {math.degrees(r["o_dyaw"]):+.1f} deg')
            self.check(f'{name} in place', r['dist'] < 0.10, f'centre drift {r["dist"]:.3f} m')
            self.check(f'stop after {name}', abs(r['w_settle']) < 0.02,
                       f'w={r["w_settle"]:+.4f} rad/s')

        # 6. Max speed (spec: 2.0 m/s)
        r = self.phase('max speed', 2.0, 0.0, 4.0, settle=3.0)
        self.check('reaches 2.0 m/s', abs(r['v_end'] - 2.0) < 0.10, f'{r["v_end"]:.3f} m/s')
        self.check('stop from max speed', abs(r['v_settle']) < 0.02,
                   f'v={r["v_settle"]:+.4f} m/s, braking distance {r["drift_after_stop"]:.3f} m')

        self.drive(0.0, 0.0, 0.5)
        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)


def main():
    rclpy.init()
    node = MotionTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
