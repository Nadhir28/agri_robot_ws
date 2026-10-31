#!/usr/bin/env python3
"""Sensor fusion test for agri_ugv in the olive_orchard world.

Checks (against Gazebo ground truth):
  - both EKFs and navsat_transform publish, and the map -> odom -> base_footprint
    tree exists
  - the global estimate is the true pose at rest, while driving, and through
    90 deg turns on the spot, where wheel odometry is wrong
  - the local estimate is smooth (no jumps)
  - with RTK switched off the estimate stays usable, and it comes back to
    centimetres when RTK returns

Writes the trace (truth, wheels, EKF) to ~/.ros/agri_ugv_tests/ekf_trace.csv
for the report figure.

Run with the simulation up:
  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true
  ros2 run agri_ugv_description ekf_test.py
Exit code 0 if every check passes.
"""
import bisect
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

MAX_POS_ERROR = 0.05       # m, RMS, RTK available
MAX_YAW_ERROR = 2.0        # deg
MAX_OUTAGE_ERROR = 0.50    # m, RTK off (half of the margin to the trees)
MAX_LOCAL_STEP = 0.05      # m between two messages of the local estimate


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose2d(msg):
    p = msg.pose.pose
    return np.array([p.position.x, p.position.y, yaw_of(p.orientation)])


class EkfTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_ekf_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.gt, self.gt_t = [], []       # true (x, y, yaw) and its stamps
        self.glob, self.local, self.wheel, self.gps = [], [], [], []
        self.latest = None                # newest global estimate
        self.trace = []
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.create_subscription(Odometry, '/odometry/global', self.on_global, 50)
        self.create_subscription(Odometry, '/odometry/local', self.local.append, 50)
        self.create_subscription(Odometry, '/odom', self.wheel.append, 50)
        self.create_subscription(Odometry, '/odometry/gps', self.gps.append, 50)
        self.param_cli = self.create_client(SetParameters, '/rtk_gnss/set_parameters')
        self.results = []

    def on_gt(self, msg):
        self.gt.append(pose2d(msg))
        self.gt_t.append(stamp_s(msg))

    def on_global(self, msg):
        self.latest = msg
        self.glob.append(msg)
        self.trace.append(msg)

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

    def truth(self, t):
        """True (x, y, yaw) at time t (nearest ground-truth sample)."""
        i = bisect.bisect_left(self.gt_t, t)
        if i == 0 or i >= len(self.gt_t):
            return None
        return self.gt[i]

    def errors(self, msgs):
        """Position (m) and heading (deg) errors of an estimate in the world frame."""
        pos, yaw = [], []
        for m in msgs:
            true = self.truth(stamp_s(m))
            if true is None:
                continue
            e = pose2d(m)
            pos.append(math.hypot(*(e - true)[:2]))
            yaw.append(abs(math.degrees(wrap(e[2] - true[2]))))
        return np.array(pos), np.array(yaw)

    def window(self, seconds, cmd=None):
        self.glob.clear()
        self.local.clear()
        self.spin_for(seconds, cmd)

    def rotate(self, angle):
        """Turn on the spot by `angle`, closing the loop on the global estimate."""
        target = pose2d(self.latest)[2] + angle
        cmd = Twist()
        t_end = self.now() + 40.0
        while rclpy.ok() and self.now() < t_end:
            left = wrap(target - pose2d(self.latest)[2])
            if abs(left) < 0.02:
                break
            cmd.angular.z = math.copysign(max(0.3, min(0.5, 2.0 * abs(left))), left)
            self.spin_for(0.05, cmd)
        self.spin_for(1.5, Twist())

    def set_rtk(self, enabled):
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='rtk_enabled', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        fut = self.param_cli.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done() and time.monotonic() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done() and fut.result().results[0].successful

    def run(self):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < 60:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gt and self.glob and self.local and self.gps and self.wheel:
                break
        got = {'/odometry/global': bool(self.glob), '/odometry/local': bool(self.local),
               '/odometry/gps': bool(self.gps), '/odom': bool(self.wheel), 'ground truth': bool(self.gt)}
        missing = [k for k, ok in got.items() if not ok]
        self.check('fusion topics publishing', not missing,
                   'global, local, gps odometry received' if not missing else f'missing: {missing}')
        if missing or not self.param_cli.wait_for_service(timeout_sec=10.0):
            return False

        self.spin_for(5.0)
        tree = (self.tf.can_transform('map', 'odom', rclpy.time.Time())
                and self.tf.can_transform('odom', 'base_footprint', rclpy.time.Time()))
        self.check('tf tree map -> odom -> base_footprint', tree, 'both transforms available' if tree else 'missing')

        # ---- at rest
        self.window(5.0)
        pos, yaw = self.errors(self.glob)
        self.check('estimate at rest', pos.mean() < MAX_POS_ERROR and yaw.mean() < MAX_YAW_ERROR,
                   f'position error {100 * pos.mean():.1f} cm, heading error {yaw.mean():.2f} deg')

        wheel_start, true_start = pose2d(self.wheel[-1]), self.gt[-1].copy()

        # ---- driving along the path: gentle S, the robot stays between the rows
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = 0.5, 0.05
        self.window(7.0, cmd)
        cmd.angular.z = -0.05
        self.spin_for(7.0, cmd)
        self.spin_for(1.0, Twist())
        pos, yaw = self.errors(self.glob)
        self.check('accuracy while driving',
                   np.sqrt(np.mean(pos ** 2)) < MAX_POS_ERROR and yaw.max() < MAX_YAW_ERROR,
                   f'RMS {100 * np.sqrt(np.mean(pos ** 2)):.1f} cm, max {100 * pos.max():.1f} cm, '
                   f'heading max {yaw.max():.2f} deg')
        steps = [math.dist(pose2d(a)[:2], pose2d(b)[:2]) for a, b in zip(self.local, self.local[1:])]
        self.check('local estimate is smooth', max(steps) < MAX_LOCAL_STEP,
                   f'largest step {100 * max(steps):.1f} cm over {len(steps)} messages')

        # ---- turns on the spot: wheels slip, the gyro does not
        turned_wheels, turned_true = 0.0, 0.0
        for angle in (math.pi / 2, -math.pi / 2):
            w0, t0 = pose2d(self.wheel[-1])[2], self.gt[-1][2]
            self.glob.clear()
            self.rotate(angle)
            pos, yaw = self.errors(self.glob)
            turned_wheels = math.degrees(wrap(pose2d(self.wheel[-1])[2] - w0))
            turned_true = math.degrees(wrap(self.gt[-1][2] - t0))
            self.check(f'turn on the spot {math.degrees(angle):+.0f} deg',
                       pos.max() < 2 * MAX_POS_ERROR and yaw.max() < MAX_YAW_ERROR
                       and abs(abs(turned_true) - 90.0) < 5.0,
                       f'true {turned_true:+.1f} deg, wheels say {turned_wheels:+.1f} deg, '
                       f'EKF error: heading max {yaw.max():.2f} deg, position max {100 * pos.max():.1f} cm')

        # ---- whole route so far: EKF against wheels alone
        true_end = self.gt[-1]
        route = math.dist(true_start[:2], true_end[:2])
        d_true = wrap(true_end[2] - true_start[2])
        c, s = math.cos(true_start[2] - wheel_start[2]), math.sin(true_start[2] - wheel_start[2])
        dw = pose2d(self.wheel[-1])[:2] - wheel_start[:2]
        wheel_end = true_start[:2] + np.array([c * dw[0] - s * dw[1], s * dw[0] + c * dw[1]])
        wheel_err = math.dist(wheel_end, true_end[:2])
        ekf_err = math.dist(pose2d(self.latest)[:2], true_end[:2])
        self.check('EKF beats wheel odometry', ekf_err < wheel_err and ekf_err < MAX_POS_ERROR,
                   f'after {route:.1f} m and two turns: EKF {100 * ekf_err:.1f} cm, wheels alone '
                   f'{100 * wheel_err:.0f} cm (true heading change {math.degrees(d_true):+.1f} deg)')

        # ---- RTK lost for 20 s of driving, then back
        cmd.linear.x, cmd.angular.z = 0.5, 0.0
        ok_off = self.set_rtk(False)
        self.window(20.0, cmd)
        pos, yaw = self.errors(self.glob)
        self.check('rtk off for 20 s while driving', ok_off and pos.max() < MAX_OUTAGE_ERROR,
                   f'position error max {100 * pos.max():.1f} cm, final {100 * pos[-1]:.1f} cm, '
                   f'heading max {yaw.max():.2f} deg')
        ok_on = self.set_rtk(True)
        self.spin_for(1.0, Twist())
        self.spin_for(5.0)
        self.window(3.0)
        pos, yaw = self.errors(self.glob)
        self.check('rtk back -> centimetre accuracy', ok_on and pos.mean() < MAX_POS_ERROR,
                   f'position error {100 * pos.mean():.1f} cm, heading error {yaw.mean():.2f} deg')

        self.write_trace()
        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)

    def write_trace(self):
        path = os.path.expanduser('~/.ros/agri_ugv_tests/ekf_trace.csv')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as out:
            out.write('t,true_x,true_y,true_yaw,ekf_x,ekf_y,ekf_yaw\n')
            for m in self.trace:
                t = stamp_s(m)
                true = self.truth(t)
                if true is None:
                    continue
                e = pose2d(m)
                out.write(f'{t:.3f},{true[0]:.4f},{true[1]:.4f},{true[2]:.4f},'
                          f'{e[0]:.4f},{e[1]:.4f},{e[2]:.4f}\n')
        self.get_logger().info(f'trace written to {path}')


def main():
    rclpy.init()
    node = EkfTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
