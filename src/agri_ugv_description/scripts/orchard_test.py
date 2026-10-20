#!/usr/bin/env python3
"""Spawn and /cmd_vel test for agri_ugv in the olive_orchard world.

Checks, against Gazebo ground truth (/ground_truth/odom):
  - the robot spawned at the expected pose and rests level on the terrain
  - its height matches the heightmap (detects a misaligned/flipped collision map)
  - /cmd_vel drives it along the dirt path without leaving it or nearing a trunk
  - stop, rotation in place, and reverse still work on uneven ground

Run with the simulation up:
  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true
  ros2 run agri_ugv_description orchard_test.py
Exit code 0 if every check passes.
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.parameter import Parameter

LIN_ACC = 1.0          # matches max_linear_acceleration in agri_ugv.gazebo.xacro
ANG_ACC = 3.0          # matches max_angular_acceleration
ROBOT_RADIUS = 0.61    # circumscribed radius of the 0.990 x 0.698 m footprint


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class Terrain:
    def __init__(self, pkg, meta):
        img = np.asarray(Image.open(os.path.join(pkg, meta['heightmap'])), dtype=float)
        self.h = img / 255.0 * meta['height_range']
        self.size = meta['terrain_size']

    def z(self, x, y):
        n = self.h.shape[0]
        step = self.size / (n - 1)
        fc, fr = (x + self.size / 2) / step, (self.size / 2 - y) / step
        c0 = min(max(int(math.floor(fc)), 0), n - 2)
        r0 = min(max(int(math.floor(fr)), 0), n - 2)
        tc, tr = fc - c0, fr - r0
        h = self.h
        return ((h[r0, c0] * (1 - tc) + h[r0, c0 + 1] * tc) * (1 - tr)
                + (h[r0 + 1, c0] * (1 - tc) + h[r0 + 1, c0 + 1] * tc) * tr)


class OrchardTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_orchard_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        pkg = get_package_share_directory('agri_ugv_description')
        with open(os.path.join(pkg, 'worlds', 'olive_orchard.json')) as f:
            self.meta = json.load(f)
        self.terrain = Terrain(pkg, self.meta)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.msg = None
        self.yaw_u = 0.0
        self._last_yaw = None
        self.log = None          # list of samples while recording
        self.results = []

    def on_gt(self, msg):
        r, p, y = rpy(msg.pose.pose.orientation)
        self.yaw_u = y if self._last_yaw is None else self.yaw_u + wrap(y - self._last_yaw)
        self._last_yaw = y
        self.msg = msg
        if self.log is not None:
            self.log.append(self.state())

    def state(self):
        pp = self.msg.pose.pose
        r, p, _ = rpy(pp.orientation)
        t = self.msg.twist.twist
        return dict(x=pp.position.x, y=pp.position.y, z=pp.position.z, roll=r, pitch=p,
                    yaw=self.yaw_u, v=t.linear.x, w=t.angular.z)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, v, w, duration):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        t_end, next_pub = self.now() + duration, 0.0
        while rclpy.ok() and self.now() < t_end:
            if self.now() >= next_pub:
                self.pub.publish(cmd)
                next_pub = self.now() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def check(self, name, ok, detail):
        self.results.append((name, bool(ok), detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name}: {detail}')

    def terrain_err(self, s):
        return s['z'] - self.terrain.z(s['x'], s['y'])

    def trunk_clearance(self, s):
        return min(math.hypot(s['x'] - t['x'], s['y'] - t['y']) - t['trunk_radius']
                   for t in self.meta['trees']) - ROBOT_RADIUS

    def run(self):
        start = time.monotonic()
        while rclpy.ok() and self.msg is None and time.monotonic() - start < 60:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.msg is None:
            self.check('robot spawned', False, 'no /ground_truth/odom within 60 s')
            return False

        sp = self.meta['spawn']
        path_y = sp['y']
        self.drive(0.0, 0.0, 3.0)
        s = self.state()
        self.check('robot spawned at safe pose',
                   math.hypot(s['x'] - sp['x'], s['y'] - sp['y']) < 0.10,
                   f'x={s["x"]:.3f} y={s["y"]:.3f} (spawn {sp["x"]}, {sp["y"]}), '
                   f'trunk clearance {self.trunk_clearance(s):.2f} m')
        self.check('resting level and still',
                   abs(math.degrees(s['roll'])) < 5 and abs(math.degrees(s['pitch'])) < 5
                   and abs(s['v']) < 0.02 and abs(s['w']) < 0.02,
                   f'roll {math.degrees(s["roll"]):+.2f} deg pitch {math.degrees(s["pitch"]):+.2f} deg '
                   f'v={s["v"]:+.3f} w={s["w"]:+.3f}')
        self.check('on terrain surface', abs(self.terrain_err(s)) < 0.04,
                   f'z={s["z"]:.3f} m, heightmap {self.terrain.z(s["x"], s["y"]):.3f} m')

        # Forward along the dirt path, into the orchard rows
        T = 16.0
        s0 = self.state()
        self.log = []
        self.drive(0.5, 0.0, T)
        samples, self.log = self.log, None
        s1 = self.state()
        exp = 0.5 * T - 0.5 * 0.5 ** 2 / LIN_ACC
        fwd = s1['x'] - s0['x']
        self.check('/cmd_vel forward along path', abs(fwd - exp) < 0.10 * exp,
                   f'{fwd:.3f} m (expected {exp:.3f} m +-10%), now at x={s1["x"]:.2f}')
        dev = max(abs(q['y'] - path_y) for q in samples)
        clear = min(self.trunk_clearance(q) for q in samples)
        self.check('stays on path, clear of trunks', dev < 0.40 and clear > 0.0,
                   f'max lateral deviation {dev:.3f} m, min trunk clearance {clear:.2f} m')
        zerr = max(abs(self.terrain_err(q)) for q in samples)
        tilt = max(max(abs(q['roll']), abs(q['pitch'])) for q in samples)
        zs = [self.terrain.z(q['x'], q['y']) for q in samples]
        self.check('follows uneven terrain', zerr < 0.05 and math.degrees(tilt) < 8,
                   f'terrain traversed {min(zs):.3f}..{max(zs):.3f} m, max |z - heightmap| '
                   f'{zerr:.3f} m, max tilt {math.degrees(tilt):.2f} deg')

        self.drive(0.0, 0.0, 2.0)
        s = self.state()
        self.check('stop', abs(s['v']) < 0.02 and abs(s['w']) < 0.02,
                   f'v={s["v"]:+.4f} w={s["w"]:+.4f}')

        # Rotation in place, +90 deg then back
        for name, w in (('rotate left 90 deg', 0.5), ('rotate right 90 deg', -0.5)):
            s0 = self.state()
            self.drive(0.0, w, (math.pi / 2) / abs(w) + abs(w) / (2 * ANG_ACC))
            self.drive(0.0, 0.0, 1.5)
            s1 = self.state()
            dyaw = math.degrees(s1['yaw'] - s0['yaw'])
            drift = math.hypot(s1['x'] - s0['x'], s1['y'] - s0['y'])
            exp = math.copysign(90.0, w)
            self.check(name, abs(dyaw - exp) < 0.15 * 90 and drift < 0.15,
                       f'{dyaw:+.1f} deg (expected {exp:+.0f} +-15%), drift {drift:.3f} m')

        # Reverse
        s0 = self.state()
        self.drive(-0.5, 0.0, 4.0)
        self.drive(0.0, 0.0, 2.0)
        s1 = self.state()
        back = (s1['x'] - s0['x']) * math.cos(s0['yaw']) + (s1['y'] - s0['y']) * math.sin(s0['yaw'])
        self.check('/cmd_vel reverse', back < -1.6, f'{back:+.3f} m (expected about -2.0 incl. braking)')
        self.check('stop after reverse', abs(s1['v']) < 0.02, f'v={s1["v"]:+.4f}')

        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)


def main():
    rclpy.init()
    node = OrchardTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
