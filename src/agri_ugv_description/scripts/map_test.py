#!/usr/bin/env python3
"""Mapping test for agri_ugv in the olive_orchard world.

Drives the robot through every path of the orchard, following waypoints with the
EKF pose, while slam_toolbox builds the map. Then checks the map against the
world file:
  - it is aligned with the world, and covers the orchard
  - every tree is on the map, at its true position
  - the paths are free: no false obstacles (ground hits, ...)
and saves it to ~/.ros/agri_ugv_tests/maps/olive_orchard.{pgm,yaml}.

Run with the simulation up:
  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true slam:=true camera:=false
  ros2 run agri_ugv_description map_test.py
Exit code 0 if every check passes.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

SPEED = 0.8                 # m/s
HEADLAND = 22.0             # m, x of the turns at both ends of the rows
MIN_TREES_FOUND = 1.0       # fraction of the trees
MAX_TRUNK_ERROR = 0.10      # m, mean
TRUNK_SEARCH = 0.5          # m around each true trunk
MAX_FALSE_CELLS = 0.02      # fraction of the occupied cells
MIN_COVERAGE = 0.90         # fraction of the orchard
MIN_TREE_CLEARANCE = 1.0    # m, robot centre to trunk


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MapTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_map_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        pkg = get_package_share_directory('agri_ugv_description')
        with open(os.path.join(pkg, 'worlds', 'olive_orchard.json')) as f:
            self.meta = json.load(f)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = self.true = self.map = None
        self.min_clearance = math.inf
        self.tree_xy = np.array([[t['x'], t['y']] for t in self.meta['trees']])
        self.create_subscription(Odometry, '/odometry/global', self.on_pose, 50)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.results = []

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.true = (p.x, p.y)
        self.min_clearance = min(self.min_clearance,
                                 np.min(np.hypot(*(self.tree_xy - [p.x, p.y]).T)))

    def on_map(self, msg):
        self.map = msg

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check(self, name, ok, detail):
        self.results.append((name, bool(ok), detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name}: {detail}')

    def wait(self, sim_seconds):
        t_end = self.now() + sim_seconds
        while rclpy.ok() and self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def go_to(self, x, y):
        """Turn towards (x, y) and drive there, on the EKF pose."""
        cmd = Twist()
        t_end = self.now() + 3.0 * math.dist(self.pose[:2], (x, y)) / SPEED + 40.0
        next_pub = 0.0
        while rclpy.ok() and self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.now() < next_pub:
                continue
            next_pub = self.now() + 0.05
            dist = math.dist(self.pose[:2], (x, y))
            if dist < 0.3:
                self.pub.publish(Twist())
                return True
            err = wrap(math.atan2(y - self.pose[1], x - self.pose[0]) - self.pose[2])
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * err))
            cmd.linear.x = 0.0 if abs(err) > 0.4 else SPEED * max(0.3, 1.0 - abs(err))
            self.pub.publish(cmd)
        self.pub.publish(Twist())
        return False

    def route(self):
        """Every path in turn: up, down, up, then back to the last one."""
        py = self.meta['path_y']        # -9, -3, 3, 9
        return [(HEADLAND, py[1]), (HEADLAND, py[2]), (-HEADLAND, py[2]), (-HEADLAND, py[3]),
                (HEADLAND, py[3]), (HEADLAND + 2.0, py[0]), (-HEADLAND, py[0])]

    def run(self):
        start = self.now()
        while rclpy.ok() and (self.pose is None or self.true is None or self.now() - start < 5.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.check('localization available', self.pose is not None,
                   f'robot at ({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        for i, (x, y) in enumerate(self.route()):
            ok = self.go_to(x, y)
            self.get_logger().info(f'[INFO] waypoint {i + 1}: ({x:.0f}, {y:.0f}) {"reached" if ok else "NOT reached"}, '
                                   f'sim time {self.now() - start:.0f} s')
        self.wait(8.0)                  # last map update
        self.check('route driven', math.dist(self.true, self.route()[-1]) < 1.0,
                   f'{self.now() - start:.0f} s of sim time, ends at ({self.true[0]:.2f}, {self.true[1]:.2f})')
        self.check('clearance to the trees', self.min_clearance > MIN_TREE_CLEARANCE,
                   f'closest trunk {self.min_clearance:.2f} m')

        if self.map is None:
            self.check('map published', False, 'no /map received')
            return False
        self.check_map()

        out = os.path.expanduser('~/.ros/agri_ugv_tests/maps')
        os.makedirs(out, exist_ok=True)
        saver = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', os.path.join(out, 'olive_orchard'),
             '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=20.0'],
            capture_output=True, text=True, timeout=120)
        saved = all(os.path.exists(os.path.join(out, 'olive_orchard' + e)) for e in ('.pgm', '.yaml'))
        self.check('map saved', saver.returncode == 0 and saved, os.path.join(out, 'olive_orchard.yaml'))

        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)

    def check_map(self):
        m, info = self.map, self.map.info
        res = info.resolution
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        ox, oy = info.origin.position.x, info.origin.position.y
        ys, xs = np.nonzero(grid >= 65)
        occ = np.c_[ox + (xs + 0.5) * res, oy + (ys + 0.5) * res]
        self.check('map published', len(occ) > 0,
                   f'{info.width}x{info.height} cells of {100 * res:.0f} cm, frame {m.header.frame_id}, '
                   f'{len(occ)} occupied')

        # every trunk: the occupied cells around its true position
        errors, missing = [], []
        for t in self.meta['trees']:
            near = occ[np.hypot(*(occ - [t['x'], t['y']]).T) < TRUNK_SEARCH]
            if len(near) == 0:
                missing.append(t['name'])
            else:
                errors.append(math.dist(near.mean(axis=0), (t['x'], t['y'])))
        found = len(errors) / len(self.meta['trees'])
        self.check('all trees on the map', found >= MIN_TREES_FOUND,
                   f'{len(errors)}/{len(self.meta["trees"])} found' + (f', missing {missing}' if missing else ''))
        self.check('trees at their true position', errors and np.mean(errors) < MAX_TRUNK_ERROR,
                   f'mean error {100 * np.mean(errors):.1f} cm, max {100 * np.max(errors):.1f} cm')

        # the orchard rectangle: free where nothing stands
        xr, yr = (self.meta['tree_x'][0] - 2.0, self.meta['tree_x'][-1] + 2.0), (-12.0, 12.0)
        inside = (occ[:, 0] > xr[0]) & (occ[:, 0] < xr[1]) & (occ[:, 1] > yr[0]) & (occ[:, 1] < yr[1])
        real = self.tree_xy.tolist() + [[r['x'], r['y']] for r in self.meta['rocks']]
        real = np.array(real)
        d = np.min(np.hypot(occ[:, None, 0] - real[:, 0], occ[:, None, 1] - real[:, 1]), axis=1)
        false = inside & (d > 1.0)
        self.check('no false obstacles in the orchard', false.sum() < MAX_FALSE_CELLS * max(1, inside.sum()),
                   f'{false.sum()} of {inside.sum()} occupied cells are more than 1 m from any tree or rock')

        i0, i1 = int((xr[0] - ox) / res), int((xr[1] - ox) / res)
        j0, j1 = int((yr[0] - oy) / res), int((yr[1] - oy) / res)
        part = grid[max(0, j0):max(0, j1), max(0, i0):max(0, i1)]
        known = np.count_nonzero(part >= 0) / ((j1 - j0) * (i1 - i0))
        self.check('orchard covered', known > MIN_COVERAGE, f'{100 * known:.1f}% of the orchard is mapped')


def main():
    rclpy.init()
    node = MapTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
