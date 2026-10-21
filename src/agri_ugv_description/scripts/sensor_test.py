#!/usr/bin/env python3
"""Sensor test for agri_ugv in the olive_orchard world.

Checks (against Gazebo ground truth and the known orchard layout):
  - every sensor publishes at its configured rate (measured on sim time)
  - IMU: gravity at rest, yaw rate during a spin matches ground truth
  - GPS: fix near the world origin's lat/lon; moves with the robot
  - 3D LiDAR: 16x900 cloud; sees ground at the right height, canopies above
    the trunks, no self-hits; derived /scan matches the known trunk positions
  - RGB-D camera: colour image not blank; depth image 32FC1 aligned with it,
    depth at the pixels of known trunks = true distance; sky = no return

Run with the simulation up:
  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard headless:=true
  ros2 run agri_ugv_description sensor_test.py
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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2

# Must match agri_ugv.sensors.xacro and the world <spherical_coordinates>
RATES = {'imu': 100.0, 'gps': 10.0, 'points': 10.0, 'scan': 10.0, 'camera': 15.0, 'depth': 15.0}
LIDAR_XYZ = (0.20, 0.0)          # lidar position in base_link (x, y)
LIDAR_HEIGHT = 0.381 + 0.25 + 0.036   # optical centre above ground: plate + mast + half body
LIDAR_RINGS, LIDAR_COLS = 16, 900
CAM_X, CAM_H, CAM_PITCH = 0.35, 0.381 + 0.04, 0.10   # camera: ahead of centre, height, down-tilt
DEPTH_MIN, DEPTH_MAX = 0.3, 10.0
ORIGIN_LAT, ORIGIN_LON = 34.74, 10.76
EARTH_R = 6378137.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SensorTest(Node):
    def __init__(self):
        super().__init__('agri_ugv_sensor_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        pkg = get_package_share_directory('agri_ugv_description')
        with open(os.path.join(pkg, 'worlds', 'olive_orchard.json')) as f:
            self.meta = json.load(f)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last = {}
        self.stamps = {k: [] for k in RATES}
        self.imu_log = None
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: self._store('gt', m), 50)
        self.create_subscription(Imu, '/imu', self.on_imu, qos)
        self.create_subscription(NavSatFix, '/gps/fix', lambda m: self._store('gps', m), qos)
        self.create_subscription(LaserScan, '/scan', lambda m: self._store('scan', m), qos)
        self.create_subscription(PointCloud2, '/points', lambda m: self._store('points', m), qos)
        # images: RELIABLE, best-effort drops most ~0.9 MB frames
        self.create_subscription(Image, '/camera/image_raw', lambda m: self._store('camera', m),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Image, '/camera/depth/image_raw', lambda m: self._store('depth', m),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda m: self._store('cinfo', m), qos)
        self.results = []

    def _store(self, key, msg):
        self.last[key] = msg
        if key in self.stamps:
            self.stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_imu(self, msg):
        self._store('imu', msg)
        if self.imu_log is not None and 'gt' in self.last:
            self.imu_log.append((msg.angular_velocity.z, self.last['gt'].twist.twist.angular.z))

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

    @staticmethod
    def latlon_to_xy(lat, lon):
        x = math.radians(lon - ORIGIN_LON) * EARTH_R * math.cos(math.radians(ORIGIN_LAT))
        y = math.radians(lat - ORIGIN_LAT) * EARTH_R
        return x, y

    def run(self):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < 60:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(k in self.last for k in ('gt', 'imu', 'gps', 'points', 'scan', 'camera', 'depth', 'cinfo')) and self.now() > 0:
                break
        missing = [k for k in ('gt', 'imu', 'gps', 'points', 'scan', 'camera', 'depth', 'cinfo') if k not in self.last]
        self.check('all sensor topics publishing', not missing,
                   'imu, gps, points, scan, camera, depth, camera_info received' if not missing else f'missing: {missing}')
        if missing:
            return False

        # ---- rates over 5 s of sim time, robot at rest
        for k in self.stamps:
            self.stamps[k].clear()
        self.spin_for(5.0)
        for k, nominal in RATES.items():
            st = self.stamps[k]
            rate = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 2 and st[-1] > st[0] else 0.0
            self.check(f'{k} rate', rate > 0.8 * nominal,
                       f'{rate:.1f} Hz sim time (configured {nominal:.0f} Hz, need > 80 %)')

        # ---- IMU at rest
        acc = self.last['imu'].linear_acceleration
        g = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)
        self.check('imu gravity at rest', abs(g - 9.81) < 0.3 and acc.z > 9.0,
                   f'|a|={g:.2f} m/s^2, a_z={acc.z:+.2f}')
        imu_yaw = yaw_of(self.last['imu'].orientation)
        gt_yaw = yaw_of(self.last['gt'].pose.pose.orientation)
        self.check('imu orientation matches ground truth (ENU)',
                   abs(math.degrees(math.atan2(math.sin(imu_yaw - gt_yaw), math.cos(imu_yaw - gt_yaw)))) < 2.0,
                   f'imu yaw {math.degrees(imu_yaw):+.2f} deg, true {math.degrees(gt_yaw):+.2f} deg')

        # ---- GPS at the spawn point
        gt = self.last['gt'].pose.pose.position
        fixes = []
        t_end = self.now() + 3.0
        while self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)
            f = self.last['gps']
            if not fixes or f is not fixes[-1]:
                fixes.append(f)
        gx, gy = np.mean([self.latlon_to_xy(f.latitude, f.longitude) for f in fixes], axis=0)
        # antenna is 0.28 m behind the robot centre (yaw ~0 at spawn)
        err = math.hypot(gx - (gt.x - 0.28 * math.cos(gt_yaw)), gy - (gt.y - 0.28 * math.sin(gt_yaw)))
        self.check('gps position at spawn', err < 0.5,
                   f'lat {fixes[-1].latitude:.7f} lon {fixes[-1].longitude:.7f} -> '
                   f'({gx:.2f}, {gy:.2f}) m, true ({gt.x:.2f}, {gt.y:.2f}), error {err:.2f} m '
                   f'({len(fixes)} fixes averaged)')

        # ---- 3D LiDAR point cloud (robot at rest on the flat spawn pad)
        cloud = self.last['points']
        names = [f.name for f in cloud.fields]
        self.check('3d lidar cloud layout',
                   cloud.height == LIDAR_RINGS and cloud.width == LIDAR_COLS and 'ring' in names,
                   f'{cloud.height} rings x {cloud.width} points, fields {names}, '
                   f'frame {cloud.header.frame_id}')
        pts = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        rng = np.linalg.norm(pts, axis=1)
        self.check('3d lidar does not see the robot itself', rng.min() > 1.0,
                   f'{len(pts)} valid points, closest {rng.min():.2f} m '
                   f'(lowest ring meets the ground at ~{LIDAR_HEIGHT / math.tan(math.radians(15)):.1f} m)')
        horiz = np.hypot(pts[:, 0], pts[:, 1])
        ground = pts[(horiz < 3.5) & (pts[:, 2] < -0.3)]
        gz = float(np.median(ground[:, 2])) if len(ground) else float('nan')
        self.check('3d lidar sees the ground at the right height',
                   len(ground) > 200 and abs(gz + LIDAR_HEIGHT) < 0.05,
                   f'{len(ground)} ground points within 3.5 m, median {gz:+.3f} m below sensor '
                   f'(expected {-LIDAR_HEIGHT:+.3f})')
        # canopy: points well above the trunks (> ~1.5 m above ground), near a known tree
        high = pts[pts[:, 2] > 0.85]
        c, s_ = math.cos(gt_yaw), math.sin(gt_yaw)
        lx0 = gt.x + LIDAR_XYZ[0] * c
        ly0 = gt.y + LIDAR_XYZ[0] * s_
        wx = lx0 + c * high[:, 0] - s_ * high[:, 1]
        wy = ly0 + s_ * high[:, 0] + c * high[:, 1]
        trees = np.array([[t['x'], t['y']] for t in self.meta['trees']])
        near = np.min(np.hypot(wx[:, None] - trees[None, :, 0], wy[:, None] - trees[None, :, 1]), axis=1) \
            if len(high) else np.array([])
        frac = float(np.mean(near < 2.0)) if len(near) else 0.0
        self.check('3d lidar sees olive canopies above the trunks', len(high) > 100 and frac > 0.95,
                   f'{len(high)} points > {LIDAR_HEIGHT + 0.85:.2f} m above ground, '
                   f'{frac * 100:.1f} % within 2 m of a known tree')

        # ---- derived 2D /scan vs known trunk positions
        scan = self.last['scan']
        x0, y0 = gt.x, gt.y
        lx = x0 + LIDAR_XYZ[0] * math.cos(gt_yaw)
        ly = y0 + LIDAR_XYZ[0] * math.sin(gt_yaw)
        errors, hits = [], 0
        for t in self.meta['trees']:
            dx, dy = t['x'] - lx, t['y'] - ly
            d = math.hypot(dx, dy)
            if d - t['trunk_radius'] > scan.range_max - 0.5:
                continue
            bearing = math.atan2(dy, dx) - gt_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            i = int(round((bearing - scan.angle_min) / scan.angle_increment)) % len(scan.ranges)
            # trunk surface is the closest return within +-1 beam
            r = min(scan.ranges[(i + k) % len(scan.ranges)] for k in (-1, 0, 1))
            expect = d - t['trunk_radius']
            if math.isfinite(r) and r < scan.range_max:
                errors.append(abs(r - expect))
                hits += 1
        visible = len(errors)
        med = float(np.median(errors)) if errors else float('inf')
        self.check('/scan (from 3d cloud) sees trunks at known positions', hits >= 5 and med < 0.10,
                   f'{hits} trunks in range matched, median range error {med:.3f} m, '
                   f'max {max(errors) if errors else float("nan"):.3f} m')
        finite = [r for r in scan.ranges if math.isfinite(r) and r < scan.range_max]
        self.check('/scan has no self-hits', min(finite) > 0.5 if finite else False,
                   f'closest return {min(finite):.2f} m' if finite else 'no returns')

        # ---- camera image
        img = self.last['camera']
        data = np.frombuffer(bytes(img.data), dtype=np.uint8)
        self.check('camera image', img.width == 640 and img.height == 480 and img.encoding == 'rgb8'
                   and data.std() > 10,
                   f'{img.width}x{img.height} {img.encoding}, pixel std {data.std():.1f} '
                   f'(blank image would be ~0), mean {data.mean():.0f}')

        # ---- RGB-D depth image (robot at rest at spawn, camera looking down the path)
        dimg, info = self.last['depth'], self.last['cinfo']
        self.check('depth image format',
                   dimg.encoding == '32FC1' and dimg.width == img.width and dimg.height == img.height
                   and dimg.header.frame_id == img.header.frame_id,
                   f'{dimg.width}x{dimg.height} {dimg.encoding}, frame {dimg.header.frame_id} '
                   f'(colour: {img.width}x{img.height}, frame {img.header.frame_id})')
        depth = np.frombuffer(bytes(dimg.data), dtype=np.float32).reshape(dimg.height, dimg.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        cyaw, syaw = math.cos(gt_yaw), math.sin(gt_yaw)
        camx, camy = gt.x + CAM_X * cyaw, gt.y + CAM_X * syaw
        cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
        derr, seen = [], []
        for t in self.meta['trees']:
            # aim at the trunk 0.6 m above the ground (trunks are 0.9-1.2 m tall)
            dxw, dyw, dz = t['x'] - camx, t['y'] - camy, 0.6 - CAM_H
            bx = cyaw * dxw + syaw * dyw          # body frame: x forward, y left, z up
            by = -syaw * dxw + cyaw * dyw
            xc = cp * bx - sp * dz                # tilt the camera down by CAM_PITCH
            zc = sp * bx + cp * dz
            if xc <= DEPTH_MIN or xc - t['trunk_radius'] > DEPTH_MAX - 0.3:
                continue
            u = int(round(cx - fx * by / xc))
            v = int(round(cy - fy * zc / xc))
            if not (3 <= u < dimg.width - 3 and 3 <= v < dimg.height - 3):
                continue
            patch = depth[v - 3:v + 4, u - 3:u + 4]
            patch = patch[np.isfinite(patch)]
            if len(patch) == 0:
                continue
            measured = float(patch.min())               # trunk front surface
            expected = xc - t['trunk_radius']
            derr.append(abs(measured - expected))
            seen.append(f"{t['name']} {measured:.2f}/{expected:.2f} m @({u},{v})")
        self.check('depth matches true distance to known trunks', len(derr) >= 2 and max(derr) < 0.15,
                   f'{len(derr)} trunks in view: ' + '; '.join(seen) + ' (measured/expected)')
        sky = depth[5:40, dimg.width // 2 - 20:dimg.width // 2 + 20]
        ground = depth[dimg.height - 40:dimg.height - 5, dimg.width // 2 - 20:dimg.width // 2 + 20]
        self.check('depth: sky = no return, near ground in range',
                   np.mean(~np.isfinite(sky) | (sky >= DEPTH_MAX)) > 0.9
                   and np.all(np.isfinite(ground)) and DEPTH_MIN < float(np.median(ground)) < 3.0,
                   f'sky patch {100 * np.mean(~np.isfinite(sky) | (sky >= DEPTH_MAX)):.0f} % no return, '
                   f'ground patch median {float(np.median(ground)):.2f} m')

        # ---- IMU yaw rate while spinning
        cmd = Twist()
        cmd.angular.z = 0.8
        self.spin_for(1.0, cmd)                 # get through the acceleration ramp
        self.imu_log = []
        self.spin_for(3.0, cmd)
        log, self.imu_log = self.imu_log, None
        self.spin_for(2.0, Twist())
        imu_w = float(np.mean([a for a, _ in log]))
        gt_w = float(np.mean([b for _, b in log]))
        self.check('imu yaw rate matches true rotation', abs(imu_w - gt_w) < 0.05,
                   f'imu {imu_w:+.3f} rad/s, true {gt_w:+.3f} rad/s (commanded +0.8)')

        # ---- GPS follows motion: drive 3 m forward along the path
        gt0 = self.last['gt'].pose.pose
        y0 = yaw_of(gt0.orientation)
        self.spin_for(1.0, Twist())
        p0 = np.mean([self.latlon_to_xy(self.last['gps'].latitude, self.last['gps'].longitude)
                      for _ in range(1)], axis=0)
        fwd = Twist()
        fwd.linear.x = 0.6
        # heading may not be exactly along the path after the spin; drive a short distance only
        self.spin_for(5.0, fwd)
        self.spin_for(2.0, Twist())
        gt1 = self.last['gt'].pose.pose
        p1 = self.latlon_to_xy(self.last['gps'].latitude, self.last['gps'].longitude)
        true_d = math.hypot(gt1.position.x - gt0.position.x, gt1.position.y - gt0.position.y)
        gps_d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        self.check('gps follows motion', abs(gps_d - true_d) < 1.0 and true_d > 2.0,
                   f'gps moved {gps_d:.2f} m, robot moved {true_d:.2f} m (noise 0.3 m/fix)')

        passed = sum(ok for _, ok, _ in self.results)
        self.get_logger().info(f'==== {passed}/{len(self.results)} checks passed ====')
        return passed == len(self.results)


def main():
    rclpy.init()
    node = SensorTest()
    try:
        ok = node.run()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
