#!/usr/bin/env python3
"""Derive a 2D LaserScan (/scan) from the 3D LiDAR point cloud (/points).

On rough ground a plain horizontal slice of the cloud also cuts the ground. So:
  1. the cloud is levelled with the roll and pitch of the IMU
  2. it is cut into small cells (angle, range). A cell holds something standing
     if its points span more than obstacle_height: the ground does not, because
     a single ring crosses a cell, or two on a slope
  3. for every angular bin, the closest standing point between min_height and
     max_height (m, around the sensor) is published

Same output as the pointcloud_to_laserscan package (not installed here), so
slam_toolbox / Nav2 can keep using /scan.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def level_rotation(q):
    """Rotation that removes the roll and pitch of orientation q, keeps the yaw."""
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ rot


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.min_h = self.declare_parameter('min_height', -0.45).value   # m, levelled lidar frame
        self.max_h = self.declare_parameter('max_height', 0.35).value    # below the canopy
        self.obstacle_h = self.declare_parameter('obstacle_height', 0.20).value
        self.cell_r = self.declare_parameter('cell_range', 0.5).value     # m
        self.cell_bins = self.declare_parameter('cell_bins', 360).value   # per turn
        # beam that hit nothing: inf, or this range (m) if > 0. slam_toolbox marks
        # free cells only along a beam with a finite range
        self.no_hit = self.declare_parameter('no_hit_range', 0.0).value
        self.bins = self.declare_parameter('bins', 900).value
        self.range_min = self.declare_parameter('range_min', 0.4).value
        self.range_max = self.declare_parameter('range_max', 30.0).value
        self.level = np.eye(3)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/points', self.on_cloud, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.level = level_rotation(msg.orientation)

    def standing(self, pts, r):
        """Two masks: the point is in a cell with something standing, and above that cell's ground."""
        az = ((np.arctan2(pts[:, 1], pts[:, 0]) + math.pi) / (2 * math.pi) * self.cell_bins).astype(int)
        cell = az % self.cell_bins * 1000 + (r / self.cell_r).astype(int)
        ids, inverse = np.unique(cell, return_inverse=True)
        low = np.full(len(ids), np.inf)
        high = np.full(len(ids), -np.inf)
        np.minimum.at(low, inverse, pts[:, 2])
        np.maximum.at(high, inverse, pts[:, 2])
        # the lowest points of the cell are the ground under the obstacle
        return (high - low)[inverse] > self.obstacle_h, pts[:, 2] > low[inverse] + 0.5 * self.obstacle_h

    def on_cloud(self, cloud):
        pts = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts @ self.level.T
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = (r >= self.range_min) & (r <= self.range_max) & (pts[:, 2] <= self.max_h)
        r, pts = r[keep], pts[keep]
        if len(r):
            cell_ok, above_ground = self.standing(pts, r)
            keep = cell_ok & above_ground & (pts[:, 2] >= self.min_h)
            r, pts = r[keep], pts[keep]

        inc = 2 * math.pi / self.bins
        idx = ((np.arctan2(pts[:, 1], pts[:, 0]) + math.pi) / inc).astype(int) % self.bins
        ranges = np.full(self.bins, self.no_hit or np.inf, dtype=np.float32)
        if len(r):
            # per-bin minimum: sort by (bin, range), first entry of each bin wins
            order = np.lexsort((r, idx))
            idx_s = idx[order]
            first = np.r_[True, idx_s[1:] != idx_s[:-1]]
            ranges[idx_s[first]] = r[order][first]

        scan = LaserScan()
        scan.header = cloud.header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - inc
        scan.angle_increment = inc
        scan.scan_time = 0.1
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    try:
        rclpy.spin(CloudToScan())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # normal stop (Ctrl+C / launch shutdown)


if __name__ == '__main__':
    main()
