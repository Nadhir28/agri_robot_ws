#!/usr/bin/env python3
"""Derive a 2D LaserScan (/scan) from the 3D LiDAR point cloud (/points).

Keeps the points in a horizontal slab around the sensor
(min_height..max_height in the lidar frame) and, for every angular bin,
publishes the closest range. Same idea as the pointcloud_to_laserscan
package (not installed here), so slam_toolbox / Nav2 can keep using /scan.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.min_h = self.declare_parameter('min_height', -0.25).value   # m, lidar frame
        self.max_h = self.declare_parameter('max_height', 0.25).value
        self.bins = self.declare_parameter('bins', 900).value
        self.range_min = self.declare_parameter('range_min', 0.4).value
        self.range_max = self.declare_parameter('range_max', 30.0).value
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/points', self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, cloud):
        pts = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[(pts[:, 2] >= self.min_h) & (pts[:, 2] <= self.max_h)]
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = (r >= self.range_min) & (r <= self.range_max)
        r, pts = r[keep], pts[keep]

        inc = 2 * math.pi / self.bins
        idx = ((np.arctan2(pts[:, 1], pts[:, 0]) + math.pi) / inc).astype(int) % self.bins
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, r.astype(np.float32))

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
    rclpy.spin(CloudToScan())


if __name__ == '__main__':
    main()
