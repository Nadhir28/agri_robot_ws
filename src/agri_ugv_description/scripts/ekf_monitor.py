#!/usr/bin/env python3
"""Live view of the localization error, for demos.

Once per second, prints the true pose (Gazebo ground truth) and how far from it
are the wheels alone (/odom) and the EKF (/odometry/global).

  ros2 launch agri_ugv_description sim.launch.py world:=olive_orchard
  ros2 run agri_ugv_description ekf_monitor.py
  ros2 param set /rtk_gnss rtk_enabled false   # in another terminal: RTK off
"""
import bisect
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose2d(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fmt(d):
    return f'{100 * d:5.1f} cm' if d < 1.0 else f'{d:5.2f} m '


class EkfMonitor(Node):
    def __init__(self):
        super().__init__('ekf_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.wheel = self.ekf = self.fix = None
        self.wheel_origin = None         # true pose when the wheels started counting
        self.gt = deque(maxlen=200)      # recent ground truth, to match each estimate by its timestamp
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: self.gt.append((stamp_s(m), m)), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'wheel', m), 10)
        self.create_subscription(Odometry, '/odometry/global', lambda m: setattr(self, 'ekf', m), 10)
        self.create_subscription(NavSatFix, '/gps/fix', lambda m: setattr(self, 'fix', m), qos_profile_sensor_data)
        # wall clock: the simulation can run much slower than real time
        self.create_timer(1.0, self.show, clock=Clock(clock_type=ClockType.STEADY_TIME))
        print('  true x, y, heading        |  wheels: position, heading  |  EKF: position, heading  | GPS')

    def truth_at(self, t):
        times = [s for s, _ in self.gt]
        i = min(bisect.bisect_left(times, t), len(times) - 1)
        if i > 0 and t - times[i - 1] < times[i] - t:
            i -= 1
        return pose2d(self.gt[i][1])

    def wheels_in_world(self):
        """Wheel odometry starts at zero: place it on the true start pose."""
        x, y, yaw = pose2d(self.wheel)
        x0, y0, yaw0 = self.wheel_origin
        c, s = math.cos(yaw0), math.sin(yaw0)
        return x0 + c * x - s * y, y0 + s * x + c * y, yaw0 + yaw

    def show(self):
        if not self.gt or self.wheel is None or self.ekf is None or self.fix is None:
            print('  waiting for /ground_truth/odom, /odom, /odometry/global, /gps/fix ...', flush=True)
            return
        if self.wheel_origin is None:
            # the robot has not moved yet when the monitor starts on a new simulation
            self.wheel_origin = self.truth_at(stamp_s(self.wheel))
        tx, ty, tyaw = self.truth_at(stamp_s(self.ekf))
        ex, ey, eyaw = pose2d(self.ekf)
        wx, wy, wyaw = self.wheels_in_world()
        mode = 'RTK' if self.fix.status.status == NavSatStatus.STATUS_GBAS_FIX else 'RTK off'
        print(f'  {tx:7.2f} {ty:7.2f} {math.degrees(tyaw):7.1f} deg'
              f'  |  {fmt(math.hypot(wx - tx, wy - ty))}  {abs(math.degrees(wrap(wyaw - tyaw))):6.1f} deg'
              f'     |  {fmt(math.hypot(ex - tx, ey - ty))}  {abs(math.degrees(wrap(eyaw - tyaw))):5.2f} deg'
              f'  | {mode}', flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(EkfMonitor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
