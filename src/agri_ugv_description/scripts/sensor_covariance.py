#!/usr/bin/env python3
"""Add the measurement uncertainty that the EKFs need to the Gazebo sensors.

Gazebo publishes wheel odometry and IMU data with empty covariances, which an
EKF would read as "perfect". This node republishes them with realistic values.

Topics:
  in   /odom         wheel odometry (Gazebo)
  in   /imu          IMU (Gazebo)
  out  /wheel/odom   same + twist covariance
  out  /imu/data     same + orientation and angular velocity covariance
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def diagonal(values):
    n = len(values)
    return [values[i] if i == j else 0.0 for i in range(n) for j in range(n)]


class SensorCovariance(Node):
    def __init__(self):
        super().__init__('sensor_covariance')
        v = self.declare_parameter('wheel_speed_sigma', 0.05).value        # m/s
        # wheels cannot see the robot sliding sideways on a slope. If they are
        # trusted here, the EKF explains the slide as a heading error
        vy = self.declare_parameter('wheel_lateral_sigma', 0.30).value     # m/s
        a = self.declare_parameter('imu_tilt_sigma', 0.01).value           # rad, roll and pitch
        g = self.declare_parameter('imu_rate_sigma', 0.02).value           # rad/s
        self.twist_cov = diagonal([v ** 2, vy ** 2, vy ** 2, 1e6, 1e6, 1e6])
        self.tilt_cov = diagonal([a ** 2, a ** 2, 1e6])
        self.rate_cov = diagonal([g ** 2] * 3)

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

    def on_odom(self, msg):
        msg.twist.covariance = self.twist_cov
        self.odom_pub.publish(msg)

    def on_imu(self, msg):
        msg.orientation_covariance = self.tilt_cov
        msg.angular_velocity_covariance = self.rate_cov
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(SensorCovariance())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # normal stop (Ctrl+C / launch shutdown)


if __name__ == '__main__':
    main()
