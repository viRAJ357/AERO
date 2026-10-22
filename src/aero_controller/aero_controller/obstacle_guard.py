#!/usr/bin/env python3
"""Reactive obstacle guard node for TurtleBot3 waffle_pi.

Subscribes to /scan, monitors frontal and lateral LiDAR beams, drives forward
when clear, and turns away when obstacles are closer than stop_distance (default 0.45 m).
Gracefully handles both Twist and TwistStamped on /cmd_vel based on parameter.
Publishes zero velocity on shutdown.
"""
import math
import sys
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('obstacle_guard')

        # Parameters
        self.declare_parameter('stop_distance', 0.45)        # metres
        self.declare_parameter('slow_distance', 0.80)        # metres
        self.declare_parameter('forward_speed', 0.18)        # m/s
        self.declare_parameter('turn_speed', 0.50)           # rad/s
        self.declare_parameter('cmd_vel_type', 'Twist')      # 'Twist' or 'TwistStamped'
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.stop_dist = self.get_parameter('stop_distance').value
        self.slow_dist = self.get_parameter('slow_distance').value
        self.fwd_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.cmd_type = self.get_parameter('cmd_vel_type').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        if self.cmd_type == 'TwistStamped':
            self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Subscriber
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos)

        # Control loop timer (10 Hz)
        self.latest_scan = None
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'ObstacleGuard initialized: stop_dist={self.stop_dist}m, '
            f'fwd_speed={self.fwd_speed}m/s, cmd_type={self.cmd_type}'
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _publish_velocity(self, linear_x: float, angular_z: float):
        if self.cmd_type == 'TwistStamped':
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        ranges = scan.ranges
        num_readings = len(ranges)
        if num_readings == 0:
            return

        # TB3 LiDAR: 0 is front, 90 is left, 180 is back, 270 is right
        # Sample sectors: Front (-30 deg to +30 deg), Left (30 to 75 deg), Right (285 to 330 deg)
        def get_sector_min(start_deg: int, end_deg: int) -> float:
            min_val = float('inf')
            deg_per_index = 360.0 / num_readings
            for i in range(num_readings):
                deg = i * deg_per_index
                if start_deg <= deg <= end_deg:
                    r = ranges[i]
                    if math.isfinite(r) and scan.range_min < r < scan.range_max:
                        if r < min_val:
                            min_val = r
            return min_val

        # Front is split across 0: 330..360 and 0..30
        front_r1 = get_sector_min(0, 30)
        front_r2 = get_sector_min(330, 360)
        front_dist = min(front_r1, front_r2)
        left_dist = get_sector_min(30, 80)
        right_dist = get_sector_min(280, 330)

        # Reactive navigation logic
        if front_dist < self.stop_dist:
            # Obstacle directly ahead: stop and turn towards the more open side
            turn_dir = 1.0 if left_dist > right_dist else -1.0
            self._publish_velocity(0.0, turn_dir * self.turn_speed)
            self.get_logger().info(
                f'Obstacle ahead ({front_dist:.2f}m < {self.stop_dist}m) -> turning'
            )
        elif front_dist < self.slow_dist:
            # Approaching obstacle: slow down and curve away
            turn_dir = 0.5 if left_dist > right_dist else -0.5
            self._publish_velocity(self.fwd_speed * 0.5, turn_dir * (self.turn_speed * 0.5))
        elif left_dist < (self.stop_dist * 0.8):
            # Left wall too close: steer right
            self._publish_velocity(self.fwd_speed * 0.8, -0.4 * self.turn_speed)
        elif right_dist < (self.stop_dist * 0.8):
            # Right wall too close: steer left
            self._publish_velocity(self.fwd_speed * 0.8, 0.4 * self.turn_speed)
        else:
            # Clear path: drive forward
            self._publish_velocity(self.fwd_speed, 0.0)

    def stop_robot(self):
        """Publish zero velocity command on shutdown."""
        self.get_logger().info('Stopping robot...')
        self._publish_velocity(0.0, 0.0)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt caught')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
