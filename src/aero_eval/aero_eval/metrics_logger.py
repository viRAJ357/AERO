#!/usr/bin/env python3
"""Metrics logger ROS 2 node.

Continuously computes and writes system metrics to CSV:
- Map coverage percentage over time
- Cumulative path length (odometry)
- Collision events (min LiDAR distance < 0.12 m)
- Object localization error vs config/ground_truth.json
- CPU and RAM consumption
"""
import csv
import json
import math
import os
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    from aero_interfaces.msg import SemanticObjectArray
    HAS_INTERFACES = True
except ImportError:
    HAS_INTERFACES = False


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('log_dir', os.path.expanduser('~/.aero/metrics'))
        self.declare_parameter('gt_path', '')
        self.declare_parameter('sample_interval', 1.0)

        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f'metrics_{int(time.time())}.csv')

        self._lock = threading.Lock()
        self.prev_pose: Optional[tuple] = None
        self.total_path_length = 0.0
        self.collision_events = 0
        self.coverage_percent = 0.0
        self.last_lidar_min = float('inf')

        # Load ground truth
        self.ground_truth = []
        gt_param = self.get_parameter('gt_path').value
        if gt_param and os.path.exists(gt_param):
            try:
                with open(gt_param, 'r') as f:
                    self.ground_truth = json.load(f).get('objects', [])
            except Exception:
                pass

        self.confirmed_objects: List[dict] = []

        # Setup CSV
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'sim_time', 'coverage_pct', 'path_length_m', 'collision_events',
            'min_lidar_dist_m', 'cpu_pct', 'ram_mb', 'median_loc_error_m'
        ])
        self.csv_file.flush()

        # Subscribers
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        if HAS_INTERFACES:
            self.create_subscription(SemanticObjectArray, '/aero/semantic_objects', self._objects_cb, 10)

        # Logging Timer
        interval = self.get_parameter('sample_interval').value
        self.create_timer(interval, self._log_sample)

        self.get_logger().info(f'MetricsLoggerNode writing to: {self.csv_path}')

    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        if valid:
            m = min(valid)
            with self._lock:
                self.last_lidar_min = m
                if m < 0.12:  # Collision condition
                    self.collision_events += 1

    def _odom_cb(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        with self._lock:
            if self.prev_pose is not None:
                dx = px - self.prev_pose[0]
                dy = py - self.prev_pose[1]
                self.total_path_length += math.sqrt(dx * dx + dy * dy)
            self.prev_pose = (px, py)

    def _map_cb(self, msg: OccupancyGrid):
        # Calculate coverage: known free space ratio
        data = np.array(msg.data)
        free_cells = np.sum((data >= 0) & (data < 25))
        total_explored = np.sum(data != -1)
        with self._lock:
            if total_explored > 0:
                self.coverage_percent = round(100.0 * float(free_cells) / float(len(data)), 2)

    def _objects_cb(self, msg):
        objs = []
        for o in msg.objects:
            objs.append({
                "label": o.label,
                "x": o.pose.pose.position.x,
                "y": o.pose.pose.position.y
            })
        with self._lock:
            self.confirmed_objects = objs

    def _compute_loc_error(self) -> float:
        if not self.ground_truth or not self.confirmed_objects:
            return 0.0
        errors = []
        for co in self.confirmed_objects:
            matches = [gt for gt in self.ground_truth if gt['label'] == co['label']]
            if matches:
                dists = [math.sqrt((gt['x'] - co['x'])**2 + (gt['y'] - co['y'])**2) for gt in matches]
                errors.append(min(dists))
        return float(np.median(errors)) if errors else 0.0

    def _log_sample(self):
        sim_time = self.get_clock().now().nanoseconds / 1e9
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().used / (1024 * 1024)

        with self._lock:
            cov = self.coverage_percent
            path = round(self.total_path_length, 2)
            cols = self.collision_events
            l_min = round(self.last_lidar_min, 3)
            loc_err = round(self._compute_loc_error(), 3)

        self.csv_writer.writerow([
            round(sim_time, 2), cov, path, cols, l_min, cpu, round(ram, 1), loc_err
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
