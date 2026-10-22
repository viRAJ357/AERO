#!/usr/bin/env python3
"""Frontier explorer ROS 2 node.

Runs only when mode == EXPLORE. Selects frontier goals, sends them to Nav2
via NavigateToPose, times out after 60 s, blacklists failed goals.

Finish rule: no valid frontier for 5 consecutive cycles → mode DONE.
"""
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros

from nav2_msgs.action import NavigateToPose

from aero_exploration.frontier_utils import (
    find_frontiers, cluster_centroid, snap_to_free,
    score_frontier, world_to_grid, grid_to_world
)

try:
    from aero_interfaces.srv import SetMode
    HAS_SET_MODE = True
except ImportError:
    HAS_SET_MODE = False

MODE_IDLE = 'IDLE'
MODE_EXPLORE = 'EXPLORE'
MODE_STOP = 'STOP'
MODE_DONE = 'DONE'
MODE_NAVIGATE = 'NAVIGATE'


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('goal_timeout_s', 60.0)
        self.declare_parameter('blacklist_radius_m', 0.5)
        self.declare_parameter('replan_threshold', 50)   # cells changed
        self.declare_parameter('empty_cycles_done', 5)
        self.declare_parameter('plan_interval_s', 3.0)

        self._timeout_s = self.get_parameter('goal_timeout_s').value
        self._bl_radius = self.get_parameter('blacklist_radius_m').value
        self._replan_thresh = self.get_parameter('replan_threshold').value
        self._empty_done = self.get_parameter('empty_cycles_done').value
        self._plan_interval = self.get_parameter('plan_interval_s').value

        self._lock = threading.Lock()
        self._mode = MODE_IDLE
        self._latest_map: Optional[OccupancyGrid] = None
        self._map_change_count = 0
        self._blacklist: List[Tuple[float, float]] = []  # (x, y) world
        self._current_goal: Optional[Tuple[float, float]] = None
        self._goal_start: float = 0.0
        self._empty_cycles: int = 0
        self._nav_active: bool = False
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Map QoS
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(String, '/aero/mode', self._mode_cb, 10)

        # Publishers
        self._mode_pub = self.create_publisher(String, '/aero/mode', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/aero/current_goal', 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # SetMode service
        if HAS_SET_MODE:
            self._set_mode_srv = self.create_service(
                SetMode, '/aero/set_mode', self._set_mode_cb)

        # Planning timer
        self.create_timer(self._plan_interval, self._plan_cb)

        self.get_logger().info('FrontierExplorerNode started (mode=IDLE)')

    # ---- Callbacks ----

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._lock:
            if self._latest_map is not None:
                old = np.array(self._latest_map.data)
                new = np.array(msg.data)
                # Count newly revealed cells
                revealed = np.sum((old == -1) & (new != -1))
                self._map_change_count += int(revealed)
            self._latest_map = msg

    def _mode_cb(self, msg: String) -> None:
        with self._lock:
            new_mode = msg.data.upper()
            if new_mode != self._mode:
                self.get_logger().info(f'Mode: {self._mode} → {new_mode}')
                self._mode = new_mode
                if new_mode == MODE_STOP:
                    self._cancel_current_goal()

    def _set_mode_cb(self, request, response):
        mode = request.mode.upper()
        if mode not in (MODE_IDLE, MODE_EXPLORE, MODE_STOP):
            response.success = False
            response.message = f'Unknown mode: {mode}'
            return response
        with self._lock:
            self._mode = mode
            if mode == MODE_STOP:
                self._cancel_current_goal()
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)
        response.success = True
        response.message = f'Mode set to {mode}'
        return response

    # ---- Planning ----

    def _plan_cb(self) -> None:
        with self._lock:
            mode = self._mode
            ogrid = self._latest_map
            nav_active = self._nav_active
            goal_start = self._goal_start
            current_goal = self._current_goal

        if mode != MODE_EXPLORE:
            return

        # Goal timeout check
        if nav_active and current_goal is not None:
            if time.time() - goal_start > self._timeout_s:
                self.get_logger().warn(
                    f'Goal {current_goal} timed out after {self._timeout_s}s, blacklisting')
                with self._lock:
                    self._blacklist.append(current_goal)
                    self._nav_active = False
                    self._current_goal = None
                self._cancel_current_goal()
            else:
                return  # Still navigating

        if ogrid is None:
            return

        # Get robot pose
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception as e:
            self.get_logger().debug(f'TF error: {e}')
            return

        # Convert map to numpy grid
        width = ogrid.info.width
        height = ogrid.info.height
        res = ogrid.info.resolution
        orig_x = ogrid.info.origin.position.x
        orig_y = ogrid.info.origin.position.y
        grid = np.array(ogrid.data, dtype=np.int16).reshape(height, width)

        # Find frontiers
        clusters = find_frontiers(grid)
        if not clusters:
            with self._lock:
                self._empty_cycles += 1
                ec = self._empty_cycles
            if ec >= self._empty_done:
                self.get_logger().info(
                    'No frontiers for 5 cycles — exploration DONE')
                self._set_mode(MODE_DONE)
            return

        with self._lock:
            self._empty_cycles = 0

        # Robot in grid coords
        robot_row, robot_col = world_to_grid(rx, ry, orig_x, orig_y, res)

        # Score clusters, pick best non-blacklisted
        best_goal = None
        best_score = -float('inf')
        for cells in clusters:
            cr, cc = cluster_centroid(cells)
            snapped = snap_to_free(int(cr), int(cc), grid)
            if snapped is None:
                continue
            gx, gy = grid_to_world(snapped[0], snapped[1], orig_x, orig_y, res)
            if self._is_blacklisted(gx, gy):
                continue
            score = score_frontier(cells, (cr, cc), robot_row, robot_col)
            if score > best_score:
                best_score = score
                best_goal = (gx, gy)

        if best_goal is None:
            with self._lock:
                self._empty_cycles += 1
            return

        self._send_goal(best_goal[0], best_goal[1])

    def _is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by in self._blacklist:
            if math.sqrt((x - bx) ** 2 + (y - by) ** 2) < self._bl_radius:
                return True
        return False

    def _send_goal(self, x: float, y: float) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        self.get_logger().info(f'Sending frontier goal: ({x:.2f}, {y:.2f})')
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

        with self._lock:
            self._current_goal = (x, y)
            self._goal_start = time.time()
            self._nav_active = True

        # Publish goal for dashboard
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self._goal_pub.publish(ps)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            with self._lock:
                if self._current_goal:
                    self._blacklist.append(self._current_goal)
                self._nav_active = False
                self._current_goal = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)
        self._goal_handle = goal_handle

    def _goal_result_cb(self, future):
        result = future.result()
        with self._lock:
            cg = self._current_goal
            self._nav_active = False
            self._current_goal = None
        if result.status != 4:   # 4 = SUCCEEDED in action_msgs
            self.get_logger().warn(f'Goal {cg} failed with status {result.status}, blacklisting')
            if cg:
                with self._lock:
                    self._blacklist.append(cg)
        else:
            self.get_logger().info(f'Goal {cg} reached successfully')

    def _nav_feedback_cb(self, feedback_msg):
        pass  # Could use remaining distance to detect stuck robot

    def _cancel_current_goal(self) -> None:
        try:
            if hasattr(self, '_goal_handle') and self._goal_handle:
                self._goal_handle.cancel_goal_async()
        except Exception:
            pass

    def _set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
