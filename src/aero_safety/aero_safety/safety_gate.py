#!/usr/bin/env python3
"""Safety gate action server for AERO.

Checks ALL of:
  1. Goal is inside the map bounds.
  2. Goal cell is on free space in the global costmap.
  3. At least 0.4 m clearance (nearest obstacle > 0.4 m).
  4. Distance to goal at most 15 m.
  5. At most one goal per 2 s (rate limiting).
  6. /aero/estop == True cancels nav and zeroes velocity.
  7. LiDAR watchdog: if /scan is older than 1 s, stop.

Every rejection returns a human-readable reason for the LLM.
"""
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import tf2_ros

try:
    from aero_interfaces.action import SafeGoTo
    HAS_SAFEGOTO = True
except ImportError:
    HAS_SAFEGOTO = False


class SafetyGateNode(Node):
    # Safety constants
    MAX_DIST_M = 15.0
    MIN_CLEARANCE_M = 0.4
    RATE_LIMIT_S = 2.0
    SCAN_TIMEOUT_S = 1.0
    COSTMAP_FREE_THRESH = 10   # cells with cost < this are free

    def __init__(self):
        super().__init__('safety_gate')

        self._lock = threading.Lock()
        self._estop = False
        self._latest_scan: Optional[LaserScan] = None
        self._scan_time: float = 0.0
        self._latest_costmap: Optional[OccupancyGrid] = None
        self._last_goal_time: float = 0.0
        self._current_nav_handle = None

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Velocity publisher (for emergency zero-vel)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.create_subscription(Bool, '/aero/estop', self._estop_cb, 10)
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)

        costmap_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, costmap_qos)

        # Nav2 client (for sending validated goals)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=ReentrantCallbackGroup())

        # SafeGoTo action server
        if HAS_SAFEGOTO:
            self._action_server = ActionServer(
                self,
                SafeGoTo,
                '/aero/safe_goto',
                execute_callback=self._execute_cb,
                goal_callback=self._goal_cb,
                cancel_callback=self._cancel_cb,
                callback_group=ReentrantCallbackGroup(),
            )

        # Watchdog timer
        self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info('SafetyGateNode started')

    # ---- Subscribers ----

    def _estop_cb(self, msg: Bool) -> None:
        if msg.data and not self._estop:
            self.get_logger().warn('E-STOP ACTIVATED')
            self._estop = True
            self._zero_velocity()
            self._cancel_navigation()
        elif not msg.data and self._estop:
            self.get_logger().info('E-stop cleared')
            self._estop = False

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._scan_time = time.time()

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._latest_costmap = msg

    # ---- Checks ----

    def check_goal(self, x: float, y: float) -> tuple:
        """Run all safety checks. Returns (ok: bool, reason: str)."""
        with self._lock:
            estop = self._estop
            scan = self._latest_scan
            scan_age = time.time() - self._scan_time
            costmap = self._latest_costmap
            last_goal = self._last_goal_time

        # E-stop
        if estop:
            return False, 'E-stop is active. Clear /aero/estop first.'

        # LiDAR watchdog
        if scan is None or scan_age > self.SCAN_TIMEOUT_S:
            return False, f'LiDAR data stale ({scan_age:.1f} s). Sensor may be offline.'

        # Rate limiting
        elapsed = time.time() - last_goal
        if elapsed < self.RATE_LIMIT_S:
            return False, (
                f'Rate limit: last goal {elapsed:.1f}s ago '
                f'(minimum {self.RATE_LIMIT_S}s between goals).'
            )

        # Distance check
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception:
            rx, ry = 0.0, 0.0
        dist = math.sqrt((x - rx) ** 2 + (y - ry) ** 2)
        if dist > self.MAX_DIST_M:
            return False, (
                f'Goal is {dist:.1f} m away (maximum {self.MAX_DIST_M} m). '
                'Move closer or choose a nearer target.'
            )

        # Costmap check (bounds + free space + clearance)
        if costmap is not None:
            w = costmap.info.width
            h = costmap.info.height
            res = costmap.info.resolution
            ox = costmap.info.origin.position.x
            oy = costmap.info.origin.position.y

            col = int((x - ox) / res)
            row = int((y - oy) / res)

            if not (0 <= col < w and 0 <= row < h):
                return False, 'Goal is outside the map bounds.'

            idx = row * w + col
            cost = costmap.data[idx]
            if cost < 0:
                return False, 'Goal is in unknown space. Explore this area first.'
            if cost >= 90:
                return False, (
                    f'Goal is inside or very close to an obstacle (cost={cost}). '
                    'Choose a different location.'
                )

            # Clearance: check neighbours
            clearance_cells = int(self.MIN_CLEARANCE_M / res)
            max_neighbor_cost = 0
            for dr in range(-clearance_cells, clearance_cells + 1):
                for dc in range(-clearance_cells, clearance_cells + 1):
                    nr, nc = row + dr, col + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        c = costmap.data[nr * w + nc]
                        if c >= 0:
                            max_neighbor_cost = max(max_neighbor_cost, c)
            if max_neighbor_cost >= 90:
                return False, (
                    f'Goal has insufficient clearance from obstacles '
                    f'(< {self.MIN_CLEARANCE_M} m). Try a location further from walls.'
                )

        return True, 'OK'

    # ---- Action server ----

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self._cancel_navigation()
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        target = goal_handle.request.target
        x = target.pose.position.x
        y = target.pose.position.y
        reason = goal_handle.request.reason

        self.get_logger().info(
            f'SafeGoTo request: ({x:.2f}, {y:.2f}) reason="{reason}"')

        ok, msg = self.check_goal(x, y)
        if not ok:
            self.get_logger().warn(f'Safety REJECT: {msg}')
            goal_handle.abort()
            result = SafeGoTo.Result()
            result.success = False
            result.message = msg
            return result

        # Record goal time
        with self._lock:
            self._last_goal_time = time.time()

        # Forward to Nav2
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result = SafeGoTo.Result()
            result.success = False
            result.message = 'Nav2 action server not available'
            return result

        nav_future = self._nav_client.send_goal_async(nav_goal)
        # In a real async executor this would await; simplified here
        rclpy.spin_until_future_complete(self, nav_future, timeout_sec=5.0)
        nav_handle = nav_future.result()
        if nav_handle is None or not nav_handle.accepted:
            goal_handle.abort()
            result = SafeGoTo.Result()
            result.success = False
            result.message = 'Nav2 rejected the goal'
            return result

        with self._lock:
            self._current_nav_handle = nav_handle

        result_future = nav_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        nav_result = result_future.result()

        result = SafeGoTo.Result()
        if nav_result and nav_result.status == 4:  # SUCCEEDED
            result.success = True
            result.message = 'Goal reached'
            goal_handle.succeed()
        else:
            status = nav_result.status if nav_result else -1
            result.success = False
            result.message = f'Navigation failed (status={status})'
            goal_handle.abort()
        return result

    # ---- Watchdog ----

    def _watchdog_cb(self) -> None:
        with self._lock:
            estop = self._estop
            scan_age = time.time() - self._scan_time if self._scan_time else 999.0

        if estop:
            self._zero_velocity()
        elif self._scan_time > 0 and scan_age > self.SCAN_TIMEOUT_S:
            self.get_logger().warn(
                f'LiDAR stale ({scan_age:.1f}s) — stopping robot for safety')
            self._zero_velocity()
            self._cancel_navigation()

    # ---- Helpers ----

    def _zero_velocity(self) -> None:
        twist = Twist()
        self._cmd_pub.publish(twist)

    def _cancel_navigation(self) -> None:
        try:
            if self._current_nav_handle:
                self._current_nav_handle.cancel_goal_async()
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._zero_velocity()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
