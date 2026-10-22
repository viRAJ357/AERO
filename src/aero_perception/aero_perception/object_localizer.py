#!/usr/bin/env python3
"""Object localizer: converts 2D bounding boxes to 3D map-frame positions.

Method: bearing from camera intrinsics + median LiDAR range in bbox angular
window → transform to map frame with tf2.
"""
import threading
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 – registers transform types

from aero_perception.geometry_utils import (
    bearing_from_pixel, angular_half_width, median_range, range_spread
)

try:
    from aero_interfaces.msg import SemanticObject, SemanticObjectArray
except ImportError:
    SemanticObject = None
    SemanticObjectArray = None


class ObjectLocalizerNode(Node):
    def __init__(self):
        super().__init__('object_localizer')

        self.declare_parameter('rmin', 0.3)
        self.declare_parameter('rmax', 4.0)
        self.declare_parameter('max_spread', 0.8)

        self._rmin = self.get_parameter('rmin').value
        self._rmax = self.get_parameter('rmax').value
        self._max_spread = self.get_parameter('max_spread').value

        self._lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None
        self._camera_info: Optional[CameraInfo] = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)
        self.create_subscription(
            Detection2DArray, '/aero/detections2d', self._detection_cb, 10)

        self._pub = self.create_publisher(PoseStamped, '/aero/detections3d', 10)
        # If aero_interfaces available, publish full detections
        if SemanticObjectArray:
            self._obj_pub = self.create_publisher(
                SemanticObjectArray, '/aero/detections3d_full', 10)

        self.get_logger().info('ObjectLocalizerNode started')

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        with self._lock:
            self._camera_info = msg

    def _detection_cb(self, msg: Detection2DArray) -> None:
        with self._lock:
            scan = self._latest_scan
            info = self._camera_info

        if scan is None or info is None:
            return

        # Camera intrinsics
        fx = info.k[0]   # K[0,0]
        cx = info.k[2]   # K[0,2]

        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score

            u_center = det.bbox.center.position.x
            u_left = u_center - det.bbox.size_x / 2.0
            u_right = u_center + det.bbox.size_x / 2.0

            bearing = bearing_from_pixel(u_center, cx, fx)
            half_w = angular_half_width(u_left, u_right, cx, fx)

            ranges = list(scan.ranges)
            r = median_range(ranges, scan.angle_min, scan.angle_increment,
                             bearing, half_w, self._rmin, self._rmax)

            if r is None:
                continue

            spread = range_spread(ranges, scan.angle_min, scan.angle_increment,
                                  bearing, half_w, self._rmin, self._rmax)
            if spread is not None and spread > self._max_spread:
                continue  # noisy reading

            # Build a point in laser frame
            import math
            import geometry_msgs.msg as gm
            ps_laser = PoseStamped()
            ps_laser.header.stamp = det.header.stamp
            ps_laser.header.frame_id = scan.header.frame_id
            ps_laser.pose.position.x = r * math.cos(-bearing)
            ps_laser.pose.position.y = r * math.sin(-bearing)
            ps_laser.pose.position.z = 0.0
            ps_laser.pose.orientation.w = 1.0

            # Transform to map frame
            try:
                ps_map = self._tf_buffer.transform(
                    ps_laser, 'map',
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception:
                # Fall back: try with latest transform if within 0.2 s
                try:
                    ps_laser.header.stamp = rclpy.time.Time().to_msg()
                    ps_map = self._tf_buffer.transform(
                        ps_laser, 'map',
                        timeout=rclpy.duration.Duration(seconds=0.05)
                    )
                except Exception as e:
                    self.get_logger().debug(f'TF error for {label}: {e}')
                    continue

            ps_map.header.frame_id = 'map'
            self._pub.publish(ps_map)

            self.get_logger().debug(
                f'Localized {label} ({conf:.2f}) at '
                f'({ps_map.pose.position.x:.2f}, {ps_map.pose.position.y:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
