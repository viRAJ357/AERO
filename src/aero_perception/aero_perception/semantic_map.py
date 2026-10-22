#!/usr/bin/env python3
"""Semantic map node.

Associates 3D detections with existing objects (same label, within
assoc_radius), updates positions with confidence-weighted average,
confirms objects after min_observations observations.

Publishes SemanticObjectArray at 1 Hz, RViz MarkerArray, and provides
QueryObjects service. Saves/loads JSON to/from ~/.aero/semantic_map.json.
"""
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from aero_interfaces.msg import SemanticObject, SemanticObjectArray
    from aero_interfaces.srv import QueryObjects
    HAS_INTERFACES = True
except ImportError:
    HAS_INTERFACES = False


@dataclass
class TrackedObject:
    id: int
    label: str
    x: float
    y: float
    confidence: float
    observations: int = 1
    last_seen: float = field(default_factory=time.time)

    @property
    def confirmed(self) -> bool:
        return self.observations >= 3

    def update(self, x: float, y: float, conf: float) -> None:
        """Confidence-weighted position update."""
        w = conf / (self.confidence + conf)
        self.x = self.x * (1 - w) + x * w
        self.y = self.y * (1 - w) + y * w
        self.confidence = max(self.confidence, conf)
        self.observations += 1
        self.last_seen = time.time()


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map')

        self.declare_parameter('assoc_radius', 0.6)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('save_interval', 10.0)
        self.declare_parameter('load_previous', False)
        self.declare_parameter('save_path', os.path.expanduser('~/.aero/semantic_map.json'))

        self._assoc_radius = self.get_parameter('assoc_radius').value
        self._min_obs = self.get_parameter('min_observations').value
        self._save_path = Path(self.get_parameter('save_path').value)
        self._save_path.parent.mkdir(parents=True, exist_ok=True)

        self._lock = threading.Lock()
        self._objects: Dict[int, TrackedObject] = {}
        self._next_id = 0

        if self.get_parameter('load_previous').value:
            self._load_from_json()

        # Subscriptions
        self.create_subscription(
            PoseStamped, '/aero/detections3d', self._detection_cb, 10)
        # Note: In production wire this to SemanticObject messages with label
        # Here we handle the pose-only variant with a label header workaround

        # Publishers
        self._obj_pub = self.create_publisher(
            SemanticObjectArray if HAS_INTERFACES else Header,
            '/aero/semantic_objects', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/aero/object_markers', 10)

        # Timers
        self.create_timer(1.0, self._publish_cb)
        self.create_timer(
            self.get_parameter('save_interval').value, self._save_to_json)

        # QueryObjects service
        if HAS_INTERFACES:
            self._query_srv = self.create_service(
                QueryObjects, '/aero/query_objects', self._query_cb)

        self.get_logger().info('SemanticMapNode started')

    def add_observation(self, label: str, x: float, y: float,
                        conf: float) -> None:
        """Add or update a tracked object. Thread-safe."""
        with self._lock:
            # Find best match: same label within assoc_radius
            best: Optional[TrackedObject] = None
            best_dist = float('inf')
            for obj in self._objects.values():
                if obj.label != label:
                    continue
                d = math.sqrt((obj.x - x) ** 2 + (obj.y - y) ** 2)
                if d < self._assoc_radius and d < best_dist:
                    best = obj
                    best_dist = d
            if best is not None:
                best.update(x, y, conf)
            else:
                self._objects[self._next_id] = TrackedObject(
                    id=self._next_id, label=label, x=x, y=y, confidence=conf)
                self._next_id += 1

    def _detection_cb(self, msg: PoseStamped) -> None:
        # In full version, subscribe to SemanticObjectArray from localizer
        # For now, detection label comes from user_data (frame_id workaround)
        label = msg.header.frame_id if msg.header.frame_id not in ('', 'map') else 'unknown'
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.add_observation(label, x, y, 0.5)

    def _publish_cb(self) -> None:
        with self._lock:
            confirmed = [o for o in self._objects.values() if o.confirmed]

        if HAS_INTERFACES:
            arr = SemanticObjectArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.header.frame_id = 'map'
            for o in confirmed:
                so = SemanticObject()
                so.id = o.id
                so.label = o.label
                so.confidence = o.confidence
                so.observations = o.observations
                so.pose.header.frame_id = 'map'
                so.pose.pose.position.x = o.x
                so.pose.pose.position.y = o.y
                so.pose.pose.orientation.w = 1.0
                arr.objects.append(so)
            self._obj_pub.publish(arr)

        # Markers
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for o in confirmed:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'semantic_objects'
            m.id = o.id
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = o.x
            m.pose.position.y = o.y
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.3
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)
            m.text = f'{o.label}\n({o.observations}obs)'
            markers.markers.append(m)

            # Sphere
            sm = Marker()
            sm.header = m.header
            sm.ns = 'semantic_spheres'
            sm.id = o.id
            sm.type = Marker.SPHERE
            sm.action = Marker.ADD
            sm.pose.position.x = o.x
            sm.pose.position.y = o.y
            sm.pose.position.z = 0.1
            sm.pose.orientation.w = 1.0
            sm.scale.x = sm.scale.y = sm.scale.z = 0.2
            sm.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            markers.markers.append(sm)

        self._marker_pub.publish(markers)

    def _query_cb(self, request, response):
        with self._lock:
            objs = list(self._objects.values())
        label_filter = request.label.strip()
        conf_filter = request.min_confidence
        results = []
        for o in objs:
            if not o.confirmed:
                continue
            if label_filter and o.label != label_filter:
                continue
            if o.confidence < conf_filter:
                continue
            results.append(o)
        response.success = True
        if HAS_INTERFACES:
            for o in results:
                so = SemanticObject()
                so.id = o.id
                so.label = o.label
                so.confidence = o.confidence
                so.observations = o.observations
                so.pose.header.frame_id = 'map'
                so.pose.pose.position.x = o.x
                so.pose.pose.position.y = o.y
                so.pose.pose.orientation.w = 1.0
                response.objects.append(so)
        return response

    def _save_to_json(self) -> None:
        with self._lock:
            data = [
                {'id': o.id, 'label': o.label, 'x': o.x, 'y': o.y,
                 'confidence': o.confidence, 'observations': o.observations}
                for o in self._objects.values()
            ]
        try:
            with open(self._save_path, 'w') as f:
                json.dump({'objects': data, 'next_id': self._next_id}, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save semantic map: {e}')

    def _load_from_json(self) -> None:
        if not self._save_path.exists():
            return
        try:
            with open(self._save_path) as f:
                data = json.load(f)
            for o in data.get('objects', []):
                obj = TrackedObject(
                    id=o['id'], label=o['label'],
                    x=o['x'], y=o['y'],
                    confidence=o['confidence'],
                    observations=o['observations'],
                )
                self._objects[obj.id] = obj
            self._next_id = data.get('next_id', len(self._objects))
            self.get_logger().info(
                f'Loaded {len(self._objects)} objects from {self._save_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load semantic map: {e}')

    def destroy_node(self) -> None:
        self._save_to_json()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
