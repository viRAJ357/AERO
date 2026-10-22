#!/usr/bin/env python3
"""YOLOv8 detector ROS 2 node.

Subscribes to /camera/image_raw, throttles to max_rate_hz, runs YOLOv8n,
publishes vision_msgs/Detection2DArray and annotated debug image.
"""
import time
import threading
from typing import List, Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header
from cv_bridge import CvBridge

from aero_perception.detector_interface import DetectorInterface, Detection


class YOLOv8Detector(DetectorInterface):
    """YOLOv8 wrapper implementing DetectorInterface."""

    def __init__(self, model_path: str, conf: float, imgsz: int,
                 device: str, allowed_classes: Optional[List[str]] = None):
        from ultralytics import YOLO
        import torch

        if device == 'auto':
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._device = device
        self._model = YOLO(model_path)
        self._conf = conf
        self._imgsz = imgsz
        self._allowed = set(allowed_classes) if allowed_classes else None

    def detect(self, image: np.ndarray) -> List[Detection]:
        results = self._model.predict(
            source=image,
            conf=self._conf,
            imgsz=self._imgsz,
            device=self._device,
            verbose=False,
        )
        detections: List[Detection] = []
        if not results:
            return detections
        for r in results:
            if r.boxes is None:
                continue
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = names.get(cls_id, str(cls_id))
                if self._allowed and label not in self._allowed:
                    continue
                conf = float(box.conf[0])
                xc, yc, w, h = box.xywh[0].tolist()
                detections.append(Detection(
                    label=label, confidence=conf,
                    x_center=xc, y_center=yc,
                    width=w, height=h,
                ))
        return detections

    def set_classes(self, classes: List[str]) -> None:
        """Not applicable for standard YOLOv8 (fixed COCO classes)."""
        pass


class YoloDetectorNode(Node):
    """ROS 2 node that wraps the YOLO detector."""

    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.40)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('allowed_classes', [
            'chair', 'bottle', 'couch', 'tv', 'potted plant',
            'person', 'dining table', 'laptop', 'book', 'cup'
        ])
        self.declare_parameter('max_rate_hz', 5.0)

        model_path = self.get_parameter('model_path').value
        conf = self.get_parameter('conf').value
        imgsz = self.get_parameter('imgsz').value
        device = self.get_parameter('device').value
        allowed = self.get_parameter('allowed_classes').value
        self._max_rate_hz = self.get_parameter('max_rate_hz').value
        self._min_interval = 1.0 / self._max_rate_hz
        self._last_inference = 0.0

        self.get_logger().info(f'Loading YOLO model: {model_path} (device={device})')
        self._detector = YOLOv8Detector(model_path, conf, imgsz, device, list(allowed))
        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # Timing stats
        self._inference_times: List[float] = []
        self._stats_timer = self.create_timer(10.0, self._log_stats)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._det_pub = self.create_publisher(
            Detection2DArray, '/aero/detections2d', 10)
        self._debug_pub = self.create_publisher(
            Image, '/aero/debug_image', 1)

        # Subscriber
        self._img_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, sensor_qos)

        self.get_logger().info('YoloDetectorNode started')

    def _image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_inference < self._min_interval:
            return
        self._last_inference = now

        with self._lock:
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'cv_bridge error: {e}')
                return

            t0 = time.monotonic()
            detections = self._detector.detect(cv_img)
            elapsed = time.monotonic() - t0
            self._inference_times.append(elapsed)

            # Build Detection2DArray
            arr = Detection2DArray()
            arr.header = msg.header
            for d in detections:
                det = Detection2D()
                det.header = msg.header
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = d.label
                hyp.hypothesis.score = d.confidence
                det.results.append(hyp)
                bb = BoundingBox2D()
                bb.center.position.x = d.x_center
                bb.center.position.y = d.y_center
                bb.size_x = d.width
                bb.size_y = d.height
                det.bbox = bb
                arr.detections.append(det)
            self._det_pub.publish(arr)

            # Annotated debug image
            debug_img = cv_img.copy()
            for d in detections:
                x1 = int(d.x_left)
                y1 = int(d.y_top)
                x2 = int(d.x_right)
                y2 = int(d.y_bottom)
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label_text = f'{d.label} {d.confidence:.2f}'
                cv2.putText(debug_img, label_text, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            debug_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _log_stats(self) -> None:
        if self._inference_times:
            avg_ms = 1000.0 * sum(self._inference_times) / len(self._inference_times)
            rate_hz = 1.0 / (sum(self._inference_times) / len(self._inference_times)) if self._inference_times else 0.0
            self.get_logger().info(
                f'YOLO stats: avg_inference={avg_ms:.1f} ms  rate={rate_hz:.1f} Hz'
            )
            self._inference_times.clear()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
