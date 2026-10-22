#!/usr/bin/env python3
"""FastAPI dashboard server for AERO running concurrently with an rclpy node.

Endpoints:
  GET  /              Static single-page dashboard
  WS   /ws            WebSocket stream (pose, mode, map, objects, lidar)
  POST /command       Send natural-language command to robot
  POST /estop         Activate or clear emergency stop
  GET  /objects       List confirmed semantic objects
  GET  /stream        MJPEG video stream from /aero/debug_image
"""
import asyncio
import base64
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped
import tf2_ros
from cv_bridge import CvBridge

try:
    from aero_interfaces.msg import SemanticObjectArray
    HAS_INTERFACES = True
except ImportError:
    HAS_INTERFACES = False


class DashboardBridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')

        self._lock = threading.Lock()
        self.robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.mode = "IDLE"
        self.estop = False
        self.latest_goal = None
        self.latest_map_data: Optional[Dict[str, Any]] = None
        self.semantic_objects: List[Dict[str, Any]] = []
        self.latest_scan_ranges: List[float] = []
        self.latest_jpeg: Optional[bytes] = None
        self.chat_history: List[Dict[str, str]] = []

        # TF for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.create_subscription(String, '/aero/mode', self._mode_callback, 10)
        self.create_subscription(Bool, '/aero/estop', self._estop_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, sensor_qos)
        self.create_subscription(PoseStamped, '/aero/current_goal', self._goal_callback, 10)
        self.create_subscription(String, '/aero/commander_reply', self._reply_callback, 10)
        self.create_subscription(Image, '/aero/debug_image', self._image_callback, sensor_qos)

        if HAS_INTERFACES:
            self.create_subscription(SemanticObjectArray, '/aero/semantic_objects', self._objects_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/aero/user_command', 10)
        self.estop_pub = self.create_publisher(Bool, '/aero/estop', 10)
        self.mode_pub = self.create_publisher(String, '/aero/mode', 10)

        # Pose update timer (10 Hz)
        self.create_timer(0.1, self._update_pose)

        self.bridge = CvBridge()
        self.get_logger().info('Dashboard bridge node initialized')

    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            # Compute yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            with self._lock:
                self.robot_pose = {
                    "x": float(t.x),
                    "y": float(t.y),
                    "yaw": float(yaw)
                }
        except Exception:
            pass

    def _map_callback(self, msg: OccupancyGrid):
        with self._lock:
            self.latest_map_data = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "data": list(msg.data)
            }

    def _mode_callback(self, msg: String):
        with self._lock:
            self.mode = msg.data

    def _estop_callback(self, msg: Bool):
        with self._lock:
            self.estop = msg.data

    def _scan_callback(self, msg: LaserScan):
        # Downsample scan to 36 beams (every 10 degrees)
        step = max(1, len(msg.ranges) // 36)
        sub = [round(float(r), 2) if np.isfinite(r) else 0.0 for r in msg.ranges[::step]]
        with self._lock:
            self.latest_scan_ranges = sub

    def _goal_callback(self, msg: PoseStamped):
        with self._lock:
            self.latest_goal = {
                "x": round(msg.pose.position.x, 2),
                "y": round(msg.pose.position.y, 2)
            }

    def _reply_callback(self, msg: String):
        with self._lock:
            self.chat_history.append({"sender": "robot", "text": msg.data, "time": time.strftime("%H:%M:%S")})
            if len(self.chat_history) > 50:
                self.chat_history.pop(0)

    def _objects_callback(self, msg):
        objs = []
        for o in msg.objects:
            objs.append({
                "id": o.id,
                "label": o.label,
                "confidence": round(float(o.confidence), 2),
                "x": round(float(o.pose.pose.position.x), 2),
                "y": round(float(o.pose.pose.position.y), 2),
                "observations": o.observations
            })
        with self._lock:
            self.semantic_objects = objs

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize for faster web transmission
            small = cv2.resize(cv_img, (320, 240))
            _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._lock:
                self.latest_jpeg = buf.tobytes()
        except Exception:
            pass

    def send_user_command(self, text: str):
        with self._lock:
            self.chat_history.append({"sender": "user", "text": text, "time": time.strftime("%H:%M:%S")})
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def trigger_estop(self, active: bool = True):
        with self._lock:
            self.estop = active
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

    def trigger_mode(self, mode: str):
        with self._lock:
            self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)


# FastAPI Web App
app = FastAPI(title="AERO Web Dashboard")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

bridge_node: Optional[DashboardBridgeNode] = None


class CommandRequest(BaseModel):
    text: str


class EstopRequest(BaseModel):
    active: bool = True


class ModeRequest(BaseModel):
    mode: str


@app.get("/", response_class=HTMLResponse)
async def get_index():
    # Load index.html from static
    static_file = Path(__file__).parent.parent / "static" / "index.html"
    if not static_file.exists():
        # Fallback to current dir static
        static_file = Path("/opt/ros/jazzy")  # dummy
    if not os.path.exists(static_file):
        share_dir = Path(os.environ.get("AMENT_PREFIX_PATH", "").split(":")[0]) / "share" / "aero_dashboard" / "static" / "index.html"
        if share_dir.exists():
            static_file = share_dir
        else:
            static_file = Path(__file__).resolve().parent.parent / "static" / "index.html"

    with open(static_file, "r", encoding="utf-8") as f:
        return f.read()


@app.post("/command")
async def post_command(req: CommandRequest):
    if bridge_node:
        bridge_node.send_user_command(req.text)
        return {"status": "sent", "command": req.text}
    return {"status": "error", "message": "ROS node offline"}


@app.post("/estop")
async def post_estop(req: EstopRequest):
    if bridge_node:
        bridge_node.trigger_estop(req.active)
        return {"status": "ok", "estop": req.active}
    return {"status": "error"}


@app.post("/mode")
async def post_mode(req: ModeRequest):
    if bridge_node:
        bridge_node.trigger_mode(req.mode)
        return {"status": "ok", "mode": req.mode}
    return {"status": "error"}


@app.get("/objects")
async def get_objects():
    if bridge_node:
        with bridge_node._lock:
            return {"objects": bridge_node.semantic_objects}
    return {"objects": []}


def gen_frames():
    while True:
        if bridge_node:
            with bridge_node._lock:
                frame = bridge_node.latest_jpeg
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)


@app.get("/stream")
async def video_stream():
    return StreamingResponse(
        gen_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    last_map_sent = 0.0
    try:
        while True:
            if bridge_node:
                now = time.time()
                with bridge_node._lock:
                    packet = {
                        "pose": bridge_node.robot_pose,
                        "mode": bridge_node.mode,
                        "estop": bridge_node.estop,
                        "goal": bridge_node.latest_goal,
                        "objects": bridge_node.semantic_objects,
                        "scan": bridge_node.latest_scan_ranges,
                        "chat": bridge_node.chat_history[-5:] if bridge_node.chat_history else []
                    }
                    # Send map every 1.5 seconds
                    if now - last_map_sent > 1.5 and bridge_node.latest_map_data:
                        packet["map"] = bridge_node.latest_map_data
                        last_map_sent = now

                await websocket.send_text(json.dumps(packet))
            await asyncio.sleep(0.2)  # 5 Hz
    except WebSocketDisconnect:
        pass


def run_ros(node):
    rclpy.spin(node)


def main(args=None):
    global bridge_node
    rclpy.init(args=args)
    bridge_node = DashboardBridgeNode()

    # Run ROS spin in background daemon thread
    ros_thread = threading.Thread(target=run_ros, args=(bridge_node,), daemon=True)
    ros_thread.start()

    port = 8000
    try:
        port_param = bridge_node.declare_parameter('port', 8000)
        port = int(bridge_node.get_parameter('port').value)
    except Exception:
        pass

    bridge_node.get_logger().info(f'Starting Dashboard on http://127.0.0.1:{port}')
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
