#!/usr/bin/env python3
"""AERO Dashboard Server.

Supports two operational modes:
1. Live ROS 2 mode (when running inside Ubuntu 24.04 with ROS 2 Jazzy).
2. Standalone Interactive Simulator (when running directly on Windows/host without ROS).

Endpoints:
  GET  /              Static single-page dashboard
  WS   /ws            WebSocket stream (pose, mode, map, objects, lidar, chat)
  POST /command       Send natural-language command to robot
  POST /estop         Activate or clear emergency stop
  POST /mode          Set exploration/stop/idle mode
  GET  /objects       List confirmed semantic objects
  GET  /stream        MJPEG video stream from debug image
"""
import asyncio
import json
import math
import os
import random
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel

# Check if rclpy is available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import String, Bool
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan, Image
    from geometry_msgs.msg import PoseStamped
    import tf2_ros
    from cv_bridge import CvBridge
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


# Base Bridge interface
class BaseBridge:
    def __init__(self):
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

    def send_user_command(self, text: str):
        pass

    def trigger_estop(self, active: bool = True):
        pass

    def trigger_mode(self, mode: str):
        pass

    def teleop(self, linear: float, angular: float):
        pass


if HAS_ROS2:
    class ROS2DashboardBridge(Node, BaseBridge):
        def __init__(self):
            Node.__init__(self, 'dashboard_bridge')
            BaseBridge.__init__(self)

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.bridge = CvBridge()

            map_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1)
            sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)

            self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
            self.create_subscription(String, '/aero/mode', self._mode_cb, 10)
            self.create_subscription(Bool, '/aero/estop', self._estop_cb, 10)
            self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
            self.create_subscription(PoseStamped, '/aero/current_goal', self._goal_cb, 10)
            self.create_subscription(String, '/aero/commander_reply', self._reply_cb, 10)
            self.create_subscription(Image, '/aero/debug_image', self._image_cb, sensor_qos)

            self.cmd_pub = self.create_publisher(String, '/aero/user_command', 10)
            self.estop_pub = self.create_publisher(Bool, '/aero/estop', 10)
            self.mode_pub = self.create_publisher(String, '/aero/mode', 10)

            self.create_timer(0.1, self._update_pose)
            self.get_logger().info('ROS 2 Dashboard Bridge connected')

        def _update_pose(self):
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = np.arctan2(siny_cosp, cosy_cosp)
                with self._lock:
                    self.robot_pose = {"x": float(t.x), "y": float(t.y), "yaw": float(yaw)}
            except Exception:
                pass

        def _map_cb(self, msg):
            with self._lock:
                self.latest_map_data = {
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "resolution": msg.info.resolution,
                    "origin_x": msg.info.origin.position.x,
                    "origin_y": msg.info.origin.position.y,
                    "data": list(msg.data)
                }

        def _mode_cb(self, msg):
            with self._lock:
                self.mode = msg.data

        def _estop_cb(self, msg):
            with self._lock:
                self.estop = msg.data

        def _scan_cb(self, msg):
            step = max(1, len(msg.ranges) // 36)
            sub = [round(float(r), 2) if np.isfinite(r) else 0.0 for r in msg.ranges[::step]]
            with self._lock:
                self.latest_scan_ranges = sub

        def _goal_cb(self, msg):
            with self._lock:
                self.latest_goal = {"x": round(msg.pose.position.x, 2), "y": round(msg.pose.position.y, 2)}

        def _reply_cb(self, msg):
            with self._lock:
                self.chat_history.append({"sender": "robot", "text": msg.data, "time": time.strftime("%H:%M:%S")})

        def _image_cb(self, msg):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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


# Standalone Simulator for Windows demo when ROS 2 is not yet running
class StandaloneSimulator(BaseBridge):
    def __init__(self):
        super().__init__()
        self.mode = "IDLE"
        self.target_x = 0.0
        self.target_y = 0.0

        # Create synthetic house occupancy grid (100x100 cells, 0.1m resolution)
        self.grid_w = 100
        self.grid_h = 100
        self.res = 0.1
        self.ox = -5.0
        self.oy = -5.0

        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)
        # Outer walls
        grid[0:2, :] = 100
        grid[-2:, :] = 100
        grid[:, 0:2] = 100
        grid[:, -2:] = 100
        # Interior room partitions
        grid[50:52, 10:70] = 100
        grid[20:80, 50:52] = 100
        # Door openings
        grid[50:52, 35:45] = 0
        grid[45:55, 50:52] = 0

        self.latest_map_data = {
            "width": self.grid_w,
            "height": self.grid_h,
            "resolution": self.res,
            "origin_x": self.ox,
            "origin_y": self.oy,
            "data": grid.flatten().tolist()
        }

        # Confirmed ground truth objects in the world
        self.semantic_objects = [
            {"id": 1, "label": "chair", "x": 2.0, "y": 1.5, "confidence": 0.88, "observations": 5},
            {"id": 2, "label": "bottle", "x": 0.5, "y": 3.0, "confidence": 0.94, "observations": 8},
            {"id": 3, "label": "couch", "x": -1.5, "y": 2.0, "confidence": 0.82, "observations": 4},
            {"id": 4, "label": "potted plant", "x": 3.0, "y": -1.0, "confidence": 0.91, "observations": 6},
            {"id": 5, "label": "dining table", "x": 1.0, "y": -2.5, "confidence": 0.85, "observations": 7}
        ]

        self.chat_history.append({
            "sender": "robot",
            "text": "AERO Standalone Simulator online! Type commands like 'explore', 'go to the chair', 'find the bottle', or 'stop'.",
            "time": time.strftime("%H:%M:%S")
        })

        # Start simulation loop in background thread
        threading.Thread(target=self._sim_loop, daemon=True).start()

    def _sim_loop(self):
        step = 0
        while True:
            time.sleep(0.1)  # 10 Hz
            step += 1

            with self._lock:
                if self.estop:
                    continue

                rx = self.robot_pose["x"]
                ry = self.robot_pose["y"]
                yaw = self.robot_pose["yaw"]

                if self.mode == "EXPLORE":
                    # Autonomous exploration motion (patrol around house)
                    t = time.time() * 0.3
                    tx = 2.2 * math.sin(t)
                    ty = 2.2 * math.cos(t * 0.7)
                    self.target_x = tx
                    self.target_y = ty
                    self.latest_goal = {"x": round(tx, 2), "y": round(ty, 2)}

                # Move towards target if goal exists
                if self.latest_goal:
                    tx = self.latest_goal["x"]
                    ty = self.latest_goal["y"]
                    dx = tx - rx
                    dy = ty - ry
                    dist = math.sqrt(dx * dx + dy * dy)

                    if dist > 0.15:
                        target_yaw = math.atan2(dy, dx)
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
                        yaw += np.clip(yaw_err, -0.1, 0.1)
                        rx += math.cos(yaw) * min(0.04, dist)
                        ry += math.sin(yaw) * min(0.04, dist)
                    else:
                        if self.mode != "EXPLORE":
                            self.latest_goal = None

                self.robot_pose = {"x": round(rx, 3), "y": round(ry, 3), "yaw": round(yaw, 3)}

                # Update synthetic camera frame
                if step % 2 == 0:
                    frame = np.full((240, 320, 3), 30, dtype=np.uint8)
                    # Draw camera perspective elements
                    cv2.putText(frame, "AERO CAMERA (YOLOv8 Throttled 5Hz)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, f"Pose: ({rx:.2f}, {ry:.2f})  Mode: {self.mode}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                    # Draw detected objects if in view
                    for obj in self.semantic_objects:
                        dx = obj["x"] - rx
                        dy = obj["y"] - ry
                        dist = math.sqrt(dx * dx + dy * dy)
                        if dist < 3.5:
                            bx = int(160 + (dx * 30))
                            by = int(120 - (dy * 20))
                            bx = max(20, min(260, bx))
                            by = max(40, min(180, by))
                            cv2.rectangle(frame, (bx - 30, by - 25), (bx + 30, by + 25), (0, 255, 0), 2)
                            cv2.putText(frame, f"{obj['label']} {obj['confidence']:.2f}", (bx - 30, by - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    self.latest_jpeg = buf.tobytes()

    def send_user_command(self, text: str):
        clean = text.strip().lower()
        t_now = time.strftime("%H:%M:%S")

        with self._lock:
            self.chat_history.append({"sender": "user", "text": text, "time": t_now})

        # Process command using pure parser logic
        reply = ""
        if "stop" in clean or "halt" in clean:
            self.mode = "STOP"
            self.latest_goal = None
            reply = "Robot stopped immediately."
        elif "explore" in clean:
            self.mode = "EXPLORE"
            reply = "Started autonomous exploration to map the house."
        elif "chair" in clean:
            self.latest_goal = {"x": 1.5, "y": 1.5}
            self.mode = "NAVIGATE"
            reply = "Navigating to the chair (standoff 0.8m)."
        elif "bottle" in clean:
            self.latest_goal = {"x": 0.5, "y": 2.3}
            self.mode = "NAVIGATE"
            reply = "Navigating to the bottle."
        elif "table" in clean:
            self.latest_goal = {"x": 1.0, "y": -1.8}
            self.mode = "NAVIGATE"
            reply = "Navigating to the dining table."
        elif "list" in clean:
            objs = [f"{o['label']} at ({o['x']}, {o['y']})" for o in self.semantic_objects]
            reply = f"Known confirmed objects: {', '.join(objs)}."
        elif "unicorn" in clean:
            reply = "Object 'unicorn' not found in semantic map. Would you like me to explore to search for it?"
        else:
            reply = f"Command received: '{text}'. Executing navigation goal."

        with self._lock:
            self.chat_history.append({"sender": "robot", "text": reply, "time": time.strftime("%H:%M:%S")})

    def trigger_estop(self, active: bool = True):
        with self._lock:
            self.estop = active
            if active:
                self.mode = "STOP"
                self.latest_goal = None
                self.chat_history.append({
                    "sender": "robot",
                    "text": "EMERGENCY STOP TRIGGERED! All motors halted.",
                    "time": time.strftime("%H:%M:%S")
                })
            else:
                self.chat_history.append({
                    "sender": "robot",
                    "text": "Emergency stop cleared.",
                    "time": time.strftime("%H:%M:%S")
                })

    def trigger_mode(self, mode: str):
        with self._lock:
            self.mode = mode.upper()
            if self.mode == "STOP":
                self.latest_goal = None

    def teleop(self, linear: float, angular: float):
        with self._lock:
            if self.estop:
                return
            self.mode = "MANUAL"
            self.latest_goal = None
            yaw = self.robot_pose["yaw"] + angular * 0.25
            rx = self.robot_pose["x"] + math.cos(yaw) * linear * 0.35
            ry = self.robot_pose["y"] + math.sin(yaw) * linear * 0.35
            # Constrain to house borders
            rx = max(-4.2, min(4.2, rx))
            ry = max(-4.2, min(4.2, ry))
            self.robot_pose = {"x": round(rx, 3), "y": round(ry, 3), "yaw": round(yaw, 3)}


# FastAPI Application
app = FastAPI(title="AERO Web Dashboard")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

bridge: Optional[BaseBridge] = None


class CommandRequest(BaseModel):
    text: str


class EstopRequest(BaseModel):
    active: bool = True


class ModeRequest(BaseModel):
    mode: str


@app.get("/", response_class=HTMLResponse)
async def get_index():
    static_file = Path(__file__).resolve().parent.parent / "static" / "index.html"
    if not static_file.exists():
        static_file = Path("src/aero_dashboard/static/index.html")
    with open(static_file, "r", encoding="utf-8") as f:
        return f.read()


@app.post("/command")
async def post_command(req: CommandRequest):
    if bridge:
        bridge.send_user_command(req.text)
        return {"status": "sent", "command": req.text}
    return {"status": "error"}


@app.post("/estop")
async def post_estop(req: EstopRequest):
    if bridge:
        bridge.trigger_estop(req.active)
        return {"status": "ok", "estop": req.active}
    return {"status": "error"}


@app.post("/mode")
async def post_mode(req: ModeRequest):
    if bridge:
        bridge.trigger_mode(req.mode)
        return {"status": "ok", "mode": req.mode}
    return {"status": "error"}


@app.get("/objects")
async def get_objects():
    if bridge:
        with bridge._lock:
            return {"objects": bridge.semantic_objects}
    return {"objects": []}


class TeleopRequest(BaseModel):
    linear: float = 0.0
    angular: float = 0.0


@app.post("/teleop")
async def post_teleop(req: TeleopRequest):
    if bridge:
        bridge.teleop(req.linear, req.angular)
        return {"status": "ok", "linear": req.linear, "angular": req.angular}
    return {"status": "error"}


def gen_frames():
    while True:
        if bridge:
            with bridge._lock:
                frame = bridge.latest_jpeg
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
            if bridge:
                now = time.time()
                with bridge._lock:
                    packet = {
                        "pose": bridge.robot_pose,
                        "mode": bridge.mode,
                        "estop": bridge.estop,
                        "goal": bridge.latest_goal,
                        "objects": bridge.semantic_objects,
                        "scan": bridge.latest_scan_ranges,
                        "chat": bridge.chat_history[-8:] if bridge.chat_history else []
                    }
                    if now - last_map_sent > 1.0 and bridge.latest_map_data:
                        packet["map"] = bridge.latest_map_data
                        last_map_sent = now

                await websocket.send_text(json.dumps(packet))
            await asyncio.sleep(0.15)  # ~6 Hz
    except WebSocketDisconnect:
        pass


def main(args=None):
    global bridge
    if HAS_ROS2:
        try:
            rclpy.init(args=args)
            bridge = ROS2DashboardBridge()
            threading.Thread(target=lambda: rclpy.spin(bridge), daemon=True).start()
        except Exception:
            bridge = StandaloneSimulator()
    else:
        print("[AERO] Operating in Interactive Standalone Simulator mode.")
        bridge = StandaloneSimulator()

    port = 8000
    print(f"\n=======================================================")
    print(f" AERO Mission Control Dashboard Started")
    print(f" Open in Browser: http://127.0.0.1:{port}")
    print(f"=======================================================\n")

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


if __name__ == '__main__':
    main()
