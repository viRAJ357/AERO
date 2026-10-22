"""Tool implementations for AERO Commander.

Each tool executes validated actions via ROS 2 services/actions and returns
a JSON-serialisable dictionary. Exceptions are captured and returned as error text
so the LLM can recover gracefully.
"""
import json
import math
import time
from typing import Any, Dict, Optional


class ToolExecutor:
    def __init__(self, node):
        self._node = node

    def list_objects(self, label: Optional[str] = None) -> Dict[str, Any]:
        """Query semantic map for known objects."""
        try:
            if not hasattr(self._node, 'query_objects_client'):
                return {"status": "error", "message": "QueryObjects service not ready"}

            req = self._node.create_query_request(label or "")
            future = self._node.query_objects_client.call_async(req)
            self._node.wait_for_future(future, timeout_sec=3.0)
            res = future.result()
            if res and res.success:
                objs = [
                    {
                        "id": o.id,
                        "label": o.label,
                        "x": round(o.pose.pose.position.x, 2),
                        "y": round(o.pose.pose.position.y, 2),
                        "confidence": round(o.confidence, 2),
                        "observations": o.observations
                    }
                    for o in res.objects
                ]
                return {"status": "success", "count": len(objs), "objects": objs}
            return {"status": "success", "count": 0, "objects": []}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def go_to_object(self, label: str, standoff_m: float = 0.8) -> Dict[str, Any]:
        """Navigate near a known object, stopping standoff_m before it."""
        try:
            objs_res = self.list_objects(label)
            if objs_res.get("status") != "success" or not objs_res.get("objects"):
                return {
                    "status": "not_found",
                    "message": f"Object '{label}' is not known yet. You can call find_object to search for it."
                }

            # Select highest confidence object
            target = max(objs_res["objects"], key=lambda o: o["confidence"])
            ox, oy = target["x"], target["y"]

            # Compute standoff goal relative to current robot pose
            rx, ry = self._node.get_robot_pose()
            dx = ox - rx
            dy = oy - ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.05:
                ratio = max(0.0, (dist - standoff_m) / dist)
                gx = rx + dx * ratio
                gy = ry + dy * ratio
            else:
                gx, gy = rx, ry

            # Call SafeGoTo
            return self.go_to_xy(gx, gy, reason=f"Approach {label}")
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def go_to_xy(self, x: float, y: float, reason: str = "Commanded goal") -> Dict[str, Any]:
        """Send target coordinates through the safety gate."""
        try:
            success, msg = self._node.call_safe_goto(x, y, reason)
            return {
                "status": "success" if success else "rejected",
                "message": msg,
                "target": {"x": round(x, 2), "y": round(y, 2)}
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def find_object(self, label: str, timeout_s: float = 300.0) -> Dict[str, Any]:
        """Search for an object by activating exploration."""
        try:
            # Check if already confirmed
            objs_res = self.list_objects(label)
            if objs_res.get("status") == "success" and objs_res.get("objects"):
                return {
                    "status": "found",
                    "message": f"Object '{label}' is already known.",
                    "object": objs_res["objects"][0]
                }

            # Start exploration
            self._node.set_mode("EXPLORE")
            return {
                "status": "searching",
                "message": f"Started exploration to find '{label}'. Mode set to EXPLORE."
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def explore(self, duration_s: Optional[float] = None) -> Dict[str, Any]:
        """Activate autonomous exploration."""
        try:
            self._node.set_mode("EXPLORE")
            return {"status": "success", "mode": "EXPLORE", "message": "Autonomous exploration activated."}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def stop(self) -> Dict[str, Any]:
        """Halt robot and cancel active tasks."""
        try:
            self._node.set_mode("STOP")
            self._node.cancel_all_navigation()
            return {"status": "success", "mode": "STOP", "message": "Robot stopped."}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def get_status(self) -> Dict[str, Any]:
        """Return robot status dictionary."""
        rx, ry = self._node.get_robot_pose()
        return {
            "status": "success",
            "pose": {"x": round(rx, 2), "y": round(ry, 2)},
            "mode": self._node.current_mode,
            "estop": self._node.is_estop_active
        }

    def describe_scene(self) -> Dict[str, Any]:
        """Capture scene description from vision API."""
        return self._node.describe_scene_vision()
