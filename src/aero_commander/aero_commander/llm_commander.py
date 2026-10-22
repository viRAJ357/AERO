#!/usr/bin/env python3
"""LLM Commander ROS 2 node for AERO.

Translates natural-language user commands into validated tool calls.
Supports Claude tool use (up to 5 turns) with automatic fallback to
offline regex parser when ANTHROPIC_API_KEY is not set.

The LLM NEVER commands velocity directly — only invokes validated tools.
"""
import base64
import json
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import tf2_ros

from aero_commander.prompts import COMMANDER_SYSTEM_PROMPT, TOOLS_SCHEMA
from aero_commander.tools import ToolExecutor
from aero_commander.offline_parser import parse_command

try:
    from aero_interfaces.action import SafeGoTo
    from aero_interfaces.srv import QueryObjects, SetMode
    from aero_interfaces.msg import SemanticObject
    HAS_INTERFACES = True
except ImportError:
    HAS_INTERFACES = False


class LLMCommanderNode(Node):
    MAX_TOOL_TURNS = 5
    API_TIMEOUT_S = 20.0

    def __init__(self):
        super().__init__('llm_commander')

        # Parameters
        self.declare_parameter('model', 'claude-3-5-sonnet-20241022')
        self.declare_parameter('use_llm', True)
        self.declare_parameter('log_dir', os.path.expanduser('~/.aero/logs'))

        self.model_name = self.get_parameter('model').value
        self.use_llm_param = self.get_parameter('use_llm').value
        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(log_dir, 'command_log.jsonl')

        self._lock = threading.Lock()
        self.current_mode = "IDLE"
        self.is_estop_active = False
        self._latest_image: Optional[Image] = None

        # TF for robot pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Tool executor
        self.tools = ToolExecutor(self)

        # ROS Communications
        cb_group = ReentrantCallbackGroup()
        self.create_subscription(String, '/aero/user_command', self._command_callback, 10)
        self.create_subscription(String, '/aero/mode', self._mode_callback, 10)
        self.create_subscription(Bool, '/aero/estop', self._estop_callback, 10)
        self.create_subscription(Image, '/aero/debug_image', self._image_callback, 1)

        self.log_pub = self.create_publisher(String, '/aero/command_log', 10)
        self.reply_pub = self.create_publisher(String, '/aero/commander_reply', 10)
        self.mode_pub = self.create_publisher(String, '/aero/mode', 10)

        # Clients for tools
        if HAS_INTERFACES:
            self.safe_goto_client = ActionClient(self, SafeGoTo, '/aero/safe_goto', callback_group=cb_group)
            self.query_objects_client = self.create_client(QueryObjects, '/aero/query_objects', callback_group=cb_group)
            self.set_mode_client = self.create_client(SetMode, '/aero/set_mode', callback_group=cb_group)

        # Initialize Anthropic client if available
        self.api_key = os.environ.get('ANTHROPIC_API_KEY', '').strip()
        self.claude_client = None
        if self.use_llm_param and self.api_key:
            try:
                import anthropic
                self.claude_client = anthropic.Anthropic(api_key=self.api_key)
                self.get_logger().info('Anthropic Claude client initialized successfully')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize Claude client: {e}. Will use offline parser.')
        else:
            self.get_logger().info('ANTHROPIC_API_KEY not set or use_llm=false. Operating in OFFLINE parser mode.')

        self.get_logger().info('LLMCommanderNode ready')

    def _mode_callback(self, msg: String):
        with self._lock:
            self.current_mode = msg.data

    def _estop_callback(self, msg: Bool):
        with self._lock:
            self.is_estop_active = msg.data

    def _image_callback(self, msg: Image):
        with self._lock:
            self._latest_image = msg

    def get_robot_pose(self) -> Tuple[float, float]:
        """Lookup current robot pose in map frame."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return 0.0, 0.0

    def set_mode(self, mode: str):
        with self._lock:
            self.current_mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def cancel_all_navigation(self):
        """Cancel goals via SetMode STOP."""
        self.set_mode("STOP")

    def call_safe_goto(self, x: float, y: float, reason: str) -> Tuple[bool, str]:
        """Send a goal to the SafeGoTo action server synchronously."""
        if not HAS_INTERFACES or not hasattr(self, 'safe_goto_client'):
            return False, "SafeGoTo action not available"

        if not self.safe_goto_client.wait_for_server(timeout_sec=3.0):
            return False, "Safety gate action server unreachable"

        goal = SafeGoTo.Goal()
        goal.target.header.frame_id = 'map'
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.pose.position.x = float(x)
        goal.target.pose.position.y = float(y)
        goal.target.pose.orientation.w = 1.0
        goal.reason = reason

        future = self.safe_goto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            return False, "Goal was rejected by Safety Gate"

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=120.0)
        result = res_future.result()

        if result and result.result.success:
            return True, result.result.message
        else:
            msg = result.result.message if result else "Navigation timed out"
            return False, msg

    def create_query_request(self, label: str):
        req = QueryObjects.Request()
        req.label = label
        req.min_confidence = 0.3
        return req

    def wait_for_future(self, future, timeout_sec: float = 3.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

    def describe_scene_vision(self) -> Dict[str, Any]:
        """Send camera frame to Claude vision."""
        if not self.claude_client:
            return {"status": "unavailable", "message": "Claude vision requires ANTHROPIC_API_KEY"}
        with self._lock:
            img = self._latest_image
        if not img or not img.data:
            return {"status": "error", "message": "No camera image available"}

        try:
            # Simple conversion if cv_bridge available
            from cv_bridge import CvBridge
            import cv2
            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_img)
            b64_image = base64.b64encode(buffer).decode('utf-8')

            resp = self.claude_client.messages.create(
                model=self.model_name,
                max_tokens=200,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Describe the scene visible from the robot camera in 2 short sentences."},
                        {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": b64_image}}
                    ]
                }],
                timeout=15.0
            )
            desc = resp.content[0].text
            return {"status": "success", "description": desc}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def execute_tool(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatch tool name to implementation."""
        if name == "list_objects":
            return self.tools.list_objects(args.get("label"))
        elif name == "go_to_object":
            return self.tools.go_to_object(args.get("label", ""), args.get("standoff_m", 0.8))
        elif name == "go_to_xy":
            return self.tools.go_to_xy(float(args.get("x", 0)), float(args.get("y", 0)))
        elif name == "find_object":
            return self.tools.find_object(args.get("label", ""), args.get("timeout_s", 300.0))
        elif name == "explore":
            return self.tools.explore(args.get("duration_s"))
        elif name == "stop":
            return self.tools.stop()
        elif name == "get_status":
            return self.tools.get_status()
        elif name == "describe_scene":
            return self.tools.describe_scene()
        else:
            return {"status": "error", "message": f"Unknown tool: {name}"}

    def _command_callback(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f'Received command: "{user_text}"')
        t0 = time.time()

        if self.claude_client:
            reply = self._process_with_claude(user_text)
        else:
            reply = self._process_offline(user_text)

        # Publish reply
        reply_msg = String()
        reply_msg.data = reply
        self.reply_pub.publish(reply_msg)
        self.get_logger().info(f'Commander reply: {reply}')

        # Log
        log_entry = {
            "timestamp": time.time(),
            "command": user_text,
            "reply": reply,
            "duration_s": round(time.time() - t0, 3)
        }
        self._log_event(log_entry)

    def _process_offline(self, text: str) -> str:
        """Handle command with regex offline parser."""
        tool_name, tool_args = parse_command(text)
        if not tool_name:
            return "Command not recognized in offline mode. Try 'go to <object>', 'explore', or 'stop'."

        self.get_logger().info(f'Offline parsed -> tool: {tool_name}, args: {tool_args}')
        result = self.execute_tool(tool_name, tool_args)

        if tool_name == "stop":
            return "Robot stopped immediately."
        elif tool_name == "explore":
            return "Started autonomous exploration to map the environment."
        elif tool_name == "go_to_object":
            if result.get("status") == "success":
                return f"Navigating to the {tool_args.get('label')}."
            elif result.get("status") == "not_found":
                return f"I haven't seen a {tool_args.get('label')} yet. You can ask me to explore to find it."
            else:
                return f"Cannot reach {tool_args.get('label')}: {result.get('message')}"
        elif tool_name == "find_object":
            return f"Exploring to find the {tool_args.get('label')}."
        elif tool_name == "go_to_xy":
            if result.get("status") == "success":
                return f"Navigating to coordinates ({tool_args.get('x')}, {tool_args.get('y')})."
            else:
                return f"Goal rejected: {result.get('message')}"
        elif tool_name == "list_objects":
            objs = result.get("objects", [])
            if not objs:
                return "No objects confirmed in the semantic map yet."
            names = [f"{o['label']} at ({o['x']}, {o['y']})" for o in objs]
            return f"Known objects: {', '.join(names)}."
        elif tool_name == "describe_scene":
            return result.get("description", result.get("message", "No scene description available."))
        return f"Completed {tool_name}: {result.get('message', 'ok')}"

    def _process_with_claude(self, text: str) -> str:
        """Multi-turn Claude tool-use loop."""
        rx, ry = self.get_robot_pose()
        robot_state = {
            "pose": {"x": round(rx, 2), "y": round(ry, 2)},
            "mode": self.current_mode,
            "estop": self.is_estop_active
        }
        sys_prompt = COMMANDER_SYSTEM_PROMPT + f"\n\nCurrent Robot State:\n{json.dumps(robot_state, indent=2)}"

        messages = [{"role": "user", "content": text}]

        for turn in range(self.MAX_TOOL_TURNS):
            try:
                resp = self.claude_client.messages.create(
                    model=self.model_name,
                    max_tokens=500,
                    system=sys_prompt,
                    tools=TOOLS_SCHEMA,
                    messages=messages,
                    timeout=self.API_TIMEOUT_S
                )
            except Exception as e:
                self.get_logger().error(f'Claude API error: {e}. Falling back to offline parser.')
                return self._process_offline(text)

            messages.append({"role": "assistant", "content": resp.content})

            calls = [b for b in resp.content if b.type == "tool_use"]
            if not calls:
                return "".join(b.text for b in resp.content if b.type == "text")

            results = []
            for c in calls:
                self.get_logger().info(f'LLM invoked tool: {c.name}({c.input})')
                out = self.execute_tool(c.name, c.input)
                results.append({
                    "type": "tool_result",
                    "tool_use_id": c.id,
                    "content": json.dumps(out)
                })

            messages.append({"role": "user", "content": results})

        return "Request exceeded maximum planning turns."

    def _log_event(self, entry: Dict[str, Any]):
        log_str = json.dumps(entry)
        msg = String()
        msg.data = log_str
        self.log_pub.publish(msg)
        try:
            with open(self.log_file, 'a', encoding='utf-8') as f:
                f.write(log_str + '\n')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
