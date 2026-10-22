"""Offline parser for natural-language commands.

Used as a deterministic, zero-latency fallback when ANTHROPIC_API_KEY is not set
or when the network is unavailable. Pure logic - fully tested with pytest.
"""
import re
from typing import Any, Dict, Optional, Tuple


def parse_command(text: str) -> Tuple[Optional[str], Dict[str, Any]]:
    """Parse natural language command into (tool_name, tool_args).

    Supported commands:
      - go to / navigate to / drive to [the] <object>
      - go to <x>, <y>
      - find / search for / look for [the] <object>
      - explore / start exploring
      - stop / halt / emergency stop / cancel
      - what do you see / describe scene / look around
      - list objects / show objects / what objects do you know

    Returns:
        (tool_name, arguments_dict) or (None, {}) if unrecognised.
    """
    # Strip trailing punctuation but keep periods inside numbers and minus signs
    clean = text.strip().lower()
    clean = re.sub(r'[!?,]', '', clean)
    clean = re.sub(r'\.(?!\d)', '', clean)  # remove period only if not followed by a digit

    # Emergency stop / stop
    if re.search(r'\b(stop|halt|cancel|pause|freeze)\b', clean):
        return "stop", {}

    # Coordinate navigation: "go to 2.5, -1.0" or "go to x=1.5 y=2.0"
    coord_match = re.search(r'go\s+to\s+([-\d.]+)\s*[, ]\s*([-\d.]+)', clean)
    if coord_match:
        try:
            x = float(coord_match.group(1))
            y = float(coord_match.group(2))
            return "go_to_xy", {"x": x, "y": y}
        except ValueError:
            pass

    # Navigation to object: "go to the chair", "drive near bottle", "navigate to table"
    nav_match = re.search(
        r'\b(?:go to|navigate to|drive to|reach|head to)\s+(?:the\s+)?([a-z\s]+?)(?:\s+please)?$',
        clean
    )
    if nav_match:
        label = nav_match.group(1).strip()
        # Clean extra qualifiers
        label = re.sub(r'^(nearest|closest)\s+', '', label)
        return "go_to_object", {"label": label, "standoff_m": 0.8}

    # Search / find: "find the bottle", "search for chair", "where is the table"
    find_match = re.search(
        r'\b(?:find|search for|look for|locate|where is)\s+(?:the\s+)?([a-z\s]+?)(?:\s+please)?$',
        clean
    )
    if find_match:
        label = find_match.group(1).strip()
        return "find_object", {"label": label, "timeout_s": 300.0}

    # Explore: "explore", "start exploration", "wander around"
    if re.search(r'\b(explore|start exploring|map the house|explore house|autonomous exploration)\b', clean):
        return "explore", {}

    # Scene description: "what do you see", "describe scene", "look around"
    if re.search(r'\b(what do you see|describe|look around|describe the scene|camera view)\b', clean):
        return "describe_scene", {}

    # Object listing: "list objects", "what objects", "show objects", "detected objects"
    if re.search(r'\b(list objects|show objects|what objects|known objects|get objects)\b', clean):
        return "list_objects", {}

    # Status: "status", "where are you", "robot status"
    if re.search(r'\b(status|where are you|current status|get status)\b', clean):
        return "get_status", {}

    return None, {}
