"""Unit tests for aero_commander.offline_parser.

Validates the mapping of text commands to structured tool calls.
"""
import sys
import os
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'aero_commander')))
from aero_commander.offline_parser import parse_command


@pytest.mark.parametrize("text, expected_tool, expected_args", [
    ("stop", "stop", {}),
    ("emergency stop!", "stop", {}),
    ("halt immediately", "stop", {}),
    ("explore", "explore", {}),
    ("start exploring the house", "explore", {}),
    ("go to the chair", "go_to_object", {"label": "chair", "standoff_m": 0.8}),
    ("navigate to bottle please", "go_to_object", {"label": "bottle", "standoff_m": 0.8}),
    ("go to 1.5, -2.0", "go_to_xy", {"x": 1.5, "y": -2.0}),
    ("find the bottle", "find_object", {"label": "bottle", "timeout_s": 300.0}),
    ("where is the table", "find_object", {"label": "table", "timeout_s": 300.0}),
    ("what do you see", "describe_scene", {}),
    ("list objects", "list_objects", {}),
    ("status", "get_status", {}),
])
def test_parse_command(text, expected_tool, expected_args):
    tool, args = parse_command(text)
    assert tool == expected_tool
    assert args == expected_args


def test_unknown_command():
    tool, args = parse_command("make me a sandwich")
    assert tool is None
    assert args == {}
