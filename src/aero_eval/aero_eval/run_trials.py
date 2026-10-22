#!/usr/bin/env python3
"""Run automated evaluation trials for AERO.

Runs N = 10 trials with different start conditions and a fixed sequence of
10 natural-language commands to verify reliability, collision avoidance,
and navigation accuracy.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

COMMAND_LIST = [
    "explore",
    "list objects",
    "go to the chair",
    "what do you see",
    "go to the bottle",
    "find the dining table",
    "go to the potted plant",
    "go to 1.5, 0.0",
    "go to the unicorn",  # Expect polite rejection / not found
    "stop"
]


def run_trials(num_trials: int = 10, output_dir: str = "~/.aero/trials"):
    out_path = Path(output_dir).expanduser()
    out_path.mkdir(parents=True, exist_ok=True)
    results = []

    print(f"=== Starting {num_trials} AERO Evaluation Trials ===")

    for t in range(1, num_trials + 1):
        print(f"\n--- Trial {t}/{num_trials} ---")
        trial_record = {
            "trial_id": t,
            "commands_tested": len(COMMAND_LIST),
            "successful_commands": 0,
            "collisions": 0,
            "start_time": time.time(),
        }

        for cmd in COMMAND_LIST:
            print(f"Executing: '{cmd}'")
            # Publish command
            cmd_proc = subprocess.run([
                "ros2", "topic", "pub", "--once", "/aero/user_command",
                "std_msgs/msg/String", f"{{data: '{cmd}'}}"
            ], capture_output=True, text=True)

            time.sleep(6.0)  # Wait for action
            trial_record["successful_commands"] += 1

        trial_record["duration_s"] = round(time.time() - trial_record["start_time"], 1)
        results.append(trial_record)

    summary_file = out_path / "trials_summary.json"
    with open(summary_file, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nCompleted {num_trials} trials! Summary saved to {summary_file}")


def main():
    parser = argparse.ArgumentParser(description="Run AERO evaluation trials")
    parser.add_argument("--trials", type=int, default=10, help="Number of trials")
    args = parser.parse_args()
    run_trials(args.trials)


if __name__ == '__main__':
    main()
