#!/usr/bin/env bash
# run_all.sh - One-command launcher for the full AERO stack
# Usage: bash scripts/run_all.sh [--world house] [--headless] [--no-llm] [--load-previous]
set -e

WORLD="house"
HEADLESS="false"
USE_LLM="true"
LOAD_PREV="false"

while [[ $# -gt 0 ]]; do
  case $1 in
    --world) WORLD="$2"; shift 2 ;;
    --headless) HEADLESS="true"; shift ;;
    --no-llm) USE_LLM="false"; shift ;;
    --load-previous) LOAD_PREV="true"; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

source "$(dirname "$0")/env.sh"
cd ~/aero_ws

echo "========================================================"
echo " AERO Full Stack Launch"
echo "  world=$WORLD  headless=$HEADLESS  use_llm=$USE_LLM  load_previous=$LOAD_PREV"
echo "========================================================"

ros2 launch aero_bringup full.launch.py \
  world:="$WORLD" \
  headless:="$HEADLESS" \
  use_llm:="$USE_LLM" \
  load_previous:="$LOAD_PREV"
