#!/usr/bin/env bash
# AERO environment check - prints PASS or FAIL for each requirement
set -euo pipefail

PASS=0
FAIL=0

check() {
    local label="$1"
    local result="$2"
    if [ "$result" = "PASS" ]; then
        echo "  PASS  $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL  $label  ($result)"
        FAIL=$((FAIL + 1))
    fi
}

echo "============================================================"
echo " AERO Environment Check"
echo "============================================================"

# Ubuntu 24.04
if grep -q "Ubuntu 24.04" /etc/os-release 2>/dev/null; then
    check "Ubuntu 24.04" "PASS"
else
    VERSION=$(grep PRETTY_NAME /etc/os-release | cut -d'"' -f2)
    check "Ubuntu 24.04" "FAIL: $VERSION"
fi

# ROS 2 Jazzy
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then
    check "ROS 2 Jazzy" "PASS"
else
    check "ROS 2 Jazzy" "FAIL: ROS_DISTRO=${ROS_DISTRO:-unset}"
fi

# gz sim
if gz sim --version 2>/dev/null | grep -q "Harmonic\|8\."; then
    check "Gazebo Harmonic (gz sim)" "PASS"
elif command -v gz &>/dev/null; then
    VER=$(gz sim --version 2>&1 | head -1)
    check "Gazebo Harmonic (gz sim)" "WARN: $VER (verify Harmonic)"
    PASS=$((PASS + 1))
    FAIL=$((FAIL - 1))
else
    check "Gazebo Harmonic (gz sim)" "FAIL: gz not found"
fi

# Python 3.12
PY_VER=$(python3 --version 2>&1 | awk '{print $2}')
if echo "$PY_VER" | grep -q "^3\.12"; then
    check "Python 3.12" "PASS"
else
    check "Python 3.12" "FAIL: $PY_VER"
fi

# RAM >= 14 GB
RAM_GB=$(free -g | awk '/Mem:/{print $2}')
if [ "${RAM_GB:-0}" -ge 14 ]; then
    check "RAM >= 14 GB (${RAM_GB} GB)" "PASS"
else
    check "RAM >= 14 GB (${RAM_GB:-?} GB)" "FAIL"
fi

# CPU count >= 4
CPUS=$(nproc)
if [ "${CPUS:-0}" -ge 4 ]; then
    check "CPU count >= 4 ($CPUS cores)" "PASS"
else
    check "CPU count >= 4 ($CPUS cores)" "FAIL"
fi

# WSLg / display
if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
    check "DISPLAY or WAYLAND_DISPLAY set (WSLg)" "PASS"
else
    check "DISPLAY or WAYLAND_DISPLAY set (WSLg)" "FAIL: set DISPLAY or WAYLAND_DISPLAY"
fi

# Optional: nvidia-smi
if command -v nvidia-smi &>/dev/null; then
    GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    echo "  INFO  GPU: $GPU (nvidia-smi found)"
else
    echo "  INFO  GPU: nvidia-smi not found (CPU-only mode will be used)"
fi

# venv
if [ -f ~/aero_venv/bin/activate ]; then
    check "aero_venv exists" "PASS"
else
    check "aero_venv exists" "FAIL: run 'python3 -m venv --system-site-packages ~/aero_venv'"
fi

# Python packages in venv
if [ -f ~/aero_venv/bin/activate ]; then
    source ~/aero_venv/bin/activate
    for pkg in ultralytics anthropic fastapi uvicorn scipy pandas matplotlib pytest; do
        if python3 -c "import $pkg" 2>/dev/null; then
            check "Python pkg: $pkg" "PASS"
        else
            check "Python pkg: $pkg" "FAIL: pip install $pkg"
        fi
    done
fi

echo "============================================================"
echo " Results: $PASS PASS  |  $FAIL FAIL"
echo "============================================================"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
