#!/usr/bin/env bash
set -e

# Kill any leftover background processes from previous runs
pkill -9 -f "rosbridge_websocket" || true
pkill -9 -f "ign gazebo" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "parameter_bridge" || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash

# Use ~/wcr_ws if available (avoids Windows path spaces breaking RViz libcurl mesh loading)
if [ -f "${HOME}/wcr_ws/install/setup.bash" ]; then
    source "${HOME}/wcr_ws/install/setup.bash"
    WS_DIR="${HOME}/wcr_ws"
else
    source "${SCRIPT_DIR}/install/setup.bash"
    WS_DIR="${SCRIPT_DIR}"
fi

# Enable NVIDIA GeForce RTX 5050 hardware acceleration in WSL2
unset LIBGL_ALWAYS_SOFTWARE
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Ensure Gazebo and RViz locate robot and world meshes
export IGN_GAZEBO_RESOURCE_PATH="${WS_DIR}/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/install/wcr_description/share:${GZ_SIM_RESOURCE_PATH}"

echo "=== Starting WCR Robot Simulation ==="
echo "GPU: NVIDIA GeForce RTX 5050 (Hardware Accelerated)"
echo "Active workspace: ${WS_DIR}"
ros2 launch wcr_launcher launcher.launch.py "$@"
