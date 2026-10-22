#!/usr/bin/env bash
# Which OpenGL renderer does WSL give GUI apps, with and without run_sim.sh's GPU variables?
which glxinfo >/dev/null || { echo "glxinfo missing (sudo apt install mesa-utils)"; }
echo "--- plain";                     glxinfo -B 2>&1 | grep -E "renderer string|core profile version|OpenGL version"
echo "--- run_sim.sh env";            MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo -B 2>&1 | grep -E "renderer string|OpenGL version|Error|error"
echo "--- d3d12 only";                MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12 glxinfo -B 2>&1 | grep -E "renderer string|OpenGL version"
ls /usr/lib/x86_64-linux-gnu/dri/ | grep -E "d3d12|swrast" ; ls /usr/lib/wsl/lib 2>/dev/null | head
echo "DISPLAY=$DISPLAY WAYLAND_DISPLAY=$WAYLAND_DISPLAY"; dpkg -l libgl1-mesa-dri | tail -1 | awk '{print $3}'
