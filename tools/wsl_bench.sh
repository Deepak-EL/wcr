#!/usr/bin/env bash
# Same-scene GL benchmark inside WSL. Modes: NVIDIA Intel software (default: all three).
#   tools/wsl_bench.sh [modes...]
D="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$D/tools/wslg_status.sh"
cp "$D/assets/bench_scene.npy" /tmp/bench_scene.npy     # read from the Linux disk, not /mnt/c
for mode in ${@:-NVIDIA Intel software}; do
  echo "=== $mode"
  # -k: the D3D12 driver can hang; force-kill after the grace period
  if [ $mode = software ]; then
    LIBGL_ALWAYS_SOFTWARE=1 timeout -k 5 150 python3 "$D/tools/bench_gl.py" /tmp/bench_scene.npy ${FRAMES:-40} 2>&1 | grep -v ATTENTION
  else
    MESA_D3D12_DEFAULT_ADAPTER_NAME=$mode vblank_mode=0 timeout -k 5 120 python3 "$D/tools/bench_gl.py" /tmp/bench_scene.npy ${FRAMES:-300} 2>&1 | grep -v ATTENTION
  fi
done
