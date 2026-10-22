#!/usr/bin/env bash
# Is the current WSLg session healthy? (copy mode = flicker/lag for every Linux window)
if grep -q "rdp_allocate_shared_memory: Failed" /mnt/wslg/weston.log 2>/dev/null; then
  echo "WSLg: COPY MODE (shared memory failed at startup)."
  echo "      Fix: in PowerShell run  wsl --shutdown , reopen Ubuntu, run this check again."
else
  echo "WSLg: OK (shared memory working)"
fi
