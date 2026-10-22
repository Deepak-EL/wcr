#!/usr/bin/env bash
# Why is WSLg in copy mode? Look at its shared-memory mount and logs.
echo "--- shared memory mounts"; mount | grep -iE "shared_memory|wslg" | cut -c1-150
ls -la /mnt/shared_memory 2>&1 | head -3
echo "--- weston log hints"
grep -iE "shared|copy|rail|shm|fallback|error|fail" /mnt/wslg/weston.log 2>/dev/null | head -25 | cut -c1-200
echo "--- wslg config"; cat /mnt/c/Users/*/.wslgconfig 2>/dev/null; cat "/mnt/c/Users/DEEPAK E L/.wslconfig" 2>/dev/null
