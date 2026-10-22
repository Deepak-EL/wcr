---
name: wcr-dam-sim-setup
description: How the WCR dam-climbing sim is built and tested (WSL paths, venv, headless test scripts, gotchas)
metadata:
  type: project
---
Repo `C:\Users\DEEPAK E L\Downloads\wcr_dam_sim` (upstream BCaran wcr_* packages + Girish's dam-inspection work, see PLAN.md phases 1-5). Phase 1 (robot climbs the Sketchfab dam) passed in headless Gazebo on 2026-10-16.

- WSL Ubuntu-22.04, ROS 2 Humble, Gazebo Fortress 6.18; `~/wcr_ws/src/*` are symlinks to the Windows checkout; symlink-install build via `tools/wsl_build.sh <pkgs>`.
- Windows Python venv for mesh tools: `C:\Users\DEEPAK E L\.wcrvenv` (trimesh, fast-simplification, pillow, pyyaml); run scripts with `-I`.
- Headless tests: `tools/wsl_sim_test.sh` (launch + `wcr_climb climb_test`), `tools/wsl_snapshot.sh` (camera frames), logs in `~/wcr_sim_logs` — WSL `/tmp` is wiped between commands.
- Call WSL with `wsl -d Ubuntu-22.04 -- bash <script>`; inline `bash -c` quoting through PowerShell breaks easily.
- Gotchas found: Fortress IMU orientation is relative to spawn pose unless `<orientation_reference_frame><localization>ENU`; wheel inertia must satisfy triangle inequality or the spawn is silently rejected; Thruster cmd_thrust is newtons (old scripts called it rad/s).

**Why:** these cost several debug rounds to discover.
**How to apply:** reuse the scripts for Phase 2+ checks; see [[wcr-adhesion-model]].
