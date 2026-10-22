# HANDOFF: WCR dam-inspection simulation

State as of 2026-10-16, written for the next Claude Code session (and Girish) after the move
from Windows/WSL to native Ubuntu. Read this first, then `handoff/PLAN.md` (the original plan;
the corrections below take priority over it).

**To resume, tell Claude:** "Read HANDOFF.md and handoff/PLAN.md, then continue with the next steps."

---

## 1. Project in one paragraph

Girish is building a wall-climbing robot to inspect a dam, simulated in ROS 2 Humble +
Gazebo Fortress. The robot is the upstream `wcr` 4WIS4WID design (4 steerable, driven wheels,
Dynamixel-style actuators) with the 4 side propellers removed; a central 70 mm EDF presses it
onto the surface. The dam is the Sketchfab scan "Dam during winter" (CC-BY-4.0, author
orphanrtg), scaled ×25. **Phase 1 (robot climbs the dam) is done and verified in headless Gazebo.**
Phases 2–5 (sensors, cracks/obstacles, Nav2 autonomy, docs) are not started.

Hardware: Acer Nitro V 16S, RTX 5050 8 GB, Core 5 210H, 16 GB RAM.

## 2. Where we stopped (the next steps Girish agreed to)

1. **Move to native Ubuntu 22.04 (dual boot).** Under WSLg the Gazebo window is choppy (every
   frame goes through the WSLg compositor) and `ogre2` crashes. See section 9.
2. **Replace the scanned dam visual with a clean modelled dam.** Girish dislikes the faceted
   look. The plan: same profile measured from the scan (section 5), smooth geometry, sharp tiling
   concrete texture (4K, PBR), cracks/stains/plants as decals and models (Phase 3), scan kept
   only as distant background (optional). Cheap partial fix if needed: merge the scan's vertices
   and recompute smooth normals (it has no shared vertices, so it is flat-shaded) and use the
   original 4K textures instead of 2K.
3. **Colour the robot.** Split the CAD bodies into groups (frame, servos, electronics/PCB,
   EDF, antennas, wheels/tyres) with one material each, or use an original CAD export with
   colours if Girish provides one (STEP/GLB).
4. Then Phase 2 (sensors) per PLAN.md.

Wait for Girish's go-ahead between steps; he likes to review each result (renders) first and
prefers short answers.

## 3. Open questions for Girish

- Keep the empty side-arm tubes removed? (Currently removed; `chassis_body_arms.obj` keeps them.)
- Is the dam scale ×25 right? (Chosen from crest width ~3.4 m, hut, parapet height.)
- Measurements still needed for real numbers: EDF thrust through the 2 m lead, skirt suction
  (pressure under the skirt), wheel-on-concrete friction (pull test), component masses.
- Does he have the original CAD (for clean colours/parts)?

## 4. What exists in this folder

Copied from the `robro` checkout (BCaran's upstream `wcr` workspace + earlier edits + this work)
without build output, git history or unused files. Not a git repo yet.

| Path | What |
|---|---|
| `wcr_description/urdf/wcr_base.xacro` | Rewritten: side props behind `side_propellers` switch; box collisions (skirt + deck); cylinder wheel collisions; realistic inertias; EDF axis `0 0 -1`; wheel units via one macro (same link/joint names) |
| `wcr_description/urdf/wcr_params.xacro` | New args: `side_propellers`, `chassis_mesh`, `chassis_com_*`, `chassis_i*`, `steering_i`, `wheel_radius/width/i_axial/i_radial`, `edf_i` |
| `wcr_description/urdf/wcr.gazebo` | EDF Thruster (`velocity_control false`, small `p_gain`, 0–45 N); side-prop thrusters conditional; OdometryPublisher (ground truth, world frame, 3D); IMU `orientation_reference_frame` ENU; wheel μ 1.0 |
| `wcr_description/urdf/mock_wcr_classic.ros2_control.xacro` | Side-prop joints conditional |
| `wcr_description/meshes/chassis_body.obj` | Chassis, side motors + arm tubes removed, simplified per CAD body within 0.5 mm (576k → 248k tris), crease-angle smooth normals. `chassis_body_arms.obj` keeps the tubes; `active_wheel.obj` (22k) and `wheel.obj` (5.7k) likewise. Robot total ~360k tris. Originals (`robotChassisNoProp.stl`, etc.) untouched. Regenerate: `tools/clean_chassis.py --tol 0.5 --keep 0.1` |
| `wcr_description/worlds/dam_world.sdf` | Generated dam world (do not hand-edit) |
| `wcr_description/worlds/dam_geometry.yaml` | Generated planes, fillet, spawn pose, cradle; launcher reads the spawn pose from it |
| `wcr_description/worlds/meshes/dam/` | `dam.obj` + 4 × 2K JPEG textures + `ATTRIBUTION.txt` (CC-BY credit, required) |
| `wcr_description/launch/gazebo.launch.py` | Gazebo server and GUI as separate processes; GUI respawns if it crashes; 6-DoF spawn; `/clock` bridge from the world name |
| `wcr_launcher/config/robot_params.yaml` | Estimated masses (2.0 kg total), new params, EDF max 45 N |
| `wcr_launcher/config/launch_params.yaml` | `world_file: dam_world.sdf`, `world_geometry`, `adhesion: auto`, `adhesion_force`, spawn roll/pitch |
| `wcr_launcher/config/wcr_controller.yaml` | EDF-only joint lists; old version in `wcr_controller_side_props.yaml` |
| `wcr_launcher/wcr_launcher/yaml_loader.py` | `WCR_<KEY>` env vars override launch params; `world_name`; spawn from geometry yaml |
| `wcr_launcher/launch/launcher.launch.py` | Starts `adhesion_controller`; passes `robot_namespace` to `inv_kin_controller` |
| `wcr_climb/` (new ament_python pkg) | `adhesion.py` (formula), `adhesion_controller` node, `climb_test` node, unit tests (3 pass) |
| `tools/clean_chassis.py` | Removes side motors/tubes from the CAD STL, exports smooth OBJ |
| `tools/build_dam_world.py` | GLB → aligned, scaled, flattened OBJ + SDF + geometry yaml. `--glb` path required, `--scale 25`, `--mu 0.7`, `--fillet 2.5`, `--spawn-up 1.5` |
| `tools/design_math.py` | Mass budget, CoM/inertia from CAD, required adhesion table |
| `tools/wsl_*.sh` | WSL test helpers: build, xacro check, headless climb test, snapshot, close-up render, GUI tests. They work on native Linux too (they only use `~/wcr_ws`), except paths noted in section 9 |
| `tools/grab_image.py`, `tools/climb_log_summary.py` | Save a ROS image as PPM; summarise a climb CSV |
| `LOCAL_SETUP.md` | User run guide, has a "Dam climbing" section |
| `handoff/` | This plan, plus `handoff/memory/` (Claude's notes; copy into the new Claude memory dir if wanted) |

Source dam scan for `tools/build_dam_world.py`: `assets/dam_during_winter.glb`.

## 5. Key numbers

**Dam (world frame: x along the dam, face toe at y = 0, floor z = 0, face normal toward −y):**
downstream slope 53.55°, 14.75 m from toe to the curve; curved transition R 2.5 m (scan shows
~2.7 m), z 11.87–13.34 m; vertical wall 89.6°, 2.0 m, up to z 15.34 m; then a cornice under the
crest (crest z 16.29 m) that the robot cannot pass. Working section 53.8 m wide (right of the
spillway). Scan flatness before flattening ~7 cm rms; visual now within 2 cm (p99) of collision.
Spawn: 1.5 m up the slope on a yellow cradle ledge.

**Robot (estimates, nothing weighed):** total 2.01 kg incl. 80 g hanging tether; chassis 1.45 kg,
CoM (−9, 0, 73) mm in `chassis_link`, 83 mm above the surface; wheelbase 0.225 m; wheel r 25.4 mm;
max wheel speed 5.04 rad/s = 0.128 m/s.

**Adhesion** F (EDF thrust + skirt suction) = `max(m g (sin β/μ − cos β), m g sin β · h/(L/2))`:

| Surface | Required | ×1.5 (used) |
|---|---|---|
| slope 53.5°, μ 0.7 | 11.7 N (peel-limited) | 17.6 N |
| slope wet μ 0.45 | 23.5 N | 35.3 N |
| wall 89.6°, μ 0.7 | 28.0 N | 42.0 N |
| wall wet μ 0.45 | 43.7 N | 65.5 N |

A 70 mm EDF gives ~10 N thrust, so the real robot needs skirt suction (~650 Pa over 304 cm² for
the dry wall). The sim's 45 N cap is a placeholder.

## 6. Verified results (headless Gazebo, WSL, 2026-10-16)

- Full climb slope → curve → wall: z 1.2 → 14.5 m at 0.119–0.120 m/s (commanded 0.12),
  adhesion 17.6 → 28.5 → 42 N automatically, hold on the wall: 0.0 mm drift. PASS.
- 3 m climb / hold / descend on the slope: 99.7 % speed, zero creep. PASS.
- Threshold check: fixed 8 N → wheels spin, no climb; fixed 13 N → climbs (formula: 11.7 N).
- With a *sharp* slope-to-wall crease the robot stalls at the corner (correct physics); the
  fillet fixes it, matching the real curved dam.
- Not tested: wet friction world (`--mu 0.45`), `ramp_world.sdf` after the changes, the
  side-propeller configuration, RViz display of the OBJ meshes.

## 7. Bugs found and fixed (worth remembering)

- Old scripts sent "rad/s" to the Thruster `cmd_thrust`, which is **newtons** (old ramp run ≈ 320 N on 3.5 kg).
- `inv_kin_controller` built topics from an empty `robot_namespace` → listened on `/cmd_vel`;
  `/wcr/cmd_vel` never reached the wheels. Launcher now passes `/wcr`.
- Fortress IMU orientation is relative to the spawn pose unless
  `<orientation_reference_frame><localization>ENU</localization>` is set.
- Wheel inertia breaking the triangle inequality (Ixx+Izz ≥ Iyy) makes Gazebo silently refuse
  to spawn the model ("invalid inertia", then controller_manager never appears).
- Old EDF axis +z pushed the robot off the wall; EDF Thruster with `velocity_control true` and
  large rotor inertia spins the chassis; full-mesh collisions (636k tris) made contacts slow.
- Below the peel force (~11.7 N) the front lifts and the robot pivots off the face; the 26 mm
  "gap" seen once was this, not a sign error.
- Aggressive whole-mesh decimation (636k → 50k) made the chassis spiky. Full detail (1.1M tris
  for the robot) made Gazebo block ~16 s at startup, so Humble's controller spawner (hard-coded
  10 s load timeout; `--service-call-timeout` doesn't cover load) failed and the wheels never
  started. Fix: per-body simplification with a 0.5 mm error bound (robot ~360k tris, looks the
  same) + `wcr_climb/wait_for_sim` gates the spawners until the robot is up and sim time advances.
- Gazebo GUI segfaulted once at startup on WSLg (Qt/GLX context teardown, intermittent,
  not reproduced in 6 runs); GUI is now a separate respawning process.

## 8. How to run / test

```bash
# build (symlink install; ~/wcr_ws/src/* are symlinks to the repo)
bash tools/wsl_build.sh wcr_description wcr_launcher wcr_climb
# full sim with GUI + RViz
bash run_sim.sh
# scripted climb (2nd terminal)
ros2 run wcr_climb climb_test --ros-args -r __ns:=/wcr -p use_sim_time:=true -p distance:=3.0
# headless end-to-end test (PASS/FAIL), logs in ~/wcr_sim_logs
bash tools/wsl_sim_test.sh -p distance:=16.0 -p speed:=0.12 -p descend:=false -p timeout:=300.0
python3 tools/climb_log_summary.py ~/wcr_sim_logs/climb.csv 10
# fixed adhesion / other world via env overrides
WCR_ADHESION=fixed WCR_ADHESION_FORCE=13.0 bash tools/wsl_sim_test.sh -p distance:=1.5
# regenerate assets (Python venv with: trimesh numpy scipy networkx fast-simplification pillow pyyaml)
python3 tools/clean_chassis.py
python3 tools/build_dam_world.py --glb assets/dam_during_winter.glb
python3 tools/design_math.py
```

## 9. Moving to native Ubuntu 22.04

1. Install Ubuntu **22.04.5** (HWE kernel) alongside Windows; NVIDIA driver **≥ 570**
   (`sudo ubuntu-drivers install`, check `nvidia-smi`). RTX 5050 is too new for older drivers.
2. ROS 2 Humble desktop + `ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-xacro
   ros-humble-ros2-controllers ros-humble-rosbridge-server ros-humble-teleop-twist-keyboard`
   (Gazebo Fortress comes with ros-gz on Humble). Later phases: `ros-humble-navigation2
   ros-humble-nav2-bringup ros-humble-robot-localization ros-humble-slam-toolbox`.
3. Copy the whole `wcr_dam_sim` folder from the Windows drive to the Linux home (e.g. `~/wcr_dam_sim`);
   don't work from the NTFS mount.
4. `mkdir -p ~/wcr_ws/src && ln -s ~/wcr_dam_sim/wcr_* ~/wcr_ws/src/`, then
   `cd ~/wcr_ws && colcon build --symlink-install` (builds the C++ controllers too).
5. Python venv for `tools/`: `python3 -m venv ~/.wcrvenv && ~/.wcrvenv/bin/pip install trimesh numpy scipy networkx fast-simplification pillow pyyaml`.
6. WSL-specific bits to adjust:
   - `run_sim.sh` and the test scripts export `MESA_D3D12_DEFAULT_ADAPTER_NAME` (harmless on
     Linux). On a hybrid-graphics laptop keep `__NV_PRIME_RENDER_OFFLOAD=1` and
     `__GLX_VENDOR_LIBRARY_NAME=nvidia` so Gazebo uses the RTX GPU.
   - Try `--render-engine-gui ogre2` (and `<render_engine>ogre2</render_engine>` in the world;
     edit the template in `tools/build_dam_world.py`), which is faster than `ogre`.
   - `LOCAL_SETUP.md` commands use `/mnt/c/...` paths; replace with `~/wcr_dam_sim`.
7. Install VS Code + the Claude Code extension, sign in, open `~/wcr_dam_sim`, and say:
   "Read HANDOFF.md and handoff/PLAN.md, then continue with the next steps."
   Optionally copy `handoff/memory/*.md` into the new project's Claude memory folder.

## 10. Corrections to PLAN.md

- `robotChassisNoProp.stl` **does not contain wheels**; wheels come from `activeWheel.stl`
  (steering pod) + `wheel.stl`, which the upstream URDF already used.
- The "propeller motors" were 2 back-to-back motor pairs on angled side arms (|y| ≈ 190 mm),
  4 motors total, matching the upstream URDF's 4 side props.
- Do **not** decimate the chassis to ~40k tris: it looks bad and isn't needed (collision is boxes).
- Dam scale is ×25 (not ×8); the scan's downstream face is a 53.5° slope + short vertical wall,
  not a tall vertical face; the reservoir is frozen at crest level, so there is no water at the base.
- Gazebo Fortress cannot load GLB; the dam is OBJ + JPEG.
- The work lives in the upstream `wcr_*` packages (this folder), not in `wall_climber_ws`.
- On native Linux, install the NVIDIA Linux driver (the "don't install a Linux driver" advice was WSL-only).
