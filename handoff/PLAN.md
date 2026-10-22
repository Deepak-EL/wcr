# Wall-climbing dam inspection robot: full context and plan

> Saved from the original planning chat. Some statements here were later corrected;
> see `HANDOFF.md` ("Corrections to PLAN.md") before relying on them.

Context hand-off for continuing in Claude Code (VS Code, WSL). It covers everything
discussed so far: questions, answers, decisions, findings and the plan.

**Owner:** Girish. Laptop: Acer Nitro V 16S, RTX 5050 8 GB, Core 5 210H, 16 GB RAM.
WSL Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (`ign gazebo`).

---

## 1. Conversation so far (questions → answers)

**Q1. Should Claude build an STL of my wall-climbing robot (from my photos), or is STL a bad choice?**
- STL is fine, but it should be one mesh per moving part (body, each wheel, prop), with
  the origin on the joint axis.
- Detailed STL is for `<visual>` only; collisions use boxes and cylinders, because mesh
  collision makes wheel contact jittery.
- STL has no units: CAD exports mm and Gazebo uses m, so scale by 0.001. STL has no colour,
  so colours go in URDF materials.
- Gazebo cannot simulate suction or magnetism; adhesion is modelled as prop thrust.

**Q2. Here are photos plus parts: 3S 2200 mAh LiPo (off-board, 2 m XT60 extension), ESP32,
40 A ESC, L298N, 4× N20, A2212 2450 kV. Environment should be a dam.**
Built `wall_climber_ws.zip`, a ROS 2 workspace:
- **Model:** parametric CadQuery model (`tools/robot_params.py` holds every dimension)
  → 15 STLs + a STEP assembly.
  - Foam H-frame 300×200×5 mm, Ø172 mm prop hole, corner wheel notches.
  - Clear channel glued under the plate (sits only ~2.5 mm off the wall).
  - Motor on the channel, 6×4 prop, N20 + 43 mm wheels.
- **URDF/xacro** for Fortress and Harmonic.
  - Thrust: the `Thruster` plugin pushes along the prop joint axis into the wall.
  - Drive option `servo`: DiffDrive.
  - Drive option `dc`: ApplyJointForce + an N20 torque-speed node.
  - Also: IMU, ground-truth odometry, and the tether weight as a mass.
- **Dam world (generated):** 12 m gravity dam section, 24 m long, 3 monoliths.
  - Vertical upstream face above a drawn-down reservoir.
  - Crest with parapets; 3 m vertical downstream top, then a 0.75:1 (53°) slope.
  - Visual-only cracks, lift joints, stains and height marks; dry (μ 0.7) and wet (μ 0.45) variants.
  - Deployment cradles at the spawn points. World starts paused, thrust is latched, then auto-unpauses.
- **Nodes:**
  - `adhesion_controller`: thrust from the IMU surface angle, or fixed.
  - `dc_motor_drive`: N20 model, optional encoder PI loop.
  - `climb_test`: scripted climb / hold / descend, logged to CSV.
- **Checks run (Gazebo itself not run):**
  - xacro expands; unit tests pass.
  - MuJoCo independently confirms the thrust formula to within 1–2 %.
  - The generated URDF + dam boxes in MuJoCo climb every face.

**Key findings (old foam-board robot):**
- Load: robot 349 g + up to 80 g of hanging tether = 429 g; centre of mass 19 mm off the surface.
- Thrust needed to climb: `T ≥ m g (sin β / μ − cos β)`.
  - Dry vertical: 613 gf.
  - Wet vertical: 953 gf.
  - 53° slope: 231 gf.
  - Sideways traverse: 680–780 gf.
- An early "equal-torque" formula was wrong: a slipping wheel still pushes with μN, which MuJoCo confirmed.
- Available thrust: ~850 gf estimated. The 2 m lead drops ~0.9 V (−16 % thrust), so the robot gets ~715 gf.
  Margin on dry vertical is ~1.17×; wet does not work.
- Battery ~4 min at ~26 A.
- N20 open-loop PWM climbs at only ~28 mm/s at 50 % duty. At 0 % duty (L298N brake) the robot rolls
  down at ~50 mm/s because the gearbox back-drives. Fix: a holding PWM (~30 %) or encoders + PID.
- N20 reflected gearbox inertia (~1e-4 kg·m²) must be in the wheel inertia, or the torque loop
  through ROS goes unstable.
- To measure: thrust (kitchen-scale method, through the 2 m lead), μ (pull test on a slab),
  masses, lead gauge, N20 rating, prop size.

**Q3. How to run it in WSL?**
1. Copy the zip into `~` (not `/mnt/c`).
2. `apt install ros-humble-ros-gz ros-humble-xacro ...`, then `colcon build --symlink-install`.
3. `ros2 launch wall_climber_gazebo sim.launch.py`.
4. In a second terminal, `ros2 run wall_climber_control climb_test ...`, or drive with teleop on `/wall_climber/cmd_vel`.
- Use `LIBGL_ALWAYS_SOFTWARE=1` only if rendering fails.

**Q4. Is it autonomous? Path planning, obstacle avoidance, cameras, sensors? Can I give my own dam (Sketchfab) and robot STL?**
- Currently: Gazebo sim, not autonomous, IMU only.
- All of the above can be added in simulation.
- A Sketchfab dam is OK as the visual, with a hidden flat collision wall.
- Your own STL is OK: one per moving part.

**Q5. Uploaded files; check if usable. Wants path planning, RViz, Nav2, camera, all sensors.**

| File | What it is | Verdict |
|---|---|---|
| `dam_during_winter.glb` | Photo-scanned dam face, ~4.8×3.1 m as exported, 80k tris, 4× 4K textures, thin ragged surface | Usable as visual: scale ~×8, textures → 2K, hidden flat collision wall behind. Check Sketchfab license. |
| `robotChasissNew_1.stl` | New design, 319×529×225 mm, 655k tris, single merged mesh | Too heavy; use NoProp version |
| `robotChassisNoProp_1.stl` | Same without prop, 319×422×225 mm, 636k tris | **Use this.** Decimate to ~40k tris. |
| `wheel.stl` | Ø50.8 mm wheel | Usable |
| `activeWheel.stl` | Wheel + motor + gearbox module, 79×52×81 mm | Usable |
| `prop9Inch.stl` | 3-blade prop, measures ~138 mm (not 229 mm) | Units or export suspect; not needed (EDF chosen) |
| `edfProp.stl` | EDF fan rotor, ~68 mm | **Use this** as the thrust unit |

The new chassis is a different, larger robot than the photos. Thrust maths must be redone for it.

**Q6. Answers / decisions from Girish:**
- **No OpenCV.** Cracks are precoded: placed at known positions. A "detector" reports a crack
  when it is inside the wall camera's view; the camera image is still shown live.
- The Sketchfab dam has no cracks or plants, so **we add them ourselves** (decals + 3D models).
- **Thrust unit: EDF.**
- **The NoProp chassis already includes the wheels**, but also contains propeller motors that
  must be **removed**. Doable: the STL is made of separate pieces, so find and delete the motor
  parts, then render for confirmation.
- Mass and specs: go with Claude's estimates.
- **Thermal camera: no.**
- Wants autonomy with SLAM; asked whether a lidar is needed.
- MoveIt ("mov") is not needed: it is for arms; Nav2 does mobile planning.

**Q7. Laptop specs given. "So lidar won't help on a flat dam? How do we map and move without collision? Don't we use the OV camera?"**
- The RTX 5050 is fine. Use the latest NVIDIA *Windows* driver; WSL uses it automatically.
  Do not install a Linux NVIDIA driver inside WSL.
- The lidar does help with **collision avoidance**: it sees anything sticking out of the face.
  It is weak for **SLAM on a bare flat face**, because there is nothing to match.
- Approach:
  - **Known map:** a 2D map of the face generated from the dam model.
  - **Localization:** EKF (robot_localization) fusing wheel odometry + IMU, with lidar
    matching to edges, ledges and pipes correcting drift.
  - **Avoidance:** lidar (+ forward camera) feeds the Nav2 costmap.
  - **Inspection pattern:** a coverage node sends lawnmower-strip goals.
- slam_toolbox can still run as a demo.
- OV2640 (ESP32-CAM style) camera: simulated; a single camera has no depth, so it is for
  cracks and seepage, not collision.

**Q8. "But the dam has no obstacles, so this is useless."**
- We add obstacles ourselves:
  - Bushes and small trees growing from cracks.
  - Drain pipes and weep holes.
  - Ledges and walkway brackets.
  - Debris or a fallen branch that is not on the map (tests replanning).
  - Cracks and wet patches as flat decals.
- Pipes and ledges also give the lidar landmarks. Placement: fixed or randomised (launch option).
- Even with no obstacles, the planner keeps the robot within safe bounds: off the water,
  below the crest, inside the dam edges.

**Q9. "First make the 3D robot climb the 3D dam, then add sensors and planning. Is Claude Code in VS Code cheaper?"**
- Claude Code in VS Code (inside WSL) is better for this project. It does not use fewer tokens
  per action, but it can run Gazebo and colcon itself and read errors directly, which avoids
  the copy-paste debug loop.

---

## 2. Decisions locked in

| Item | Decision |
|---|---|
| Robot body | `robotChassisNoProp_1.stl`, propeller motors removed, decimated to ~40k tris |
| Wheels | Use the wheels already in the chassis (split into separate links); fall back to `wheel.stl` |
| Thrust | EDF (`edfProp.stl`) at the centre, blowing away from the wall → presses the robot on |
| Dam | `dam_during_winter.glb` scaled to real size, 2K textures, hidden flat collision wall, water at the base |
| Cracks | Precoded positions + decals; no OpenCV |
| Obstacles | Added by us: plants, pipes, drains, ledges, debris; fixed or random |
| Sensors | 2D lidar, wall-facing OV2640-style camera, forward camera, IMU, wheel encoders. No thermal. |
| Autonomy | Known map + EKF localization + Nav2 (map frame flat on the wall) + coverage planner; slam_toolbox optional demo |
| Viz | RViz: robot, map, costmap, path, lidar scan, both camera feeds, crack markers |
| MoveIt | Not used |
| Sim stack | ROS 2 Humble + Gazebo Fortress on WSL (Harmonic supported via `gz:=harmonic`) |

---

## 3. Plan

### Phase 1: robot climbs the Sketchfab dam (do first)
1. **Chassis cleanup.**
   - Load `robotChassisNoProp_1.stl` and split it into connected parts.
   - Identify and delete the propeller motors.
   - Split out the 4 wheels as separate meshes, each with its origin on its axle.
   - Decimate the body to ~40k tris. Render for Girish to confirm.
2. **EDF unit.** Place `edfProp.stl` as a spinning link at the centre; thrust axis = into the wall.
3. **Mass properties.** Estimate the mass of the new robot and EDF. Redo the thrust and
   friction maths (`design_math.py`) for the new size and EDF thrust.
4. **Dam world.**
   - Convert the GLB for Gazebo: scale it, Z-up, textures → 2K.
   - Fit a plane or boxes to the face for collision.
   - Add water at the base, then a spawn pose and cradle on the face.
5. **Reuse physics.** Thruster plugin, DiffDrive / DC drive, adhesion controller, climb_test.
6. **Verify** in MuJoCo here (URDF + collision wall), then in Gazebo on the laptop.

### Phase 2: sensors
- 2D lidar (gpu_lidar) scanning parallel to the face.
- Wall camera (OV2640-like resolution/FOV) and forward camera.
- IMU, joint states for the encoders.
- Bridge everything to ROS; view in RViz and `rqt_image_view`.

### Phase 3: inspection features
- Crack and wet-patch decals at known positions (YAML list).
- Plants, pipes, ledges and debris models (fixed or randomised).
- `inspection_detector` node: reports a crack or wet patch when it is in the wall camera's view,
  logs position and time, and publishes RViz markers.

### Phase 4: autonomy
- Generate the occupancy map of the face (frame `wall_map` lying on the face).
- robot_localization EKF (wheel odometry + IMU).
- Nav2: planner + controller + costmaps fed by the lidar.
- Coverage node: lawnmower waypoints over the face, avoiding obstacles, staying within bounds.
- Optional slam_toolbox demo.

### Phase 5: RViz config, launch options, README, results

---

## 4. Known risks
- WSL + GPU rendering for the lidar and cameras: works with the latest Windows NVIDIA driver;
  falls back to slow software rendering.
- Nav2 on a vertical wall is non-standard (a rotated map frame); expect tuning.
- The scanned dam mesh is ragged, so collision must use the fitted flat wall, not the scan.
- A real lidar (100–200 g) would hurt the thrust margin on hardware; fine in simulation.
- The Sketchfab license must allow use.

## 5. Existing workspace
`wall_climber_ws/`: see README.md. `tools/` regenerates everything from `robot_params.py`
(`python3 tools/build_all.py [--check]`). Results are in `results/`.
