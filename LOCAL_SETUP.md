# Local Simulation & Wall Climbing Guide - WCR 4WIS4WID Robot

This ROS 2 workspace simulates a **Wall Climbing Robot (WCR)** equipped with a 4-wheel independent steering & drive system (4WIS4WID) and a central Electric Ducted Fan (EDF). The original 4 side propellers are switched off (`side_propellers: false` in `wcr_launcher/config/robot_params.yaml`).

---

## Dam climbing (default world)

The default world is `dam_world.sdf`: the Sketchfab scan *"Dam during winter"* by orphanrtg
(CC-BY-4.0), scaled x25 (about 120 m long, 16 m high). The robot spawns on the downstream face,
on a small yellow cradle, and the EDF adhesion controller holds it there.

| Surface | Angle | Length | Adhesion needed (x1.5) |
|---|---|---|---|
| downstream slope | 53.5 deg | 14.8 m | 17.6 N |
| curved transition (R 2.5 m) | 53.5 -> 90 deg | 1.6 m | up to 42 N |
| upper wall | 89.6 deg | 2.0 m | 42 N |

Build (WSL) and run:
```bash
bash "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim/tools/wsl_build.sh" wcr_description wcr_launcher wcr_climb
cd "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim" && ./run_sim.sh
# second terminal: scripted climb -> hold -> descend, prints PASS/FAIL
ros2 run wcr_climb climb_test --ros-args -r __ns:=/wcr -p use_sim_time:=true -p distance:=3.0
# or drive by hand (forward = up the face)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/wcr/cmd_vel
```
Headless test without GUI/RViz: `bash tools/wsl_sim_test.sh -p distance:=3.0`.

- **Adhesion** (`wcr_climb/adhesion_controller`): `thrust/edf` is the total force pressing the robot
  on (fan thrust + skirt suction), `1.5 x m g max(sin b / mu - cos b, sin b h / (L/2))` from the IMU
  surface angle, clamped to 45 N. Set `adhesion: fixed` + `adhesion_force:` in `launch_params.yaml`
  for a constant force. Don't run `climb_ramp.py` at the same time (both publish `thrust/edf`).
- **Masses are estimates** (2.0 kg total, `tools/design_math.py`). A 70 mm EDF gives ~10 N of thrust,
  so the real robot needs ~0.6 kPa of skirt suction for the vertical wall. Measure thrust, suction
  and wheel/concrete friction, then update `robot_params.yaml` and the controller parameters.
- **Regenerating assets**: `tools/clean_chassis.py` (removes the side motors from the chassis STL,
  decimates it) and `tools/build_dam_world.py --glb <path to dam_during_winter.glb>` (scales and
  aligns the scan, flattens the face onto the fitted planes, writes `dam_world.sdf` and
  `dam_geometry.yaml`). Needs `pip install trimesh numpy scipy fast-simplification pillow pyyaml`.
- Any launch parameter can be overridden with an env var, e.g. `WCR_WORLD_FILE=ramp_world.sdf WCR_WORLD_GEOMETRY= ./run_sim.sh`.

---

## ⚡ Performance: Hardware Acceleration (NVIDIA RTX 5050)

Your laptop has an **NVIDIA GeForce RTX 5050 (16GB VRAM)** and an **Intel Core 5 210H CPU**.

### Why it was previously slow:
1. To bypass an initial Ogre2 crash on WSLg, software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) was temporarily active, forcing the CPU to rasterize 3D graphics in software.
2. WSLg by default routes OpenGL to the integrated Intel GPU rather than your discrete NVIDIA GPU.

### How it is accelerated now:
- `MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"` is exported in `run_sim.sh`, binding directly to your **RTX 5050**.
- Gazebo GUI is configured with `--render-engine-gui ogre`, giving full native hardware GPU acceleration with high FPS.

---

## 🚀 How to Launch the Simulation

In your WSL terminal:
```bash
cd "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim"
./run_sim.sh
```

This starts:
- **Ignition Gazebo (Fortress 6.18)** with `ramp_world.sdf`
- **RViz2** displaying the 3D robot model centered at `base_link`
- **ros2_control** hardware interface and joint controllers
- **Inverse Kinematics Controller** (`inv_kin_controller`)
- **Odometry & Sensor Bridges**

---

## 🧗 How to Move & Climb the Ramp / Wall

### How Wall Climbing Works on WCR:
WCR stands for **Wall Climbing Robot**. When a wheeled vehicle hits a steep ramp or vertical wall, gravity pulls it down. WCR uses **aerodynamic suction / downforce**:
- **4 Propellers + 1 central EDF ducted fan** push air upwards, creating strong normal force ($N$) pushing the robot flat against the surface.
- The normal force generates tyre friction ($F_f = \mu \cdot N$), allowing the wheels to drive straight up the curved ramp and climb vertical walls!

### Method 1: Using the automated Climber script (Recommended)
Open a second WSL terminal while the simulation is running:
```bash
cd "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash

# Option A: Drive with wheels only (test basic driving and ramp approach)
python3 climb_ramp.py --drive-only --speed 0.35

# Option B: Climb ramp & wall with aerodynamic downforce adhesion
python3 climb_ramp.py --speed 0.35 --prop-speed 60.0 --edf-speed 80.0
```
- `--speed`: Forward driving velocity in m/s (default: 0.35)
- `--prop-speed`: Propeller angular velocity in rad/s (default: 60.0 rad/s produces ~40 N downforce)
- `--edf-speed`: EDF fan angular velocity in rad/s (default: 80.0 rad/s)
- `--drive-only`: Disables suction and tests pure wheel driving
- `--duration`: Run time in seconds (default: 25)

### Method 2: Manual ROS 2 Driving (Wheels Only)
In a second terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash

# Drive the robot forward towards the ramp:
ros2 topic pub /wcr/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.35, y: 0.0, z: 0.0}}" -r 20
```

### Method 3: Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/wcr/cmd_vel
```

---

## 🛠️ Modifying & Rebuilding Code
Your workspace files are linked between Windows and WSL:
- Windows: `C:\Users\DEEPAK E L\Downloads\wcr_dam_sim`
- WSL: `~/wcr_ws/src`

Whenever you edit any code, rebuild with:
```bash
cd ~/wcr_ws
colcon build
```
