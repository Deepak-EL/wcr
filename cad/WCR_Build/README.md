# WCR robot in Fusion 360

This folder rebuilds the real WCR robot in Fusion, part by part from the original CAD export. That includes the steering servos and pods, the wheels, the Jetson board with its fan, the EDF, the brackets, and every screw and nut.

## Build it (one time, about 5–15 min)
1. In Fusion, open **Utilities → Add-Ins → Scripts and Add-Ins**.
2. Under **Scripts**, click **+** and choose *this folder* (`WCR_Build`).
3. Select **WCR_Build** and click **Run**.

The script creates a new design and then does the following:
- It imports `wcr_robot.step`. This holds the parts that could be rebuilt exactly (screws, washers, standoffs, plates, boards and boxes), with true flat, cylindrical and arc faces.
- It converts each remaining part with Fusion's own **Convert Mesh → Prismatic**: the filleted 3D parts such as the servo brackets, fan, suction bell, nuts and tyres. Each part is converted once and then copied to every position where it occurs.
- It adds the tiny board details (resistors, pins, LEDs) as mesh bodies named `details`.
- It adds the joints.
4. Save the result with **File → Save**. That gives you a native Fusion design (`.f3d`).

Progress and any problems are written to `WCR_Build.log` in this folder.

Prismatic conversion needs Fusion 2025.07 or newer. In older versions those parts stay as mesh bodies. They still look right, but you can't edit them as solids.

## What you get
| Component | URDF link | Joint (already added) |
|---|---|---|
| `chassis` (grounded) | chassis_link | – |
| `edf_rotor` | edf_link | `edf_joint`, revolute about z, free |
| `FL/FR/BL/BR_steering_pod` | XX_steering_link | `XX_steering_joint`, revolute about z, −90°…+90° |
| `FL/FR/BL/BR_wheel` | XX_wheel_link | `XX_wheel_joint`, revolute about y, free |
| `sim_sensors` (lidar, wall camera, mounts) | added in the simulation | rigid |

Units are mm. The origin is the robot's `base_link`: z = 0 is the wall, x points forward (up the wall) and y points to the left.

To give the parts real materials, right-click a body or component and choose **Physical Material**. Then **Inspect → Properties** shows mass and inertia. The **fusion2urdf** add-in exports the joints, masses and inertias back to a URDF.

## Rebuilding this folder
```
C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe tools\build_chassis_cad.py
```
This reads the original meshes in `wcr_description/meshes` and writes everything to `cad/WCR_Build/`. It needs cadquery, trimesh, shapely, networkx and scipy.
