# WCR robot: simple model (18 parts)

This is a plain, beginner-style version of the WCR. Every part is a chamfered block, a tube, a plate or a simple revolve. Each one sits at the real position and size of the matching part in the original CAD. Electronics, the battery, screws and all small parts are left out.

- `wcr_simple.step`: the model (0.9 MB). It holds 18 kinds of part, placed 44 times. Copies are true instances. Colours follow the simulation: charcoal chassis, red steering pods, black wheels, aluminium sensor mast.
- `preview.png`: what it looks like.
- `WCR_Build.py`: an optional Fusion script. It opens the STEP and adds the joints.

## Open it
- **Just the model:** in Fusion, choose **File → Open → Open from my computer** and pick `wcr_simple.step`.
- **With joints:** go to **Utilities → Add-Ins → Scripts and Add-Ins**, click **+**, choose this folder, select **WCR_Build** and click **Run**. It takes under a minute. Then save with **File → Save** to get an `.f3d`.

## Parts
| Group | Part | Copies |
|---|---|---|
| Frame | square_tube (Ø30, 100 long) | 4 |
| | cross_tube (Ø25, 136 long) | 2 |
| | corner_bracket_left / corner_bracket_right (mirror pair): flat plate, tube corner block, cross-tube clamp block, solid servo tower | 2 + 2 |
| | tube_clamp (two halves) | 4 |
| Steering on top | steering_servo (plain block with output shaft) | 4 |
| Antennas | antenna (2 front, 1 rear, on the clamp blocks) | 3 |
| Suction + EDF | suction_bell, edf_duct, edf_motor | 1 each |
| | skirt_flap | 4 |
| | rotor (EDF fan) | 1 |
| Steering pod (red) | steering_fork (post, top plate, two tapered side plates), drive_motor (plain block) | 4 each |
| Wheel (black) | wheel (one solid part) | 4 |
| Simulation sensors | sensor_mounts, lidar, wall_camera | 1 each |

Components match the robot's links: `chassis`, `edf_rotor`, `FL/FR/BL/BR_steering_pod`, `FL/FR/BL/BR_wheel` and `sim_sensors`. The script adds these joints:
- steering: revolute about z, −90°…+90°;
- wheels: revolute about y;
- EDF: revolute about z;
- sensors: rigid.

Units are mm. The origin is the robot's `base_link`, with z = 0 on the wall.

## In the simulation
The same model also drives the robot's look in Webots and in RViz/Foxglove. Physics, sensors and control are unchanged; only the visuals differ. Add `model:=simple` to any run:
```
bash "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim/run_webots_ros.sh" winter_dam rviz:=false foxglove:=true mission:=true speed:=0.25 model:=simple
```
Without `model:=simple`, the original detailed robot is used, exactly as before.

## Rebuild after a change
1. Run `C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe tools\build_simple_cad.py`. This writes the STEP, the preview and the Webots meshes in `wcr_description/meshes/simple/`. Every size is a number in that script.
2. Run `C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe tools\build_webots.py --simple`. This writes `webots/protos/WcrSimple.proto` and `wcr_webots/resource/wcr_simple.urdf`.
3. In WSL, run `bash tools/wsl_build.sh wcr_webots wcr_description`.

The detailed version, with every original part, is still in `cad/WCR_Build/`.
