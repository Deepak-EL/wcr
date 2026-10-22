#!/usr/bin/env bash
# Expand the URDF exactly as the launcher does and sanity-check it.
set -e
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
OUT=${1:-/tmp/wcr.urdf}
ARGS=$(python3 - <<'EOF'
import yaml, os
from ament_index_python.packages import get_package_share_directory as d
p = yaml.safe_load(open(os.path.join(d('wcr_launcher'), 'config', 'robot_params.yaml')))
print(' '.join(f'{k}:={str(v).lower() if isinstance(v, bool) else v}' for k, v in p.items()))
EOF
)
xacro "$(ros2 pkg prefix wcr_description)/share/wcr_description/urdf/wcr.urdf.xacro" \
  variant:=mock sim:=true propeller_control:=false namespace:=wcr $ARGS > "$OUT"
check_urdf "$OUT" | head -40
python3 - "$OUT" <<'EOF'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
m = sum(float(l.find('inertial/mass').get('value')) for l in r.iter('link') if l.find('inertial') is not None)
print(f'total mass {m:.3f} kg, links {len(list(r.iter("link")))}, joints {len(list(r.iter("joint")))}')
print('thrusters', [p.find('joint_name').text for p in r.iter('plugin') if p.find('joint_name') is not None])
print('mesh collisions', sum(1 for c in r.iter('collision') if c.find('geometry/mesh') is not None))
EOF
cd "/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim/wcr_climb" && python3 -m pytest -q test 2>&1 | tail -3
