---
name: wcr-adhesion-model
description: Robot mass/adhesion assumptions for the EDF-only WCR and which numbers are unmeasured estimates
metadata:
  type: project
---
EDF-only robot (side props removed, `side_propellers: false`). Total mass ~2.0 kg is an ESTIMATE (user said to go with Claude's estimates); chassis CoM/inertia from CAD volumes (`tools/design_math.py`).
Sim models EDF as one adhesion force (thrust + skirt suction), max 45 N placeholder. Required x1.5: slope 53.5 deg 17.6 N, vertical wall 42 N (mu 0.7). A 70 mm EDF gives ~10 N, so the real robot depends on ~0.6 kPa skirt suction — unmeasured.
Gazebo threshold check matched the formula: 8 N fails, 13 N climbs (predicted 11.7 N).

**Why:** these numbers drive every later phase; user still needs to measure thrust, suction, mu, masses.
**How to apply:** treat them as placeholders; ask for measurements before claiming hardware feasibility. Related: [[wcr-dam-sim-setup]].
