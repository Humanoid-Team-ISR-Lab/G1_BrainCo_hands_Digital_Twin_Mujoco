# G1_BrainCo_hands_Digital_Twin

A MuJoCo digital twin of the Unitree G1 humanoid (29-DOF body) fitted with
BrainCo dexterous hands, driven entirely over DDS - the same transport,
topics, message types, and control convention used on the real robot.
A control script written against this simulation talks to real hardware
by changing two lines (network domain and interface); nothing else in the
control code needs to know whether it's talking to sim or a physical G1.

It's built on top of Unitree's own [`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco)
simulator, extended with native BrainCo hand support, which stock
`unitree_mujoco` doesn't include.

## Why this exists

Unitree's `unitree_mujoco` simulates the G1 body over DDS out of the box,
which is great for whole-body control, but has no concept of a BrainCo
hand - there's no matching bridge, no verified actuator/joint model for
it, and no examples showing the two working together. This repository
fills that gap: a G1 body + BrainCo hands combination, simulated as one
coherent DDS-addressable robot, so hand and body controllers can be
developed and tested against a common testbed before either touches real
hardware.

## What's included / what's new compared to stock `unitree_mujoco`

| Feature | Status |
|---|---|
| G1 29-DOF body over `rt/lowcmd` / `rt/lowstate` | Adapted from upstream `unitree_sdk2py_bridge.py`, trimmed to a fixed motor count (see [Architecture](#architecture)) |
| **BrainCo hand simulation over DDS** | **New.** `brainco_sim_bridge.py` mirrors the real robot's `brainco_hand.service` - same topics (`rt/brainco/{side}/cmd`/`state`), same message shape, same position/speed convention |
| **Verified hand actuator + tendon-coupling model** | **New.** Cross-checked against BrainCo's own official Revo2 MuJoCo model - see [Hand model](#hand-model) below |
| Elastic band (virtual lift/support) | Ported from upstream's H1/G2 support, toggle via `config.py` |
| **Camera switching in the viewer** | **New.** Press `C` to cycle between free/orbit view and the robot's head-mounted egocentric camera |
| **DDS example scripts** | **New.** Read-only state monitor and an interpolated whole-body + hand motion test, both standalone. So they double as a template for real-robot scripts |
| Self-contained repository | Mesh assets and the built scene are included directly - no external asset dependency to clone or download separately |

## Scope

- **Body**: full 29-DOF, low-level control only - `rt/lowcmd` /
  `rt/lowstate`, `unitree_hg` messages. The high-level `rt/arm_sdk`
  channel (commanding only the arms while onboard firmware holds the
  legs) is intentionally **not** simulated; every controller in this
  project targets low-level topics exclusively. This may be added later.
- **Hands**: BrainCo, position-only, matching the real hand exactly.
  Commands and state are both a single `q` value per finger DoF in
  `[0.0, 1.0]` (0 = open, 1 = closed).

## Architecture

Two independent DDS bridges run inside `unitree_mujoco.py`, each owning a
disjoint slice of the MuJoCo model:

- **`body_bridge.py`** owns the first 29 actuators (legs, waist, arms) and
  the first 87 sensors (joint pos/vel/torque) plus the IMU. It converts
  `rt/lowcmd` motor commands into torques via `tau + kp*(q_err) +
  kd*(dq_err)`, written straight into MuJoCo's `ctrl`, and publishes
  `rt/lowstate` back from sensor readings. This mirrors upstream
  `unitree_mujoco`'s own approach almost exactly, with one deliberate
  change: upstream indexes `ctrl[0:model.nu]` on the assumption that every
  actuator in the scene belongs to the body. That assumption breaks once
  hand actuators are added to the same model, so this bridge hard-codes
  `NUM_BODY_MOTOR=29` instead and never touches anything past it.
- **`brainco_sim_bridge.py`** owns the 6 hand actuators per side (12
  total) and speaks a completely separate set of topics
  (`rt/brainco/{left,right}/cmd`/`state`), matching the real hand's own
  service rather than being folded into the body's `LowCmd`/`LowState`
  messages - because that's how the real robot splits it too (see
  [Hand model](#hand-model)).

Both bridges run against the same live `MjModel`/`MjData`, stepped once
per tick in `unitree_mujoco.py`'s simulation loop - from a DDS client's
point of view they're just two more publishers/subscribers on the bus,
indistinguishable in principle from the real robot's own processes.

## Hand model

BrainCo's Revo2 hand has 6 actively-driven joints (thumb flex, thumb
abduction/adduction, and one flex joint per remaining finger) and 11
total degrees of freedom - the extra 5 are passive, tendon-coupled
followers of their corresponding active joint, not independently
actuated. This was verified directly against BrainCo's own official
MuJoCo model for the hand (downloadable from their
[Revo2 documentation site](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/download.html)),
which defines exactly 6 `<position>` actuators per hand and drives the
passive joints via MJCF `<equality>` joint constraints:

```
distal = 1.0   * proximal   (index finger)
distal = 1.155 * proximal   (thumb, middle, ring, pinky)
```

`scenes/g1_brainco/g1_brainco_body.xml` reproduces this exactly - see
`scenes/g1_brainco/README.md` for the full derivation and the one open
question that has no authoritative answer yet (the fingertip joints,
which aren't modeled consistently even in BrainCo's own left- vs
right-hand files).

## Repository layout

```
config.py                    Robot/topic/DDS configuration for the sim entry point
unitree_mujoco.py             Entry point: loads the scene, runs physics, wires up both bridges + viewer controls
body_bridge.py                 rt/lowcmd + rt/lowstate for the 29 body motors; also hosts ElasticBand
brainco_sim_bridge.py          rt/brainco/{side}/cmd + /state for the hands
viewer_controls.py             Camera switching for the MuJoCo passive viewer
scenes/g1_brainco/             Built scene XML - see its README.md for the full hand/body model derivation
assets/unitree_g1_brainco/     Robot MJCF + mesh assets (self-contained, no external download needed)
docs/                          Verified reference snippets copied from upstream sources (see Credits)
examples/                      Standalone DDS scripts - state monitor, interpolated whole-body + hand motion
```

## Setup

1. Install `mujoco` and [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python)
   (see that repo's own README for platform-specific dependencies, notably
   `cyclonedds`).
2. Nothing else to build or download - the scene and mesh assets are
   already included.
3. Run the simulator:
   ```
   python3 unitree_mujoco.py
   ```
   A viewer window opens with the robot standing in its default pose.
   Press `C` to cycle to the egocentric head camera, back to free view.
4. In a separate process (same machine, or a different one on the same
   network for the real robot), run a controller against `rt/lowcmd` /
   `rt/lowstate` for the body and `rt/brainco/{left,right}/cmd` / `state`
   for the hands. See `examples/` for minimal working scripts to start
   from, including exactly what changes to point them at real hardware
   instead of sim.

## Credits / attribution

This repository vendors and modifies source from:

- [`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco)
  (Unitree Robotics, BSD-3-Clause) - `body_bridge.py` and
  `unitree_mujoco.py` are adapted from `simulate_python/unitree_sdk2py_bridge.py`
  and `simulate_python/unitree_mujoco.py`, including the `ElasticBand`
  class. The 29-DOF body actuator/sensor block
  (`docs/reference_g1_29dof_body_snippet.xml`) is copied verbatim from
  `unitree_robots/g1/g1_29dof.xml`.
- [`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python)
  (Unitree Robotics, BSD-3-Clause) - DDS channel/publisher/subscriber
  classes, message types (`LowCmd_`/`LowState_`, `MotorCmds_`/
  `MotorStates_`), CRC utility, and threading utilities used throughout.
- [`brainco_hand_service`](https://github.com/unitreerobotics/brainco_hand_service)
  (Unitree Robotics) - no code is vendored from this repo (it's C++,
  serial-side, and doesn't currently list an OSS license); its `README.md`
  and `main.cpp` were read to confirm the real hand's topic names and
  message shapes, which `brainco_sim_bridge.py` reimplements independently
  for simulation.
- [BrainCo Revo2 documentation](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/parameters.html)
  and BrainCo's official Revo2 MuJoCo model (same site, "Download" page) -
  the hand's active/passive joint structure and actuator/coupling values
  were verified directly against BrainCo's own model files. A trimmed
  reference copy is kept in
  `docs/reference_brainco_revo2_right_hand_snippet.xml`, explicitly marked
  as **not** covered by this repository's license, since BrainCo doesn't
  state an explicit OSS license for it - included for verification
  purposes only.

See `LICENSE` for the full BSD-3-Clause text, with the original Unitree
Robotics copyright notice preserved as required by that license.

## Known limitations

- **`_tip` finger joints have no authoritative model.** BrainCo's own
  left-hand reference model doesn't include one at all; their right-hand
  model has one but leaves it fully passive with no actuator or equality
  constraint. This repository leaves them uncontrolled rather than invent
  a coupling ratio - revisit if fingertip contact turns out to matter for
  grasp quality.
- **`tau_est` is not published for hands.** The real `brainco_hand_service`
  derives it from motor current in Amps; this simulation has no current
  model, so it's left at `0.0` rather than fabricate a number in the
  wrong units. Current-threshold stall/grasp detection - a real technique
  with this hand - will not work against this simulation as-is.
- **`rt/arm_sdk` is out of scope** (see Scope above).