# The `g1_brainco_body.xml` scene

This document explains how the included scene (`g1_brainco_body.xml`,
`assets/unitree_g1_brainco/mjcf/g1_brainco.xml`, and their mesh assets)
is put together, and the reasoning behind each design choice - useful if
you need to regenerate this scene from a newer version of the source
URDF, or just want to understand the underlying model before trusting it.

Verified to compile and step cleanly in MuJoCo directly: 29 body `<motor>`
actuators, 12 hand `<position>` actuators, 10 `<equality>` constraints,
92 sensors. The hand tendon coupling was checked empirically too - driving
the index finger's proximal actuator produced a distal-joint response at
a 0.999 ratio, matching the intended 1.0 `polycoef` below.

## 1. Body actuators are torque (`<motor>`), not position servos

`body_bridge.py` computes PD torque in Python from the incoming
`rt/lowcmd` message and writes it straight into MuJoCo's `ctrl` array:
`tau + kp*(q_err) + kd*(dq_err)`. That formula only makes sense for
`<motor>` (torque) actuators - a `<position>` (implicit position servo)
actuator interprets the same `ctrl` value as a target angle, not a torque,
so the two are not interchangeable.

The 29 body actuators are therefore `<motor>` type, verified identical to
Unitree's own reference implementation for the G1 (`docs/reference_g1_29dof_body_snippet.xml`,
copied from `unitree_mujoco/unitree_robots/g1/g1_29dof.xml`) - same joint
names, same order: legs (12) -> waist (3) -> left arm (7) -> right arm
(7). The hand actuators stay `<position>` type (see section 3) since the
real hand takes direct position/speed commands and does its own
closed-loop tracking in firmware - no PD computation needed on this side.

## 2. Actuator ordering: body first, hands after

`body_bridge.py` hard-codes `NUM_BODY_MOTOR=29` and only ever touches
`ctrl[0:29]` / the first 87 sensor entries - so the 29 body actuators are
the first 29 entries in the `<actuator>` block, contiguous, in the exact
order above. All hand actuators come after that. `brainco_sim_bridge.py`
looks its actuators up by name rather than index, so their exact position
past entry 29 doesn't matter, only that they aren't interleaved among the
body ones.

## 3. Hand actuators and the tendon-coupling model

BrainCo's Revo2 hand has 6 actively-driven joints and 11 total degrees of
freedom - verified directly against BrainCo's own official MuJoCo model
for this hand (`xml_left/brainco-lefthand-v2.xml`,
`xml_right/brainco-righthand-v2.xml`, downloadable from
https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/download.html,
"MuJoCo XML Model"). That model defines exactly 6 `<position>` actuators
per hand - one per proximal joint (or metacarpal, for thumb abduction/
adduction) - and drives every other joint passively:

```xml
<position name="{side}_thumb_metacarpal_joint" joint="{side}_thumb_metacarpal_joint" kp="30" kv="10" ctrlrange="0 1.51"/>
<position name="{side}_thumb_proximal_joint"   joint="{side}_thumb_proximal_joint"   kp="30" kv="10" ctrlrange="0 1.04"/>
<position name="{side}_index_proximal_joint"   joint="{side}_index_proximal_joint"   kp="30" kv="10" ctrlrange="0 1.46"/>
<position name="{side}_middle_proximal_joint"  joint="{side}_middle_proximal_joint"  kp="30" kv="10" ctrlrange="0 1.46"/>
<position name="{side}_ring_proximal_joint"    joint="{side}_ring_proximal_joint"    kp="30" kv="10" ctrlrange="0 1.46"/>
<position name="{side}_pinky_proximal_joint"   joint="{side}_pinky_proximal_joint"   kp="30" kv="10" ctrlrange="0 1.46"/>
```

The distal joint of each finger is not independently actuated - it's
driven by an MJCF `<equality>` joint constraint instead, at a fixed
linear ratio to its proximal joint (index uses a different ratio than
the other four):

```xml
<equality>
  <joint joint1="{side}_index_proximal_joint"  joint2="{side}_index_distal_joint"  polycoef="0 1.    0 0"/>
  <joint joint1="{side}_middle_proximal_joint" joint2="{side}_middle_distal_joint" polycoef="0 1.155 0 0"/>
  <joint joint1="{side}_ring_proximal_joint"   joint2="{side}_ring_distal_joint"   polycoef="0 1.155 0 0"/>
  <joint joint1="{side}_pinky_proximal_joint"  joint2="{side}_pinky_distal_joint"  polycoef="0 1.155 0 0"/>
  <joint joint1="{side}_thumb_proximal_joint"  joint2="{side}_thumb_distal_joint"  polycoef="0 1.155 0 0"/>
</equality>
```

`{side}` is `left`/`right`. The gains (`kp=30, kv=10`) are BrainCo's own
tuned values, adopted directly rather than re-derived - noticeably
stiffer than a naive default. The `ctrlrange` upper bounds match this
hand's real joint limits closely (e.g. `thumb_metacarpal` here is `0
1.5184` vs BrainCo's own `0 1.51`), which is expected since both trace
back to the same physical hardware limits, not a coincidence to double
check.

**The fingertip (`_tip`) joints have no authoritative model to copy.**
BrainCo's own left-hand reference file doesn't include a tip joint at all
(the distal segment is the last one in the chain); their right-hand file
does have one, but it carries no actuator and no equality constraint -
fully passive and unconstrained. There is no ratio to adopt here, so
these joints are left uncontrolled in this scene rather than guessing one
- revisit if fingertip contact turns out to matter for grasp fidelity.

No MuJoCo `<sensor>` entries are needed for the hand joints -
`brainco_sim_bridge.py` reads position and velocity back directly via
`mj_data.actuator_length` / `actuator_velocity` on the active actuator,
which mirrors what the real hand's own state feedback provides (position
and speed; no torque/current-equivalent - see the main README's Known
Limitations section).

## 4. Body sensors and the `imu` site

The body `<sensor>` block - `jointpos`/`jointvel`/`jointactuatorfrc` for
all 29 motors in the same order as their actuators, then
`imu_quat`/`imu_gyro`/`imu_acc`/`frame_pos`/`frame_vel` tied to a site
named `imu` - is copied verbatim from Unitree's own G1 reference
(`docs/reference_g1_29dof_body_snippet.xml`). `body_bridge.py` needs all
of it to publish a complete `LowState` message (joint state plus IMU
orientation/angular velocity/acceleration). The `imu` site itself sits on
the `pelvis` body at its local origin, matching Unitree's own placement.

## 5. Joint naming consistency

One joint - the right thumb's fingertip - was originally named
`right_thumb_tip` (no `_joint` suffix) while every other joint in this
scene, including its left-hand counterpart `left_thumb_tip_joint`, follows
a consistent `..._joint` naming pattern. It's renamed to
`right_thumb_tip_joint` here purely for internal consistency with the
rest of the file - not because it matches any external convention.
(BrainCo's own official model doesn't resolve this either: their
right-hand file uses the same un-suffixed form, and their left-hand file
doesn't have a matching joint at all, so there's no consistent upstream
naming to defer to on this specific joint.) `brainco_sim_bridge.py`'s
actuator lookup tries both the suffixed and un-suffixed forms regardless,
for robustness against the same pattern recurring elsewhere.

## 6. What this scene does *not* set

Physics tuning parameters that a walking or RL policy would typically
depend on - per-joint damping, armature, friction, or a non-default
solver timestep - are **not** baked into this XML. Those values, if you
have them from prior work with this robot, would need to be added
directly as XML attributes (or a `<default>` class) if a policy trained
under different assumptions is brought into this repository later.
`unitree_mujoco.py` only sets the timestep, from `config.SIMULATE_DT`.