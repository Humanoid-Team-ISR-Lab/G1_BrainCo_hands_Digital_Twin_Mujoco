"""
Configuration for the G1 + BrainCo hands digital twin.

Scope (see README.md "Scope" section for the full rationale):
  - Body: G1 29-DOF, low-level control only (rt/lowcmd / rt/lowstate,
    unitree_hg messages). rt/arm_sdk is NOT simulated here.
  - Hands: BrainCo, position-only commands (0.0=open .. 1.0=closed),
    position-only state readback. No dq/tau feedback, matching what the
    real robot's brainco_hand.service actually exposes.
"""

# --- Body ---
ROBOT_SCENE = "scenes/g1_brainco/g1_brainco_body.xml"
NUM_BODY_MOTOR = 29  # fixed: legs(12) + waist(3) + arms(14), unitree_hg order

# --- Hands ---
# Logical DoF order per hand, matching brainco_hand.py exactly:
#   [Thumb, Thumb_aux, Index, Middle, Ring, Pinky]
HAND_SIDES = ["left", "right"]
NUM_HAND_DOF = 6
HAND_CMD_TOPIC = "rt/brainco/{side}/cmd"
HAND_STATE_TOPIC = "rt/brainco/{side}/state"
HAND_STATE_HZ = 50  # publish rate for MotorStates_ readback

# --- DDS ---
DOMAIN_ID = 1        # sim; use 0 on the real robot
INTERFACE = "lo"     # sim; use the robot NIC name (e.g. "enp2s0") on hardware

# --- Sim loop ---
SIMULATE_DT = 0.002
VIEWER_DT = 0.02
PRINT_SCENE_INFORMATION = True
ENABLE_ELASTIC_BAND = True # virtual spring to lift the robot; keys 7/8/9 in-viewer
