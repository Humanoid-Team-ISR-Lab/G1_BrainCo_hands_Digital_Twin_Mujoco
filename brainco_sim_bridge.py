"""
brainco_sim_bridge.py

Sim-side stand-in for the real robot's `brainco_hand.service`. Mirrors its
topics and message shapes exactly, so brainco_hand.py's BrainCoHandController
talks to this bridge and the real hardware identically - only
ChannelFactoryInitialize's domain/interface differ.

Topics   : rt/brainco/{side}/cmd   (MotorCmds_, subscribe)
           rt/brainco/{side}/state (MotorStates_, publish)
Logical DoF order (must match brainco_hand.py):
           [Thumb, Thumb_aux, Index, Middle, Ring, Pinky]
Convention: 0.0 = open, 1.0 = closed (matches the real hand's convention).

Verified against three sources:
  - unitreerobotics/brainco_hand_service, main.cpp - real topic/message
    behavior (q, dq, tau_est all published; commands are position+speed).
  - BrainCo's Revo2 spec (brainco-hz.com/docs/revolimb-hand/en/revo2/
    parameters.html) - 6 active joints, 11 total DOF.
  - BrainCo's own official MuJoCo model for this hand
    (brainco-hz.com/docs/revolimb-hand/en/revo2/download.html ->
    "MuJoCo XML Model", inspected directly: xml_left/brainco-lefthand-v2.xml,
    xml_right/brainco-righthand-v2.xml).

That last one settles what was previously a guess: BrainCo's own model has
exactly ONE <position> actuator per logical DoF - the proximal joint (or
metacarpal, for thumb_aux). The distal joint of each finger is NOT
independently actuated; it's coupled via an MJCF <equality> joint
constraint:

    index:              distal = 1.0   * proximal
    middle/ring/pinky:   distal = 1.155 * proximal
    thumb:               distal = 1.155 * proximal

(all polycoef "0 <ratio> 0 0", i.e. a fixed linear ratio, no offset).
See scenes/g1_brainco/README.md for the exact <equality> block to add to
the scene XML - MuJoCo's own solver handles the distal joint from there,
this bridge never touches it.

The "tip" joints present in this project's MJCF (thumb/index/middle/ring/
pinky _tip) don't fully exist in BrainCo's own model either - the left-hand
model has no tip joint at all (distal is the last segment), and the
right-hand model has one but with NO actuator and NO equality constraint
(fully free). There's no authoritative coupling to copy for these, so this
bridge leaves them uncontrolled, matching BrainCo's own right-hand
precedent, rather than inventing a ratio.

tau_est is intentionally left at 0.0 (see publish_state()) - the real
service derives it from motor current in Amps, which this sim has no
model for.
"""

import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl import unitree_go_msg_dds__MotorCmd_, unitree_go_msg_dds__MotorState_
from unitree_sdk2py.utils.thread import RecurrentThread

import config

THUMB, THUMB_AUX, INDEX, MIDDLE, RING, PINKY = range(6)

# Logical DoF -> the single physical joint it actually drives (the ACTIVE
# joint). Distal joints are handled by an MJCF <equality> constraint, not
# by this bridge - see scenes/g1_brainco/README.md.
ACTIVE_JOINT_STEM = {
    THUMB_AUX: "{side}_thumb_metacarpal",
    THUMB: "{side}_thumb_proximal",
    INDEX: "{side}_index_proximal",
    MIDDLE: "{side}_middle_proximal",
    RING: "{side}_ring_proximal",
    PINKY: "{side}_pinky_proximal",
}


def _find_actuator(mj_model, stem):
    aid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{stem}_joint")
    if aid != -1:
        return aid
    return mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, stem)


class BrainCoSimBridge:
    def __init__(self, mj_model, mj_data, side: str):
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.side = side

        # One actuator id (+ ctrl upper bound) per logical DoF. ctrlrange
        # lower bound is 0.0 == open for every hand joint in this project's
        # MJCF, matching BrainCo's own model, so the logical [0,1] value
        # maps directly onto the fraction of ctrlrange upper bound.
        self.actuator_id = [-1] * config.NUM_HAND_DOF
        self.ctrl_upper = [0.0] * config.NUM_HAND_DOF
        for dof in range(config.NUM_HAND_DOF):
            stem = ACTIVE_JOINT_STEM[dof].format(side=side)
            aid = _find_actuator(mj_model, stem)
            if aid == -1:
                print(f"[BrainCoSimBridge:{side}][WARN] actuator not found for '{stem}'")
                continue
            lo, hi = mj_model.actuator_ctrlrange[aid]
            if lo != 0.0:
                print(
                    f"[BrainCoSimBridge:{side}][WARN] '{stem}' ctrlrange lower "
                    f"bound is {lo}, not 0 - open/closed mapping may be wrong."
                )
            self.actuator_id[dof] = aid
            self.ctrl_upper[dof] = hi

        self.cmd_topic = config.HAND_CMD_TOPIC.format(side=side)
        self.state_topic = config.HAND_STATE_TOPIC.format(side=side)

        self.state = MotorStates_(
            states=[unitree_go_msg_dds__MotorState_() for _ in range(config.NUM_HAND_DOF)]
        )
        self.state_puber = ChannelPublisher(self.state_topic, MotorStates_)
        self.state_puber.Init()
        self.state_thread = RecurrentThread(
            interval=1.0 / config.HAND_STATE_HZ,
            target=self.publish_state,
            name=f"brainco_state_{side}",
        )
        self.state_thread.Start()

        self.cmd_suber = ChannelSubscriber(self.cmd_topic, MotorCmds_)
        self.cmd_suber.Init(self.cmd_handler, 10)

        self._last_logical = [0.0] * config.NUM_HAND_DOF

    def cmd_handler(self, msg: MotorCmds_):
        if self.mj_data is None:
            return
        for dof in range(config.NUM_HAND_DOF):
            if dof >= len(msg.cmds):
                break
            value = float(np.clip(msg.cmds[dof].q, 0.0, 1.0))
            self._last_logical[dof] = value
            aid = self.actuator_id[dof]
            if aid != -1:
                self.mj_data.ctrl[aid] = value * self.ctrl_upper[dof]
            # Distal joint moves automatically via the MJCF <equality>
            # constraint on the active joint above - no ctrl write needed.

    def publish_state(self):
        if self.mj_data is None:
            return
        for dof in range(config.NUM_HAND_DOF):
            aid = self.actuator_id[dof]
            if aid != -1:
                hi = self.ctrl_upper[dof]
                q = self.mj_data.actuator_length[aid]
                dq = self.mj_data.actuator_velocity[aid]
                logical_q = float(np.clip(q / hi, 0.0, 1.0)) if hi != 0 else 0.0
                logical_dq = float(dq / hi) if hi != 0 else 0.0
            else:
                logical_q = self._last_logical[dof]
                logical_dq = 0.0
            self.state.states[dof].q = logical_q
            self.state.states[dof].dq = logical_dq
            # tau_est intentionally left at 0.0 - see module docstring.
        self.state_puber.Write(self.state)
