"""
examples/go_to_zero_and_hands.py

DDS write test: reads the robot's CURRENT state over rt/lowstate /
rt/brainco/*/state, then runs ONE continuous control loop for the whole
script - body and hands are published together, every tick, for as long
as the script runs. Only the TARGET changes over time:

  phase A [0, BODY_MOVE_SECONDS)                body: current -> zero
                                                 hands: hold at current
  phase B [.., +HAND_CLOSE_SECONDS)             body: hold at zero
                                                 hands: current -> closed
  phase C [.., +HAND_HOLD_SECONDS)               body: hold at zero
                                                 hands: hold closed
  phase D [.., +HAND_OPEN_SECONDS)               body: hold at zero
                                                 hands: closed -> open
  phase E forever after (until Ctrl+C)           body: hold at zero
                                                 hands: hold open

Always publish, only change what's being published - never stop mid-
sequence. On real hardware the low-level controller times out and cuts
torque when commands stop arriving; in this sim, body_bridge.py's ctrl
values simply stop being refreshed. Either way, the fix is the same:
keep one loop running nonstop from start to finish.

Standalone on purpose - no import of this repo's config.py, so the exact
same file runs against the real robot by changing only DOMAIN_ID/INTERFACE
below (matching the pattern in joint_controller.py / brainco_pose_test.py).

Sim:  DOMAIN_ID = 1, INTERFACE = "lo"
Real: DOMAIN_ID = 0, INTERFACE = "<your robot NIC, e.g. enp2s0>"

Run unitree_mujoco.py first (in another terminal, same machine), then:
    python3 examples/go_to_zero_and_hands.py
"""

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmd_default
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.utils.crc import CRC

DOMAIN_ID = 1
INTERFACE = "lo"

BODY_CMD_TOPIC = "rt/lowcmd"
BODY_STATE_TOPIC = "rt/lowstate"
NUM_BODY_MOTOR = 29  # legs(12) + waist(3) + arms(14), unitree_hg order

HAND_SIDES = ["left", "right"]
HAND_CMD_TOPIC = "rt/brainco/{side}/cmd"
HAND_STATE_TOPIC = "rt/brainco/{side}/state"
NUM_HAND_DOF = 6  # order: Thumb, Thumb_aux, Index, Middle, Ring, Pinky

CONTROL_HZ = 50
BODY_MOVE_SECONDS = 3.0
HAND_CLOSE_SECONDS = 2.0
HAND_HOLD_SECONDS = 1.0
HAND_OPEN_SECONDS = 2.0

# Phase boundaries, as elapsed seconds since the loop started.
T_BODY_DONE = BODY_MOVE_SECONDS
T_HAND_CLOSE_DONE = T_BODY_DONE + HAND_CLOSE_SECONDS
T_HAND_HOLD_DONE = T_HAND_CLOSE_DONE + HAND_HOLD_SECONDS
T_HAND_OPEN_DONE = T_HAND_HOLD_DONE + HAND_OPEN_SECONDS

# Reasonable default gains for whole-body position control, taken from
# this project's own G1_Sim-Developmnet/config/g1_config.json (already
# tuned for this exact joint order: legs+waist, then arms flat) - not
# re-derived here, just reused. Adjust if your team has since retuned them.
KP_LEGS_WAIST = [150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40, 250, 250, 250]
KD_LEGS_WAIST = [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5]
KP_ARMS = 100.0
KD_ARMS = 2.0
KP = KP_LEGS_WAIST + [KP_ARMS] * 14
KD = KD_LEGS_WAIST + [KD_ARMS] * 14
assert len(KP) == len(KD) == NUM_BODY_MOTOR

# mode is written into every motor_cmd but is NOT interpreted by
# body_bridge.py's sim (it always applies the kp/kd PD formula regardless
# of mode) - set here so this script also behaves sanely on real
# hardware, where mode matters. mode=1 matches the common "enable
# position control" convention for full 29-DOF low-level control; check
# joint_controller.py's own convention before pointing this at a real
# robot if it differs.
BODY_MOTOR_MODE = 1

HAND_SPEED = 1.0  # matches brainco_pose_test.py's convention for the dq/speed field

_latest_body_state = None
_latest_hand_state = {}


def _body_state_handler(msg):
    global _latest_body_state
    _latest_body_state = msg


def _make_hand_state_handler(side):
    def handler(msg):
        _latest_hand_state[side] = msg

    return handler


def _wait_for(predicate, what, timeout=5.0):
    start = time.time()
    while predicate() is None:
        if time.time() - start > timeout:
            raise TimeoutError(f"No {what} received after {timeout}s - is unitree_mujoco.py running?")
        time.sleep(0.05)


def _interp(q_start, q_end, alpha):
    alpha = max(0.0, min(1.0, alpha))
    return [a + (b - a) * alpha for a, b in zip(q_start, q_end)]


def body_target_at(t, q0, q_zero):
    """Current -> zero over BODY_MOVE_SECONDS, then hold at zero forever."""
    if t < T_BODY_DONE:
        return _interp(q0, q_zero, t / BODY_MOVE_SECONDS)
    return q_zero


def hand_target_at(t, q0, q_closed, q_open):
    """Hold current -> close -> hold closed -> open -> hold open."""
    if t < T_BODY_DONE:
        return q0
    if t < T_HAND_CLOSE_DONE:
        return _interp(q0, q_closed, (t - T_BODY_DONE) / HAND_CLOSE_SECONDS)
    if t < T_HAND_HOLD_DONE:
        return q_closed
    if t < T_HAND_OPEN_DONE:
        return _interp(q_closed, q_open, (t - T_HAND_HOLD_DONE) / HAND_OPEN_SECONDS)
    return q_open


def make_body_cmd(crc, q_cmd):
    cmd = LowCmd_default()
    for i in range(NUM_BODY_MOTOR):
        cmd.motor_cmd[i].mode = BODY_MOTOR_MODE
        cmd.motor_cmd[i].q = q_cmd[i]
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = KP[i]
        cmd.motor_cmd[i].kd = KD[i]
    cmd.crc = crc.Crc(cmd)
    return cmd


def make_hand_cmd(q_cmd):
    cmd = MotorCmds_(cmds=[unitree_go_msg_dds__MotorCmd_() for _ in range(NUM_HAND_DOF)])
    for i in range(NUM_HAND_DOF):
        cmd.cmds[i].q = q_cmd[i]
        cmd.cmds[i].dq = HAND_SPEED
    return cmd


def main():
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    body_sub = ChannelSubscriber(BODY_STATE_TOPIC, LowState_)
    body_sub.Init(_body_state_handler, 10)
    body_pub = ChannelPublisher(BODY_CMD_TOPIC, LowCmd_)
    body_pub.Init()

    hand_pubs = {}
    for side in HAND_SIDES:
        state_sub = ChannelSubscriber(HAND_STATE_TOPIC.format(side=side), MotorStates_)
        state_sub.Init(_make_hand_state_handler(side), 10)
        cmd_pub = ChannelPublisher(HAND_CMD_TOPIC.format(side=side), MotorCmds_)
        cmd_pub.Init()
        hand_pubs[side] = cmd_pub

    print("Waiting for current state over DDS...")
    _wait_for(lambda: _latest_body_state, BODY_STATE_TOPIC)
    for side in HAND_SIDES:
        _wait_for(lambda: _latest_hand_state.get(side), HAND_STATE_TOPIC.format(side=side))

    body_q0 = [_latest_body_state.motor_state[i].q for i in range(NUM_BODY_MOTOR)]
    hand_q0 = {
        side: [_latest_hand_state[side].states[i].q for i in range(NUM_HAND_DOF)]
        for side in HAND_SIDES
    }
    print(f"Current body q: {[round(v, 3) for v in body_q0]}")
    print(f"Current hand q: {{ {', '.join(f'{s}: {[round(v, 3) for v in hand_q0[s]]}' for s in HAND_SIDES)} }}")

    body_zero = [0.0] * NUM_BODY_MOTOR
    hand_closed = [1.0] * NUM_HAND_DOF
    hand_open = [0.0] * NUM_HAND_DOF

    crc = CRC()
    dt = 1.0 / CONTROL_HZ
    start = time.time()
    last_phase = None

    print(f"\nRunning continuously at {CONTROL_HZ} Hz - body to zero ({BODY_MOVE_SECONDS}s), "
          f"then hands close/hold/open. Holds final pose after that. Ctrl+C to stop.\n")

    try:
        while True:
            step_start = time.perf_counter()
            t = time.time() - start

            q_body = body_target_at(t, body_q0, body_zero)
            body_pub.Write(make_body_cmd(crc, q_body))
            for side in HAND_SIDES:
                q_hand = hand_target_at(t, hand_q0[side], hand_closed, hand_open)
                hand_pubs[side].Write(make_hand_cmd(q_hand))

            phase = (
                "body -> zero" if t < T_BODY_DONE else
                "hands -> closed" if t < T_HAND_CLOSE_DONE else
                "hands holding closed" if t < T_HAND_HOLD_DONE else
                "hands -> open" if t < T_HAND_OPEN_DONE else
                "holding (body zero, hands open)"
            )
            if phase != last_phase:
                print(f"[t={t:5.1f}s] {phase}")
                last_phase = phase

            elapsed = time.perf_counter() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\nstopped (this process is no longer publishing - the sim/robot "
              "will only keep applying whatever target it last received).")


if __name__ == "__main__":
    main()