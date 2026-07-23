"""
examples/read_states.py

Read-only DDS smoke test: subscribes to rt/lowstate (body) and
rt/brainco/{left,right}/state (hands), prints the latest values to the
terminal at a fixed rate. Publishes nothing.

Standalone on purpose - no import of this repo's config.py, so the exact
same file runs against the real robot by changing only DOMAIN_ID/INTERFACE
below (matching the pattern in joint_controller.py / brainco_pose_test.py).

Sim:  DOMAIN_ID = 1, INTERFACE = "lo"
Real: DOMAIN_ID = 0, INTERFACE = "<your robot NIC, e.g. enp2s0>"

Run unitree_mujoco.py first (in another terminal, same machine), then:
    python3 examples/read_states.py
"""

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

DOMAIN_ID = 1
INTERFACE = "lo"

BODY_STATE_TOPIC = "rt/lowstate"
NUM_BODY_MOTOR = 29  # legs(12) + waist(3) + arms(14), unitree_hg order

HAND_SIDES = ["left", "right"]
HAND_STATE_TOPIC = "rt/brainco/{side}/state"
NUM_HAND_DOF = 6  # order: Thumb, Thumb_aux, Index, Middle, Ring, Pinky

PRINT_HZ = 2.0

latest = {"body": None}
for side in HAND_SIDES:
    latest[side] = None


def body_handler(msg):
    latest["body"] = msg


def make_hand_handler(side):
    def handler(msg):
        latest[side] = msg

    return handler


def main():
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

    body_sub = ChannelSubscriber(BODY_STATE_TOPIC, LowState_)
    body_sub.Init(body_handler, 10)

    hand_subs = []
    for side in HAND_SIDES:
        sub = ChannelSubscriber(HAND_STATE_TOPIC.format(side=side), MotorStates_)
        sub.Init(make_hand_handler(side), 10)
        hand_subs.append(sub)  # keep references alive

    print(f"Listening on {BODY_STATE_TOPIC} + rt/brainco/*/state "
          f"(domain={DOMAIN_ID}, interface={INTERFACE!r}). Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(1.0 / PRINT_HZ)

            body = latest["body"]
            if body is None:
                print("[body] no message yet - is unitree_mujoco.py running?")
            else:
                q = [round(body.motor_state[i].q, 3) for i in range(NUM_BODY_MOTOR)]
                quat = [round(v, 3) for v in body.imu_state.quaternion]
                print(f"[body] q ({NUM_BODY_MOTOR} joints) = {q}")
                print(f"[body] imu quat = {quat}")

            for side in HAND_SIDES:
                msg = latest[side]
                if msg is None:
                    print(f"[hand:{side}] no message yet")
                else:
                    q = [round(msg.states[i].q, 3) for i in range(NUM_HAND_DOF)]
                    dq = [round(msg.states[i].dq, 3) for i in range(NUM_HAND_DOF)]
                    print(f"[hand:{side}] q  = {q}   (order: Thumb, Thumb_aux, Index, Middle, Ring, Pinky)")
                    print(f"[hand:{side}] dq = {dq}")

            print("-" * 70)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()