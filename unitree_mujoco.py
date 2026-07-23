"""
Entry point: loads the G1 + BrainCo hands scene, steps physics, and runs
the body + hand DDS bridges. Low-level control only (rt/lowcmd /
rt/brainco/*) - rt/arm_sdk is intentionally not simulated (see README.md).

Adapted from unitree_mujoco's simulate_python/unitree_mujoco.py
(BSD-3-Clause, Unitree Robotics) - see LICENSE.
"""

import time
import threading
from threading import Thread

import mujoco
import mujoco.viewer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config
from body_bridge import BodyBridge, ElasticBand
from brainco_sim_bridge import BrainCoSimBridge
from viewer_controls import CameraSwitcher

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
mj_model.opt.timestep = config.SIMULATE_DT

if mj_model.nu < config.NUM_BODY_MOTOR:
    raise RuntimeError(
        f"Scene has {mj_model.nu} actuators, fewer than "
        f"config.NUM_BODY_MOTOR={config.NUM_BODY_MOTOR}. Check "
        f"scenes/g1_brainco/g1_brainco_body.xml against scenes/g1_brainco/README.md."
    )

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    band_attached_link = mj_model.body("torso_link").id
else:
    elastic_band = None

camera_switcher = CameraSwitcher(mj_model)


def key_callback(key):
    """Single dispatcher for every viewer key binding. Only ONE
    key_callback can be registered with launch_passive(), so every
    feature that wants keyboard input (elastic band, camera switch, ...)
    goes through here rather than each grabbing its own launch_passive()
    call - a second concurrent passive-viewer launch on the same mj_data
    will segfault."""
    if elastic_band is not None:
        elastic_band.MujuocoKeyCallback(key)
    camera_switcher.key_callback(key)


# Exactly ONE launch_passive() call - see key_callback's docstring above
# for why. If you're merging edits back in by hand, keep it that way.
viewer = mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=key_callback)
camera_switcher.attach(viewer)
time.sleep(0.2)


def simulation_thread():
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)

    body = BodyBridge(mj_model, mj_data)
    hands = [BrainCoSimBridge(mj_model, mj_data, side) for side in config.HAND_SIDES]

    if config.PRINT_SCENE_INFORMATION:
        body.print_scene_information()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()
        if config.ENABLE_ELASTIC_BAND and elastic_band.enable:
            mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                mj_data.qpos[:3], mj_data.qvel[:3]
            )
        mujoco.mj_step(mj_model, mj_data)
        locker.release()

        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def physics_viewer_thread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    print("Viewer keys: C = cycle camera (free <-> egocentric)"
          + (", 7/8 = elastic band length, 9 = toggle elastic band" if config.ENABLE_ELASTIC_BAND else ""))
    Thread(target=physics_viewer_thread).start()
    Thread(target=simulation_thread).start()