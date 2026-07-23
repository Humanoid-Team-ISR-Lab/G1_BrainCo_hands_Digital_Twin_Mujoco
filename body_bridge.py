"""
body_bridge.py

G1 body bridge: rt/lowcmd (unitree_hg LowCmd_) -> MuJoCo ctrl,
MuJoCo sensordata -> rt/lowstate (unitree_hg LowState_).

Adapted from unitree_mujoco's simulate_python/unitree_sdk2py_bridge.py
(BSD-3-Clause, Unitree Robotics) - see LICENSE.

Key difference from upstream: upstream sets `self.num_motor = mj_model.nu`
and indexes ctrl[0:num_motor] positionally. That is unsafe here because our
scene has 29 body actuators *plus* BrainCo hand actuators appended after
them (see scenes/g1_brainco/README.md) - mj_model.nu is > 29. This bridge
hard-codes NUM_BODY_MOTOR=29 from config.py and only ever touches
ctrl[0:29] / sensordata for the body. Hand actuators are owned entirely by
brainco_sim_bridge.py.
"""

import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
from unitree_sdk2py.utils.thread import RecurrentThread

import config

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

MOTOR_SENSOR_STRIDE = 3  # jointpos, jointvel, jointactuatorfrc per body motor


class ElasticBand:
    """Virtual spring band for lifting the robot during standup/testing.
    Verbatim from unitree_mujoco's simulate_python/unitree_sdk2py_bridge.py
    (BSD-3-Clause, Unitree Robotics) - see LICENSE. Toggle with config.py's
    ENABLE_ELASTIC_BAND; in-viewer keys 7/8 shrink/extend it, 9 toggles it
    on/off.
    """

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          x: current position
          dx: current velocity
        """
        delta_x = self.point - x
        distance = np.linalg.norm(delta_x)
        direction = delta_x / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable


class BodyBridge:
    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = config.NUM_BODY_MOTOR
        self.dim_motor_sensor = MOTOR_SENSOR_STRIDE * self.num_motor
        self.dt = self.mj_model.opt.timestep

        # Confirm the scene actually has the sensors this bridge assumes,
        # instead of silently publishing zeros (see scenes/g1_brainco/README.md).
        self.have_frame_sensor = False
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if name == "frame_pos":
                self.have_frame_sensor = True
        if not self.have_frame_sensor:
            print(
                "[BodyBridge][WARN] 'frame_pos' sensor not found - IMU/pose "
                "fields of LowState will not be published. Check that the "
                "body scene includes the imu site + sensor block "
                "(see scenes/g1_brainco/README.md)."
            )

        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.low_state_thread = RecurrentThread(
            interval=self.dt, target=self.publish_low_state, name="body_lowstate"
        )
        self.low_state_thread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.low_cmd_handler, 10)

    def low_cmd_handler(self, msg: LowCmd_):
        if self.mj_data is None:
            return
        for i in range(self.num_motor):
            self.mj_data.ctrl[i] = (
                msg.motor_cmd[i].tau
                + msg.motor_cmd[i].kp * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
                + msg.motor_cmd[i].kd
                * (msg.motor_cmd[i].dq - self.mj_data.sensordata[i + self.num_motor])
            )

    def publish_low_state(self):
        if self.mj_data is None:
            return

        for i in range(self.num_motor):
            self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
            self.low_state.motor_state[i].dq = self.mj_data.sensordata[i + self.num_motor]
            self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[
                i + 2 * self.num_motor
            ]

        if self.have_frame_sensor:
            base = self.dim_motor_sensor
            self.low_state.imu_state.quaternion[:] = self.mj_data.sensordata[base : base + 4]
            self.low_state.imu_state.gyroscope[:] = self.mj_data.sensordata[base + 4 : base + 7]
            self.low_state.imu_state.accelerometer[:] = self.mj_data.sensordata[
                base + 7 : base + 10
            ]

        self.low_state_puber.Write(self.low_state)

    def print_scene_information(self):
        print("<<------------- Body joints (first 29 actuators expected) ------------->>")
        for i in range(min(self.mj_model.nu, self.num_motor + 5)):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            marker = "" if i < self.num_motor else "  <- should be a HAND actuator, not body"
            print(f"  actuator_index: {i}, name: {name}{marker}")
