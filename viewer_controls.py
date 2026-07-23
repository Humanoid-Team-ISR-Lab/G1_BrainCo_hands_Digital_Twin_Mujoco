"""
viewer_controls.py

Toggle the MuJoCo passive viewer between the free/orbit camera and a
fixed, robot-mounted camera - press C to cycle. Viewer-only concern, no
DDS involved, kept separate from body_bridge.py's ElasticBand for that
reason.

The scene already defines head-mounted cameras on `torso_link` (see
assets/unitree_g1_brainco/mjcf/g1_brainco.xml): `egocentric` (RGB),
`egocentric_depth`, `egocentric_wide`, and `pov`. CAMERA_NAMES below
defaults to just `egocentric` - the one that matches a D435/D455-style RGB
feed, which is what a vision-language pipeline consuming egocentric images
would want. Add more names to the list (e.g. "pov") if you want C to cycle
through several; order in the list is cycle order, then back to free.
"""

import mujoco

CAMERA_NAMES = ["egocentric"]


class CameraSwitcher:
    def __init__(self, mj_model, camera_names=CAMERA_NAMES):
        self.mj_model = mj_model
        self.cameras = []
        for name in camera_names:
            cid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            if cid == -1:
                print(f"[CameraSwitcher][WARN] camera '{name}' not found in scene - skipping.")
            else:
                self.cameras.append((name, cid))
        self.index = -1  # -1 == free/orbit camera
        self.viewer = None  # set via attach() once launch_passive() returns

    def attach(self, viewer):
        """Must be called with the viewer object after launch_passive()
        returns - the callback needs it to actually change the active
        camera, but it doesn't exist yet at the point key_callback is
        registered."""
        self.viewer = viewer

    def cycle(self):
        if self.viewer is None or not self.cameras:
            return
        self.index += 1
        if self.index >= len(self.cameras):
            self.index = -1
        if self.index == -1:
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            print("[CameraSwitcher] free camera")
        else:
            name, cid = self.cameras[self.index]
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.viewer.cam.fixedcamid = cid
            print(f"[CameraSwitcher] camera: {name}")

    def key_callback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_C:
            self.cycle()
