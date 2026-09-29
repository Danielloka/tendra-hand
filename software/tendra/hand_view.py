"""A light 3D view of the hand model, drawn into an image (for OpenCV windows).

MuJoCo's interactive viewer redraws as fast as it can. On a laptop whose CPU and integrated GPU
share one power budget, that slows everything else down (measured on the i3-1125G4: hand
tracking fell from 28 to 7 fps). `HandView` draws only when asked, at a size and rate you choose,
with MuJoCo's offscreen renderer.

    view = HandView(model)
    image = view.render(data)        # BGR image, mirrored like a selfie camera
    cv2.setMouseCallback(window, lambda *a: view.mouse(*a[:4]))   # drag: rotate, wheel: zoom

    view = HandView(scene.model, camera="view")   # a fixed camera of the model (no mouse)
"""

import cv2
import mujoco
import numpy as np

HIDDEN_PREFIXES = ("forearm", "servo", "spool")  # not part of the hand itself


class HandView:
    def __init__(self, model: mujoco.MjModel, width: int = 480, height: int = 480,
                 tendons: bool = False, mirror: bool = True,
                 camera: str | None = None):  # fmt: skip
        """`camera`: name of a model camera to draw from (fixed, mouse disabled); None = a free
        camera aimed at the hand, which the mouse can turn and zoom."""
        self.model = model
        self.width, self.height = width, height
        self.mirror = mirror
        self.renderer = mujoco.Renderer(model, height, width)
        self.opt = mujoco.MjvOption()
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = tendons
        self.cam = mujoco.MjvCamera()
        self.camera = camera
        if camera is None:
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._frame_hand(model)
        else:
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.cam.fixedcamid = model.camera(camera).id  # KeyError if there is no such camera
        self._drag: tuple[int, int, float, float] | None = None

    def _frame_hand(self, model: mujoco.MjModel) -> None:
        """Aim at the hand (not the forearm and servos), seen from the palm side."""
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        keep = [
            g
            for g in range(model.ngeom)
            if not model.body(model.geom_bodyid[g]).name.startswith(HIDDEN_PREFIXES)
            and model.body(model.geom_bodyid[g]).name != "world"
        ]
        pts = data.geom_xpos[keep] if keep else data.xpos[1:]
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        self.cam.lookat[:] = (lo + hi) / 2 + np.array([0, 0, 0.02])
        self.cam.distance = 1.6 * float(np.max(hi - lo)) + 0.05
        self.cam.azimuth = 90.0  # camera on the palm side (-Y), looking toward +Y
        self.cam.elevation = -10.0

    def render(self, data: mujoco.MjData) -> np.ndarray:
        self.renderer.update_scene(data, self.cam, self.opt)
        rgb = self.renderer.render()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return cv2.flip(bgr, 1) if self.mirror else bgr

    def mouse(self, event: int, x: int, y: int, flags: int) -> None:
        """OpenCV mouse event in view coordinates: left-drag rotates, wheel zooms."""
        if self.camera is not None:
            return  # fixed model camera
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drag = (x, y, self.cam.azimuth, self.cam.elevation)
        elif event == cv2.EVENT_LBUTTONUP:
            self._drag = None
        elif event == cv2.EVENT_MOUSEMOVE and self._drag and flags & cv2.EVENT_FLAG_LBUTTON:
            x0, y0, az, el = self._drag
            sign = -1 if self.mirror else 1
            self.cam.azimuth = az - sign * 0.4 * (x - x0)
            self.cam.elevation = float(np.clip(el - 0.4 * (y - y0), -89, 89))
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.cam.distance *= 0.9 if flags > 0 else 1.1

    def close(self) -> None:
        self.renderer.close()
