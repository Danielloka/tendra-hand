"""Wrist pose from the webcam: where the operator's hand is and how it is turned.

MediaPipe gives two sets of landmarks per frame (see `hand_tracking.TrackedHand`):

- `world`: metric 3D points around the hand's centre, in axes aligned with the camera (x right
  in the unmirrored image, y down, z away from the camera). They carry the hand's *rotation*
  relative to the camera, but not its position.
- `image`: the same points in pixels.

So only the hand's translation T (camera frame) is unknown. A pinhole camera projects a point
P = world_i + T to

    u = cx + f Px / Pz,    v = cy + f Py / Pz,    f = (width / 2) / tan(hfov / 2)

and T is found by nonlinear least squares (Gauss-Newton, 3 unknowns, 42 residuals), so that the
projected world landmarks land on the image landmarks. It starts from a weak-perspective guess
(depth from the ratio of the hand's 3D spread to its 2D spread). This uses the whole hand shape,
so a tilted palm doesn't read as "farther away" the way hand size in pixels would.

Output frame (the "view frame", shared with the simulation scene): x right on the mirrored
screen, y up on screen, z toward the viewer (the camera); metres; origin = the palm centre at the
image centre at `nominal_distance` from the camera. The screen is a mirror, so for a camera point
view = (-x, -y, -(z - nominal_distance)), and for a direction view = -cam: a reflection. A real
LEFT hand therefore appears with right-hand geometry and matches the (right) robot hand exactly;
a real right hand drives it as its mirror image, like the finger retargeting does.

    tracker = WristTracker()
    pose = tracker.update(hand, t=time.monotonic(), chirality=retargeter.chirality)
    scene.set_wrist_from_view(pose.pos, pose.rot)
"""

import math
from dataclasses import dataclass, field

import numpy as np

from tendra.hand_tracking import TrackedHand
from tendra.retarget import PALM_POINTS, OneEuroFilter, Retargeter, bend_cue, palm_frame

# Least-squares weights per landmark: the palm (wrist, thumb base, knuckles) is nearly rigid;
# finger landmarks move with the fingers and MediaPipe estimates their depth less well.
LANDMARK_WEIGHTS = np.full(21, 0.3)
LANDMARK_WEIGHTS[[*PALM_POINTS, 1]] = 1.0

MIN_DEPTH, MAX_DEPTH = 0.15, 1.5  # m: sane camera -> palm distances
MAX_RMS_PX = 25.0  # a solve that fits the image worse than this is rejected


def focal_length(width: float, hfov_deg: float) -> float:
    """Pinhole focal length in pixels from the horizontal field of view."""
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


def initial_translation(
    world: np.ndarray, image: np.ndarray, focal: float, center: np.ndarray,
    weights: np.ndarray = LANDMARK_WEIGHTS,
) -> np.ndarray:  # fmt: skip
    """Weak-perspective estimate of T: every point is taken to be at the centroid's depth.

    Then image offsets are world offsets scaled by f / Z, so Z = f * (3D spread / 2D spread),
    using the x/y spread of the camera-aligned world points (which already includes the
    foreshortening of a tilted palm).
    """
    w = weights / weights.sum()
    wc, ic = w @ world, w @ image
    spread3 = math.sqrt(w @ ((world[:, :2] - wc[:2]) ** 2).sum(axis=1))
    spread2 = math.sqrt(w @ ((image - ic) ** 2).sum(axis=1))
    z = focal * spread3 / max(spread2, 1e-6)
    xy = (ic - center) * z / focal - wc[:2]
    return np.array([xy[0], xy[1], z - wc[2]])


def solve_translation(
    world: np.ndarray, image: np.ndarray, focal: float, center: np.ndarray,
    weights: np.ndarray = LANDMARK_WEIGHTS, t0: np.ndarray | None = None, iterations: int = 10,
) -> tuple[np.ndarray, float] | None:  # fmt: skip
    """Hand translation T (camera frame, m) and the weighted RMS reprojection error (px).

    Gauss-Newton on r_i = project(world_i + T) - image_i with the analytic Jacobian
    du/dT = f [1/Pz, 0, -Px/Pz^2], dv/dT = f [0, 1/Pz, -Py/Pz^2]. Returns None if the solve
    leaves the camera's front (a point behind or at the lens) or doesn't give finite numbers.
    """
    T = initial_translation(world, image, focal, center, weights) if t0 is None else t0.copy()
    tu, tv = image[:, 0] - center[0], image[:, 1] - center[1]
    w = weights
    done = False
    for k in range(iterations + 1):
        P = world + T
        z = P[:, 2]
        if not (np.all(np.isfinite(P)) and np.all(z > 0.02)):
            return None
        a = focal / z  # du/dTx = dv/dTy
        ru, rv = P[:, 0] * a - tu, P[:, 1] * a - tv
        if done or k == iterations:
            break  # this pass only computed the final residuals
        bu, bv = -P[:, 0] * a / z, -P[:, 1] * a / z  # du/dTz, dv/dTz
        # Normal equations J^T W J step = -J^T W r, written out (each row of J has only two
        # non-zeros), which is much faster than building the 42 x 3 J.
        waa = w @ (a * a)
        A = np.array([
            [waa, 0.0, w @ (a * bu)],
            [0.0, waa, w @ (a * bv)],
            [0.0, 0.0, w @ (bu * bu + bv * bv)],
        ])  # fmt: skip
        A[2, 0], A[2, 1] = A[0, 2], A[1, 2]
        g = np.array([w @ (a * ru), w @ (a * rv), w @ (bu * ru + bv * rv)])
        try:
            step = -np.linalg.solve(A, g)
        except np.linalg.LinAlgError:  # degenerate landmarks (e.g. all in one point)
            return None
        # Don't let one step change the depth by more than half (keeps it in front of the lens).
        limit = 0.5 * abs(T[2])
        if abs(step[2]) > limit:
            step *= limit / abs(step[2])
        T = T + step
        done = np.abs(step).max() < 1e-6  # 1 um: converged, one more pass for the residuals
    rms = math.sqrt(float(w @ (ru * ru + rv * rv)) / float(2 * w.sum()))
    return T, rms


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method, stable)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def hand_axes_view(world: np.ndarray, chirality: float) -> np.ndarray:
    """Robot hand axes in the view frame (columns X, Y, Z), from camera-frame world landmarks.

    Z_hand = fingers (palm `up`), Y_hand = back of the hand (-palm normal), X_hand = Y x Z.
    Directions map to the view frame by -1 (the mirror); the palm normal still points out of the
    palm after it, because it is mapped as a vector like every other direction.
    """
    frame = palm_frame(world, chirality)
    z = -frame.up
    y = frame.palm  # -(view palm normal) = -(-palm) = palm
    y = y - (y @ z) * z  # Gram-Schmidt (they are already orthogonal, up to rounding)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


@dataclass
class WristPose:
    pos: np.ndarray  # (3,) view frame, m
    rot: np.ndarray  # (3, 3) columns = robot hand axes (X thumb, Y back, Z fingers) in the view
    distance: float  # camera -> palm centre, m (for display/debug)


@dataclass
class _Raw:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    distance: float = 0.0


class WristTracker:
    """Operator wrist pose (view frame), smoothed, with a clutch.

    Clutch: while disengaged, `update` keeps returning the last pose. On re-engaging, position
    continues from there (an offset between the raw and the output position is stored, like
    lifting a mouse off its pad); orientation is always absolute. `gain` scales position motion.
    """

    CUE_THRESHOLD = Retargeter.CUE_THRESHOLD

    def __init__(
        self, image_size=(640, 480), hfov_deg: float = 62.0, nominal_distance: float = 0.5,
        gain: float = 1.0, smoothing: bool = True,
    ) -> None:  # fmt: skip
        self.image_size = image_size
        self.focal = focal_length(image_size[0], hfov_deg)
        self.center = np.array(image_size, dtype=float) / 2
        self.nominal_distance = nominal_distance
        self.gain = gain
        self.smoothing = smoothing
        # Position in m (beta in 1/(m/s)): x/y are precise, depth is noisier, so filter it more.
        self._f_xy = OneEuroFilter(min_cutoff=1.5, beta=10.0)
        self._f_z = OneEuroFilter(min_cutoff=0.8, beta=6.0)
        self._f_rot = OneEuroFilter(min_cutoff=1.5, beta=1.0)  # on the quaternion
        self._engaged = True
        self.reset()

    @property
    def engaged(self) -> bool:
        return self._engaged

    def set_engaged(self, engaged: bool) -> None:
        if engaged and not self._engaged:
            self._rebase = True  # compute the new offset at the next valid frame
            self._f_xy.reset()  # the hand has moved meanwhile: don't smooth across the gap
            self._f_z.reset()
        self._engaged = engaged

    def reset(self) -> None:
        """Forget everything: output back to the home pose, filters and chirality cleared."""
        for f in (self._f_xy, self._f_z, self._f_rot):
            f.reset()
        self.chirality: float | None = None
        self.last_rms_px: float | None = None  # reprojection error of the last solve
        self._T: np.ndarray | None = None  # last translation (warm start)
        self._offset = np.zeros(3)
        self._rebase = False
        self._quat = np.array([1.0, 0, 0, 0])
        self._pose = WristPose(np.zeros(3), np.eye(3), self.nominal_distance)

    def raw(self, hand: TrackedHand, chirality: float | None = None) -> _Raw | None:
        """Unsmoothed palm-centre position (view frame) and hand orientation, or None."""
        world = np.asarray(hand.world, dtype=float)
        image = np.asarray(hand.image, dtype=float)
        if world.shape != (21, 3) or image.shape != (21, 2):
            raise ValueError(f"expected 21 landmarks, got {world.shape} and {image.shape}")
        if not (np.all(np.isfinite(world)) and np.all(np.isfinite(image))):
            return None
        self._update_chirality(world, hand.label, chirality)

        solved = solve_translation(world, image, self.focal, self.center)
        if solved is None or solved[1] > MAX_RMS_PX:
            return None
        T, self.last_rms_px = solved
        self._T = T
        c = world[list(PALM_POINTS)].mean(axis=0) + T  # palm centre, camera frame
        distance = float(np.linalg.norm(c))
        c[2] = min(max(c[2], MIN_DEPTH), MAX_DEPTH)
        pos = np.array([-c[0], -c[1], self.nominal_distance - c[2]])
        quat = _rot_to_quat(hand_axes_view(world, self.chirality))
        return _Raw(pos, quat, distance)

    def update(self, hand: TrackedHand, t: float, chirality: float | None = None) -> WristPose:
        """Smoothed pose with the clutch applied. `chirality`: the Retargeter's, if known."""
        if not self._engaged:
            return self._copy()
        r = self.raw(hand, chirality)
        if r is None:  # garbage frame: keep the last pose
            return self._copy()

        # Orientation: keep the quaternion on the same hemisphere as the last one (q and -q are
        # the same rotation), filter its components, renormalise.
        q = r.quat if r.quat @ self._quat >= 0 else -r.quat
        pos = r.pos
        if self.smoothing:
            q = self._f_rot(q, t)
            pos = np.concatenate([self._f_xy(pos[:2], t), self._f_z(pos[2:], t)])
        self._quat = q / np.linalg.norm(q)

        if self._rebase:  # clutch just re-engaged: continue from the frozen position
            self._offset = self._pose.pos - self.gain * pos
            self._rebase = False
        self._pose = WristPose(self._offset + self.gain * pos, _quat_to_rot(self._quat),
                               r.distance)  # fmt: skip
        return self._copy()

    def _copy(self) -> WristPose:
        p = self._pose
        return WristPose(p.pos.copy(), p.rot.copy(), p.distance)

    def _update_chirality(self, world: np.ndarray, label: str, given: float | None) -> None:
        """Same rule as `Retargeter`: the bend cue if it is clear, else keep the last value, and
        the tracker's label only as the first guess ("Right" on an unmirrored image = a real
        left hand = -1)."""
        if given is not None:
            self.chirality = 1.0 if given > 0 else -1.0
            return
        cue = bend_cue(world, palm_frame(world, 1.0).x)
        if abs(cue) > self.CUE_THRESHOLD:
            self.chirality = 1.0 if cue > 0 else -1.0
        elif self.chirality is None:
            self.chirality = -1.0 if label == "Right" else 1.0
