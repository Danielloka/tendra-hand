"""Retargeting: a tracked human hand (21 3D landmarks) -> Tendra joint angles.

The landmarks follow the MediaPipe hand model (the same numbering is used by most hand
trackers):

    0 wrist
    thumb   1 CMC   2 MCP  3 IP   4 tip
    index   5 MCP   6 PIP  7 DIP  8 tip      (middle 9-12, ring 13-16, little 17-20)

Two methods, because the fingers and the thumb differ:

- **Fingers: model fit.** Tendra's fingers have the same joints as a human finger, so the
  human angles can be copied. They are found by fitting a small finger model (sideways +
  three bends, one bending plane) to the finger's four landmarks with nonlinear least squares
  (Gauss-Newton), started from a direct estimate. The palm plane is a least-squares fit through
  the wrist and all knuckles.
- **Thumb: position optimisation.** Tendra's thumb doesn't sit like a human one (at q = 0 it
  sticks straight out of the palm), so its angles can't be copied. Instead we look for thumb
  angles that put the robot's thumb joints and tip where the human's are (scaled to robot
  size), using the Jacobian and damped least squares on the MuJoCo model. When the human
  pinches (thumb tip near the index tip), the target moves to the robot's index tip, so pinches
  stay pinches even though the two hands have different proportions.

Frames: the human palm frame is built from the landmarks and mapped onto the robot palm frame
(+X thumb side, +Z along the fingers, -Y palm side). Left hands are mirrored onto the right
Tendra hand. Units: landmarks in metres, angles in radians, positive = closing.
"""

import math
from dataclasses import dataclass
from itertools import pairwise
from types import MappingProxyType

import mujoco
import numpy as np

from tendra.joints import HandSpec, get_hand

WRIST = 0
THUMB = (1, 2, 3, 4)
FINGERS = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12), "ring": (13, 14, 15, 16),
           "little": (17, 18, 19, 20)}  # fmt: skip
FINGER_JOINTS = ("mcp_flex", "pip", "dip", "mcp_abd")
PARAM_INDEX = {"mcp_abd": 0, "mcp_flex": 1, "pip": 2, "dip": 3}  # finger fit parameter order

# Pinch blending: human thumb-index tip distance (m) where the pinch target fully takes over /
# stops having any effect.
# Solvers stop once a step changes no angle by more than this (rad, = 0.06 deg): far below the
# tracker's noise (several degrees), and it saves iterations.
ANGLE_TOLERANCE = 1e-3

PINCH_CLOSE = 0.02
PINCH_FAR = 0.05


def _path_length(points) -> float:
    return float(sum(np.linalg.norm(b - a) for a, b in pairwise(points)))


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product of two 3-vectors (much faster than np.cross for single vectors)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]])  # fmt: skip


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


@dataclass(frozen=True)
class PalmFrame:
    """Human palm axes, expressed in the tracker's coordinates (unit vectors)."""

    x: np.ndarray  # toward the thumb side, in the palm plane
    up: np.ndarray  # wrist -> middle knuckle
    palm: np.ndarray  # out of the palm (the side the fingers curl toward)

    def to_robot(self, v: np.ndarray) -> np.ndarray:
        """A human vector in robot palm axes: X thumb side, Y back of the hand, Z fingers."""
        return np.array([v @ self.x, -(v @ self.palm), v @ self.up])


PALM_POINTS = (WRIST, 5, 9, 13, 17)  # wrist + the four knuckles: the palm plane


def _plane_normal(points: np.ndarray) -> np.ndarray:
    """Unit normal of the least-squares plane through `points`.

    Principal component analysis: the singular vector of the centred points with the smallest
    singular value is the direction they vary least in, i.e. the plane normal.
    """
    return np.linalg.svd(points - points.mean(axis=0), full_matrices=False)[2][-1]


def _reject(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """`v` without its component along the unit vector `axis`."""
    return v - (v @ axis) * axis


def palm_frame(lm: np.ndarray, chirality: float) -> PalmFrame:
    """Palm axes from the landmarks. `chirality` +1: right-hand geometry, -1: left (mirrored).

    The palm plane is fitted through the wrist and all four knuckles (least squares), so one
    noisy landmark can't tilt it. `up` and `x` are then projected into that plane.
    """
    n = _plane_normal(lm[list(PALM_POINTS)])
    up = _unit(_reject(lm[9] - lm[WRIST], n))
    x = _unit(_reject(_reject(lm[5] - lm[17], n), up))  # little -> index knuckle: thumb side
    return PalmFrame(x=x, up=up, palm=chirality * _cross(x, up))


def bend_cue(lm: np.ndarray, x: np.ndarray) -> float:
    """Total bend of the finger joints about the thumb-side axis `x` (sum of sines).

    Every finger joint (MCP, PIP, DIP) flexes about an axis that points to the thumb side on a
    right hand and away from it on a left (mirrored) one: for two successive bones a and b,
    (a x b) . x = +sin(bend) for a right hand. The sine stays positive for any bend between 0
    and 180 deg, so the sum keeps its sign even in a tight fist. (The first version compared bone
    *directions*, which flips once a curled finger points back at the wrist.)
    """
    total = 0.0
    for idx in FINGERS.values():
        bones = [_unit(b) for b in np.diff(lm[[WRIST, *idx]], axis=0)]
        total += sum(float(_cross(a, b) @ x) for a, b in pairwise(bones))
    return total


def bone_lengths(lm: np.ndarray, finger: str) -> np.ndarray:
    """Lengths of the finger's three bones (MCP-PIP, PIP-DIP, DIP-tip) in this frame."""
    return np.linalg.norm(np.diff(lm[list(FINGERS[finger])], axis=0), axis=1)


def _finger_axes(lm: np.ndarray, frame: PalmFrame, finger: str):
    """Zero-abduction direction `f` and `side` (toward the thumb), both in the palm plane.

    The reference is the palm's `up` axis for every finger, because Tendra's fingers are
    parallel at mcp_abd = 0. (The first version used each finger's wrist -> knuckle line, but
    on a human hand those lines fan out by up to ~35 deg, so spread fingers read as ~0 and
    fingers held together read as bent toward the middle finger.)
    """
    return frame.up, frame.x


def _finger_model(params: np.ndarray, base: np.ndarray, lengths: np.ndarray, f, side, n):
    """Forward kinematics of human fingers and their Jacobians, for a batch of B fingers.

    Model: the MCP is a universal joint, sideways (abduction `a`) about the palm normal first,
    then flexion; PIP and DIP are hinges parallel to the MCP flexion axis. All three bones then
    lie in one plane, spanned by D(a) (the abducted direction) and the palm normal n:

        u(phi) = cos(phi) D + sin(phi) n,    D(a) = cos(a) f + sin(a) side
        P_k    = base + sum_{i<=k} L_i u(phi_i),   phi_i = theta_1 + ... + theta_i

    Shapes: params (B, 4) = (a, theta_1..3); base, lengths, f, side (B, 3); n (3,). Single
    fingers (1-D arrays) work too. Returns the PIP, DIP and tip positions (B, 9) and
    d(positions)/d(params) (B, 9, 4). All fingers are computed together, because for arrays
    this small NumPy's per-call overhead costs more than the arithmetic.
    """
    single = np.ndim(params) == 1
    P, base, L, f, side = (np.atleast_2d(v) for v in (params, base, lengths, f, side))
    a = P[:, :1]
    D = np.cos(a) * f + np.sin(a) * side  # (B, 3)
    dD = np.cos(a) * side - np.sin(a) * f
    phis = np.cumsum(P[:, 1:], axis=1)[:, :, None]  # (B, 3, 1)
    c, s, L = np.cos(phis), np.sin(phis), L[:, :, None]
    D, dD = D[:, None, :], dD[:, None, :]
    pts = base[:, None, :] + np.cumsum(L * (c * D + s * n), axis=1)  # (B, 3 points, 3)
    # d(bone i)/d(phi_i); theta_j turns every bone from j on, so
    # d P_i / d theta_j = sum_{k=j..i} d(bone k)/d(phi_k) = C_i - C_{j-1}
    C = np.cumsum(L * (c * n - s * D), axis=1)
    jac = np.zeros((len(P), 3, 3, 4))
    jac[..., 0] = np.cumsum(L * c * dD, axis=1)
    jac[..., 1] = C
    jac[:, 1:, :, 2] = C[:, 1:] - C[:, :1]
    jac[:, 2:, :, 3] = C[:, 2:] - C[:, 1:2]
    pts, jac = pts.reshape(-1, 9), jac.reshape(-1, 9, 4)
    return (pts[0], jac[0]) if single else (pts, jac)


def _closed_form(lm: np.ndarray, frame: PalmFrame, finger: str) -> np.ndarray:
    """Direct estimate of (abd, mcp_flex, pip, dip), used to start the fit."""
    m, p, d, t = (lm[i] for i in FINGERS[finger])
    n = frame.palm
    f, side = _finger_axes(lm, frame, finger)
    b1, b2, b3 = p - m, d - p, t - d
    # MCP: elevation of the first bone out of the palm plane (past 90 deg if it points back),
    # abduction from its direction within the plane (faded out near 90 deg, where it's tiny).
    inplane = _reject(b1, n)
    elev = math.atan2(b1 @ n, np.linalg.norm(inplane))
    forward = inplane @ f >= 0
    flex = elev if forward else math.pi - elev
    v = inplane if forward else -inplane
    abd = math.atan2(v @ side, v @ f) * min(1.0, math.cos(elev) / 0.5)
    # PIP/DIP: signed angles about the hinge axis, with the bones projected onto the finger's
    # bending plane (so sideways tracking noise doesn't count as bending).
    hinge = _unit(_cross(math.cos(abd) * f + math.sin(abd) * side, n))
    return np.array([abd, flex, _hinge_angle(b1, b2, hinge), _hinge_angle(b2, b3, hinge)])


def _hinge_angle(a: np.ndarray, b: np.ndarray, hinge: np.ndarray) -> float:
    a, b = _reject(a, hinge), _reject(b, hinge)
    return math.atan2(float(_cross(a, b) @ hinge), float(a @ b))


def fit_fingers(
    lm: np.ndarray, frame: PalmFrame, fingers, starts: dict | None = None,
    lengths: dict | None = None, iterations: int = 8,
) -> dict[str, np.ndarray]:  # fmt: skip
    """Joint angles of several fingers, each fitted to all its landmarks at once.

    Nonlinear least squares (Gauss-Newton with a little Levenberg-Marquardt damping): for each
    finger, find params = (abd, mcp_flex, pip, dip) whose finger model (`_finger_model`) puts
    the PIP, DIP and tip where the tracker saw them. Fitting the whole finger at once averages
    out noise in single landmarks and keeps the angles consistent with a real finger (one
    bending plane). Each finger starts from its closed-form estimate or from `starts[finger]`
    (last frame's result), whichever fits better. `lengths[finger]` (three bone lengths)
    default to this frame's; an average over many frames is less noisy.

    Returns {finger: params (rad)}; pass it back as `starts` next frame.
    """
    fingers = list(fingers)
    starts, lengths = starts or {}, lengths or {}
    idx = np.array([FINGERS[name] for name in fingers])
    base, observed = lm[idx[:, 0]], lm[idx[:, 1:]].reshape(len(fingers), 9)
    L = np.array([lengths.get(name, bone_lengths(lm, name)) for name in fingers])
    f, side = map(np.array, zip(*(_finger_axes(lm, frame, name) for name in fingers), strict=True))
    n = frame.palm

    def residual(x):
        pts, jac = _finger_model(x, base, L, f, side, n)
        return pts - observed, jac

    x = np.array([_closed_form(lm, frame, name) for name in fingers])
    warm = np.array([starts.get(name, x[k]) for k, name in enumerate(fingers)], dtype=float)
    better = (residual(warm)[0] ** 2).sum(axis=1) < (residual(x)[0] ** 2).sum(axis=1)
    x[better] = warm[better]

    damping = 1e-6 * L.sum(axis=1) ** 2  # scaled to the finger size
    eye = np.eye(4)
    for _ in range(iterations):
        r, J = residual(x)
        JtJ = np.einsum("bki,bkj->bij", J, J) + damping[:, None, None] * eye
        step = np.linalg.solve(JtJ, -np.einsum("bki,bk->bi", J, r)[..., None])[..., 0]
        x = x + step
        if np.abs(step).max() < ANGLE_TOLERANCE:
            break
    x = (x + math.pi) % (2 * math.pi) - math.pi  # keep angles in (-pi, pi]
    return dict(zip(fingers, x, strict=True))


def finger_angles(
    lm: np.ndarray, frame: PalmFrame, finger: str, start: np.ndarray | None = None,
    lengths: np.ndarray | None = None,
) -> tuple[dict[str, float], np.ndarray]:  # fmt: skip
    """One finger's angles, {mcp_flex, pip, dip, mcp_abd} (rad), and its params (see
    `fit_fingers`)."""
    starts = {finger: start} if start is not None else None
    lens = {finger: lengths} if lengths is not None else None
    x = fit_fingers(lm, frame, [finger], starts, lens)[finger]
    return {"mcp_abd": x[0], "mcp_flex": x[1], "pip": x[2], "dip": x[3]}, x


class ThumbSolver:
    """Finds thumb angles that place the robot thumb like the human one (damped least squares).

    Keypoints (robot <- human landmark): thumb base (the CMC rotation axis) <- 1, MCP joint <- 2,
    IP joint <- 3, tip <- 4. Targets are positions relative to the thumb base, in robot palm
    axes (the model's world frame, since the palm is welded to the world).
    """

    WEIGHTS = MappingProxyType({"mcp": 0.3, "ip": 0.6, "tip": 1.0})
    DAMPING = 0.01  # m; larger = smoother but slower to converge

    def __init__(self, spec: HandSpec, model: mujoco.MjModel | None = None):
        self.spec = spec
        self.model = model or mujoco.MjModel.from_xml_path(str(spec.model_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        names = spec.joint_names
        self._qadr = np.array([m.joint(n).qposadr[0] for n in names])
        self.thumb = np.array([i for i, n in enumerate(names) if n.startswith("thumb_")])
        self._dof = np.array([m.joint(names[i]).dofadr[0] for i in self.thumb])
        mcp = "thumb_mcp_flex" if "thumb_mcp_flex" in names else "thumb_mcp"
        self._joints = {"base": m.joint("thumb_cmc_rot").id, "mcp": m.joint(mcp).id,
                        "ip": m.joint("thumb_ip").id}  # fmt: skip
        self._tip = m.site("thumb_tip").id
        self._index_tip = m.site("index_tip").id
        self.lower, self.upper = spec.lower[self.thumb], spec.upper[self.thumb]

        # Thumb size for scaling: MCP -> IP -> tip. (Base -> MCP is left out: the two thumb-base
        # axes don't meet, so that distance changes with the pose.)
        p = self.points(np.zeros(spec.num_joints))
        self.thumb_length = _path_length([p["mcp"], p["ip"], p["tip"]])
        self.index_length = self._index_length()

    def points(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Robot keypoints (m, palm frame) for joint angles `q` (spec order)."""
        d = self.data
        d.qpos[self._qadr] = q
        mujoco.mj_kinematics(self.model, d)
        pts = {k: d.xanchor[j].copy() for k, j in self._joints.items()}
        pts["tip"] = d.site_xpos[self._tip].copy()
        pts["index_tip"] = d.site_xpos[self._index_tip].copy()
        return pts

    def _index_length(self) -> float:
        d, m = self.data, self.model
        self.points(np.zeros(self.spec.num_joints))
        chain = [d.xanchor[m.joint(f"index_{j}").id] for j in ("mcp_flex", "pip", "dip")]
        chain.append(d.site_xpos[self._index_tip])
        return _path_length(chain)

    def solve(
        self, q: np.ndarray, targets: dict[str, np.ndarray], weights: dict[str, float],
        iterations: int = 8,
    ) -> np.ndarray:  # fmt: skip
        """Improve the thumb entries of `q` (warm start) so keypoints reach `targets`.

        `targets` are keypoint positions relative to the thumb base. Other joints are left as
        they are (they matter because the pinch target is the robot's index tip).
        """
        q = q.copy()
        m, d = self.model, self.data
        jac = np.zeros((3, m.nv))
        for _ in range(iterations):
            pts = self.points(q)
            mujoco.mj_comPos(m, d)
            rows, errs = [], []
            for key, target in targets.items():
                w = math.sqrt(weights[key])
                if key == "tip":
                    mujoco.mj_jacSite(m, d, jac, None, self._tip)
                else:
                    body = m.jnt_bodyid[self._joints[key]]
                    mujoco.mj_jac(m, d, jac, None, pts[key], body)
                rows.append(w * jac[:, self._dof])
                errs.append(w * (target - (pts[key] - pts["base"])))
            J, e = np.vstack(rows), np.concatenate(errs)
            dq = np.linalg.solve(J.T @ J + self.DAMPING**2 * np.eye(len(self.thumb)), J.T @ e)
            q[self.thumb] = np.clip(q[self.thumb] + dq, self.lower, self.upper)
            if np.abs(dq).max() < ANGLE_TOLERANCE:
                break
        return q


class OneEuroFilter:
    """Smooths a noisy signal with little lag (Casiez et al., CHI 2012).

    At rest it filters strongly (removes tracker jitter); when the signal moves fast the cutoff
    rises, so fast motions aren't delayed. `min_cutoff` (Hz) sets the jitter filtering, `beta`
    how quickly the cutoff opens with speed.
    """

    def __init__(self, min_cutoff: float = 2.0, beta: float = 0.8, d_cutoff: float = 1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self) -> None:
        self._x = self._dx = self._t = None

    @staticmethod
    def _alpha(dt: float, cutoff: np.ndarray | float):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x: np.ndarray, t: float) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if self._x is None:
            self._x, self._dx, self._t = x.copy(), np.zeros_like(x), t
            return x.copy()
        dt = t - self._t
        if dt <= 0:
            return self._x.copy()
        a_d = self._alpha(dt, self.d_cutoff)
        self._dx = a_d * (x - self._x) / dt + (1 - a_d) * self._dx
        a = self._alpha(dt, self.min_cutoff + self.beta * np.abs(self._dx))
        self._x = a * x + (1 - a) * self._x
        self._t = t
        return self._x.copy()


class Retargeter:
    """Human hand landmarks -> Tendra joint targets (spec order, clamped to the joint limits).

        rt = Retargeter("v1")
        q = rt(landmarks, t=time.monotonic(), label="Left")   # smoothed
        rt.calibrate(landmarks)   # hold your hand open and flat: this pose becomes q = 0

    `label` is the tracker's handedness guess. MediaPipe assumes a mirrored (selfie) image, so on
    a normal webcam image a real right hand is labelled "Left"; both give the same geometry,
    which is all that matters here. The label is only a starting guess: the side the palm faces
    is re-checked every frame from the axis the finger joints bend about (`bend_cue`).
    """

    LENGTH_RATE = 0.05  # bone-length running average: ~20 frames of memory
    CUE_THRESHOLD = 0.5  # |bend cue| needed to trust it (sum of sines: ~3 deg per joint)

    def __init__(
        self, hand: str | HandSpec = "v1", *, model: mujoco.MjModel | None = None,
        smoothing: bool = True,
    ):  # fmt: skip
        self.spec = get_hand(hand)
        self.thumb = ThumbSolver(self.spec, model)
        self.filter = OneEuroFilter() if smoothing else None
        self.chirality: float | None = None
        self.offsets = np.zeros(self.spec.num_joints)
        self.pinch = 0.0  # 0..1, how much the last frame was treated as a pinch
        self._q = np.zeros(self.spec.num_joints)  # last unfiltered result (thumb warm start)
        self._finger_fit: dict[str, np.ndarray] = {}  # last fit per finger (warm start)
        self._lengths: dict[str, np.ndarray] = {}  # running average of the bone lengths
        self._fingers = [
            (i, name.split("_", 1)[0], name.split("_", 1)[1])
            for i, name in enumerate(self.spec.joint_names)
            if not name.startswith("thumb_")
        ]
        self._finger_names = list(dict.fromkeys(f for _, f, _ in self._fingers))

    def __call__(self, lm: np.ndarray, t: float, label: str | None = None) -> np.ndarray:
        q = self.raw(lm, label)
        if self.filter:
            q = self.filter(q, t)
        return np.clip(q, self.spec.lower, self.spec.upper)

    def raw(self, lm: np.ndarray, label: str | None = None) -> np.ndarray:
        """Unsmoothed joint targets for one frame of landmarks (21 x 3, metres)."""
        lm = np.asarray(lm, dtype=float)
        if lm.shape != (21, 3):
            raise ValueError(f"expected 21 x 3 landmarks, got {lm.shape}")
        self._update_chirality(lm, label)
        frame = palm_frame(lm, self.chirality)

        q = self._q.copy()
        for finger in self._finger_names:
            measured = bone_lengths(lm, finger)
            avg = self._lengths.get(finger)
            self._lengths[finger] = (
                measured if avg is None else avg + self.LENGTH_RATE * (measured - avg)
            )
        self._finger_fit = fit_fingers(
            lm, frame, self._finger_names, self._finger_fit, self._lengths
        )
        for i, finger, joint in self._fingers:
            params = self._finger_fit[finger]
            value = params[PARAM_INDEX[joint]]
            if joint == "mcp_abd":
                # Anatomy: the MCP collateral ligaments tighten as the knuckle flexes, so a
                # finger can hardly move sideways at 90 deg. Fading abduction with cos(flexion)
                # copies that, and keeps curled robot fingers from swinging into each other.
                value *= max(0.0, math.cos(params[PARAM_INDEX["mcp_flex"]]))
            q[i] = value - self.offsets[i]
        q = np.clip(q, self.spec.lower, self.spec.upper)

        q = self.thumb.solve(q, *self._thumb_targets(lm, frame, q))
        self._q = q
        return q.copy()

    def calibrate(self, lm: np.ndarray, label: str | None = None) -> None:
        """Take this pose (hand open, fingers straight and together) as the fingers' zero."""
        self.offsets[:] = 0.0
        self.offsets = np.where(self._finger_mask(), self.raw(lm, label), 0.0)
        if self.filter:
            self.filter.reset()

    def flip(self) -> None:
        """Swap which side is the palm (if the fingers bend backward)."""
        if self.chirality is not None:
            self.chirality = -self.chirality

    def _finger_mask(self) -> np.ndarray:
        mask = np.zeros(self.spec.num_joints, dtype=bool)
        mask[[i for i, _, _ in self._fingers]] = True
        return mask

    def _update_chirality(self, lm: np.ndarray, label: str | None) -> None:
        cue = bend_cue(lm, palm_frame(lm, 1.0).x)  # x doesn't depend on the chirality
        if abs(cue) > self.CUE_THRESHOLD:
            self.chirality = 1.0 if cue > 0 else -1.0
        elif self.chirality is None:
            self.chirality = -1.0 if label == "Right" else 1.0

    def _thumb_targets(self, lm: np.ndarray, frame: PalmFrame, q: np.ndarray):
        human_thumb = _path_length(lm[list(THUMB[1:])])
        human_index = _path_length(lm[list(FINGERS["index"])])
        s_thumb = self.thumb.thumb_length / human_thumb
        s_hand = self.thumb.index_length / human_index

        base = lm[THUMB[0]]
        targets = {
            key: s_thumb * frame.to_robot(lm[i] - base)
            for key, i in (("mcp", THUMB[1]), ("ip", THUMB[2]), ("tip", THUMB[3]))
        }
        # Pinch: aim the thumb tip at the robot's index tip, offset like the human's.
        gap = lm[THUMB[3]] - lm[FINGERS["index"][3]]
        w = float(np.clip((PINCH_FAR - np.linalg.norm(gap)) / (PINCH_FAR - PINCH_CLOSE), 0, 1))
        self.pinch = w
        weights = dict(ThumbSolver.WEIGHTS)
        if w > 0:
            pts = self.thumb.points(q)
            pinch_tip = pts["index_tip"] + s_hand * frame.to_robot(gap) - pts["base"]
            targets["tip"] = (1 - w) * targets["tip"] + w * pinch_tip
            weights["tip"] *= 1 + 4 * w  # the tip matters most in a pinch
        return targets, weights
