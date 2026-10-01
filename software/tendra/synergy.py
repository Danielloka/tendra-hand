"""Hand synergies ("eigengrasps"): a few human-like joint patterns that span most grasps.

Humans don't move their finger joints independently. Santello et al. (1998) found that two patterns
explain ~80% of the variance of human grasp postures: "close everything" and "thumb opposition +
radial/ulnar shaping"; a handful explain ~95%. Robot grasp planners use the same idea
(eigengrasps, Ciocarlie & Allen 2009), and so do dexterous RL policies: acting in synergy space
makes learning faster and the motion more human-like, because odd postures (one finger curled
backward, fingertip hooks) sit far outside the space.

A `Synergies` object maps a small action vector `a` (each entry in [-1, 1]) to the 16 servo joint
angles (V1; each DIP follows its PIP through the coupling tendon):

    q = rest + basis @ a            (then clipped to the joint limits)

`default_synergies()` is hand-designed from human anatomy (below). `fit_synergies()` learns them
by PCA from recorded postures, e.g. the owner's own teleop demos, retargeted to Tendra joints:
then the hand literally moves in the operator's eigen-postures.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .joints import V1

FINGERS = ("index", "middle", "ring", "little")


def _vec(weights: dict[str, float]) -> np.ndarray:
    """A 20-vector (V1 order) from {joint name: rad}; unknown names raise."""
    names = V1.joint_names
    out = np.zeros(len(names))
    for name, w in weights.items():
        out[names.index(name)] = w
    return out


def _fingers(**joint_weights: float | Sequence[float]) -> dict[str, float]:
    """Weights for the four fingers; a sequence gives one weight per finger (index → little)."""
    out = {}
    for joint, w in joint_weights.items():
        ws = w if isinstance(w, Sequence) else [w] * 4
        out.update({f"{f}_{joint}": float(x) for f, x in zip(FINGERS, ws, strict=True)})
    return out


# Hand-designed synergies (radians at action +1). Signs: flexion positive = closing; mcp_abd
# positive = toward the thumb (every finger); thumb_cmc_rot positive = across the palm.
DEFAULT_SYNERGIES: dict[str, dict[str, float]] = {
    # 1. Close the hand: all fingers + thumb together (the first human synergy, power grasp).
    "close": {
        **_fingers(mcp_flex=1.4, pip=1.5),  # DIP: 0.75 x PIP (coupled)
        "thumb_cmc_flex": 0.75, "thumb_mcp_flex": 0.4, "thumb_ip": 0.4,  # cmc_flex limit 45 deg
    },
    # 2. Opposition: thumb swings across toward index/middle, which pre-curl (precision/tripod).
    "oppose": {
        "thumb_cmc_rot": 0.6, "thumb_cmc_flex": 0.35,
        **_fingers(mcp_flex=(0.5, 0.35, 0.0, -0.2), pip=(0.4, 0.3, 0.0, 0.0)),
    },
    # 3. Spread: fan the fingers apart (index toward the thumb, ring and little away).
    "spread": _fingers(mcp_abd=(0.26, 0.0, -0.2, -0.33)),
    # 4. Hook vs. flat: curl the distal joints while the knuckles straighten (and back).
    "hook": _fingers(mcp_flex=-0.5, pip=0.6),
    # 5. Radial/ulnar roll: little side closes while the index side opens (second human synergy).
    "roll": _fingers(mcp_flex=(-0.4, -0.13, 0.13, 0.4), pip=(-0.3, -0.1, 0.1, 0.3)),
    # 6. Thumb curl: bend the thumb's own joints (flat pad vs. tip contact).
    "thumb_curl": {"thumb_mcp_flex": 0.6, "thumb_ip": 0.6, "thumb_cmc_flex": -0.25},
}  # fmt: skip

# Rest posture (action 0): relaxed, slightly curled (35% of "close"), like a human hand hanging
# loose. "close" at -1 then gives a flat, open hand (clipped at the limits), at +1 the full power
# grasp (the same angles as the scene's scripted grasp, which holds the cylinder).
REST_CLOSE = 0.35


@dataclass(frozen=True)
class Synergies:
    """q = clip(rest + basis @ a); basis columns are joint patterns (rad at a_i = 1)."""

    rest: np.ndarray  # (16,) = V1.num_joints
    basis: np.ndarray  # (16, k)
    names: tuple[str, ...]

    @property
    def k(self) -> int:
        return self.basis.shape[1]

    def posture(self, a: np.ndarray) -> np.ndarray:
        """Joint angles (rad, clipped to the limits) for synergy coefficients `a` (k,) or (n, k)."""
        q = self.rest + np.asarray(a, dtype=float) @ self.basis.T
        return np.clip(q, V1.lower, V1.upper)

    def project(self, q: np.ndarray) -> np.ndarray:
        """Least-squares synergy coefficients for posture(s) `q` (the inverse of `posture`)."""
        a, *_ = np.linalg.lstsq(self.basis, (np.asarray(q, dtype=float) - self.rest).T, rcond=None)
        return a.T

    def residual(self, q: np.ndarray) -> np.ndarray:
        """The part of `q` the synergies can't express (0 for a perfectly 'human' posture)."""
        q = np.asarray(q, dtype=float)
        return q - (self.rest + self.project(q) @ self.basis.T)

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        np.savez(path, rest=self.rest, basis=self.basis, names=np.array(self.names))
        return path

    @classmethod
    def load(cls, path: str | Path) -> Synergies:
        f = np.load(path)
        if f["rest"].shape != (V1.num_joints,):
            raise ValueError(
                f"{path}: synergies for {f['rest'].shape[0]} joints, V1 has {V1.num_joints} servos "
                "(fitted on the 20-servo layout?): fit them again"
            )
        return cls(f["rest"], f["basis"], tuple(str(n) for n in f["names"]))


def default_synergies() -> Synergies:
    close = _vec(DEFAULT_SYNERGIES["close"])
    basis = np.column_stack([_vec(w) for w in DEFAULT_SYNERGIES.values()])
    basis[:, 0] = (1.0 - REST_CLOSE) * close  # rest + basis = the full grasp at a_close = +1
    return Synergies(REST_CLOSE * close, basis, tuple(DEFAULT_SYNERGIES))


def fit_synergies(postures: np.ndarray | Iterable[np.ndarray], k: int = 6,
                  spread: float = 2.0) -> Synergies:  # fmt: skip
    """PCA eigengrasps from recorded postures (n, 16): rest = mean, basis = top-k components
    scaled so a = ±1 covers ±`spread` standard deviations."""
    q = np.asarray(postures if isinstance(postures, np.ndarray) else np.vstack(list(postures)))
    if q.ndim != 2 or q.shape[1] != V1.num_joints:
        raise ValueError(f"expected postures of shape (n, {V1.num_joints}), got {q.shape}")
    if len(q) < 2 * k:
        raise ValueError(f"need at least {2 * k} postures to fit {k} synergies, got {len(q)}")
    mean = q.mean(axis=0)
    _, s, vt = np.linalg.svd(q - mean, full_matrices=False)
    std = s[:k] / np.sqrt(len(q) - 1)
    basis = vt[:k].T * (spread * std)
    # Make the first component "closing" (positive = more flexion), for readable actions.
    signs = np.sign(basis.sum(axis=0))
    basis *= np.where(signs == 0, 1.0, signs)
    return Synergies(mean, basis, tuple(f"pc{i + 1}" for i in range(k)))


def explained_variance(postures: np.ndarray, k: int) -> float:
    """Fraction of posture variance the first k principal components explain (0..1)."""
    q = np.asarray(postures, dtype=float)
    s = np.linalg.svd(q - q.mean(axis=0), compute_uv=False)
    return float((s[:k] ** 2).sum() / max((s**2).sum(), 1e-12))


def postures_from_datasets(roots: Iterable[str | Path], column: str = "action") -> np.ndarray:
    """All finger postures (n, 16) from recorded grasp datasets (the first 16 columns of the
    action = the finger targets from teleop retargeting)."""
    from .dataset import Dataset

    rows = []
    for root in roots:
        ds = Dataset(root)
        names = ds._info.get(f"{column}_names") or []
        want = [f"{j}_target" if column == "action" else j for j in V1.joint_names]
        if names and names[: V1.num_joints] != want:
            raise ValueError(
                f"{root}: recorded with another joint layout (the 20-servo V1?), not the current "
                f"16 servos: re-record it"
            )
        for meta in ds:
            rows.append(ds.load(meta["episode_index"])[column][:, : V1.num_joints])
    if not rows:
        raise ValueError("no episodes found")
    return np.vstack(rows).astype(float)
