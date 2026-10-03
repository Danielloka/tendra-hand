"""Roll out a trained policy in the GPU env and film it (mp4), for Colab or a laptop.

    MUJOCO_GL=egl python rollout_video.py --bundle tendra_gpu.npz --params params_best.pkl \
        --out grasp.mp4 --episodes 4

The physics is the training env (`TendraGrasp`, MuJoCo Warp, one world at a time); the frames are
drawn afterwards with `mujoco.Renderer` from the saved positions, camera "view" (from the
operator's side). The policy runs in numpy (`policy.BraxPolicy`, deterministic action), exactly as
it would on the real hand. Standalone like the other files in the Colab kit.

On Colab set `MUJOCO_GL=egl` *before* importing mujoco (the notebook does).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import jax
import jax.numpy as jp
import mujoco
import numpy as np

try:
    from .grasp_env_jax import TendraGrasp
    from .policy import BraxPolicy
except ImportError:  # plain script next to the other files (Colab)
    from grasp_env_jax import TendraGrasp
    from policy import BraxPolicy


def rollout(env: TendraGrasp, policy: BraxPolicy, episodes: int, seed: int = 0) -> list[dict]:
    """Run `episodes` episodes; each result has the per-step poses and the final metrics."""
    reset, step = jax.jit(env.reset), jax.jit(env.step)
    out = []
    for ep in range(episodes):
        state = reset(jax.random.PRNGKey(seed + ep))
        qpos, mpos, mquat, success, lift = [], [], [], 0.0, 0.0
        for _ in range(env._config.episode_length):
            obs = {k: np.asarray(v) for k, v in state.obs.items()}
            action = jp.asarray(policy(obs), dtype=jp.float32)
            state = step(state, action)
            qpos.append(np.asarray(state.data.qpos))
            mpos.append(np.asarray(state.data.mocap_pos))
            mquat.append(np.asarray(state.data.mocap_quat))
            success = max(success, float(state.metrics["success"]))
            lift = max(lift, float(state.metrics["lift_cm"]) * env._config.episode_length)
            if float(state.done) > 0:
                break
        out.append({"qpos": np.stack(qpos), "mocap_pos": np.stack(mpos),
                    "mocap_quat": np.stack(mquat), "success": success,
                    "obj": int(state.info["obj"])})  # fmt: skip
    return out


def render(
    model: mujoco.MjModel,
    frames: list[dict],
    width: int = 640,
    height: int = 480,
    camera: str = "view",
) -> list[np.ndarray]:
    """Draw every step of every episode; returns RGB frames."""
    data = mujoco.MjData(model)
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    images = []
    with mujoco.Renderer(model, height, width) as r:
        for ep in frames:
            for q, p, quat in zip(ep["qpos"], ep["mocap_pos"], ep["mocap_quat"], strict=True):
                data.qpos[:] = q
                data.mocap_pos[:] = p
                data.mocap_quat[:] = quat
                mujoco.mj_forward(model, data)
                r.update_scene(data, camera=cam if cam >= 0 else -1)
                images.append(r.render().copy())
    return images


def write_video(images: list[np.ndarray], path: Path, fps: int = 21) -> Path:
    """mp4 via mediapy (preinstalled on Colab) or imageio-ffmpeg."""
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import mediapy

        mediapy.write_video(str(path), images, fps=fps)
    except ImportError:
        import imageio.v2 as imageio

        imageio.mimwrite(str(path), images, fps=fps)
    return path


def film(bundle: str | Path, params: str | Path, out: str | Path, episodes: int = 4,
         seed: int = 0, objects: list[str] | None = None, width: int = 640,
         height: int = 480) -> tuple[Path, list[dict]]:  # fmt: skip
    """Policy file -> mp4. Returns the path and the episode summaries."""
    overrides = {"demo_prob": 0.0, "naconmax": 96 * 8}
    if objects:
        overrides["objects"] = objects
    env = TendraGrasp(bundle, config_overrides=overrides)
    policy = BraxPolicy.load(params)
    eps = rollout(env, policy, episodes, seed)
    ctrl_dt = env._config.ctrl_dt
    images = render(env.mj_model, eps, width, height)
    path = write_video(images, Path(out), fps=round(1 / ctrl_dt))
    return path, [{"success": e["success"], "steps": len(e["qpos"]), "obj": e["obj"]}
                  for e in eps]  # fmt: skip


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--bundle", default="tendra_gpu.npz")
    p.add_argument("--params", required=True, help="params_best.pkl / params_latest.pkl / .npz")
    p.add_argument("--out", default="grasp.mp4")
    p.add_argument("--episodes", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--objects", nargs="*", default=None)
    args = p.parse_args()
    path, summary = film(args.bundle, args.params, args.out, args.episodes, args.seed,
                         args.objects)  # fmt: skip
    print(path, summary)


if __name__ == "__main__":
    main()
