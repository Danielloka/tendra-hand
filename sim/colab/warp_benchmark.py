"""GPU physics benchmark: how fast does the Tendra grasp scene run in MuJoCo Warp?

Runs N copies ("worlds") of the V1 hand grasping and lifting the cylinder in parallel, replaying
the scripted grasp's commands, and reports control steps per second (the number that sets RL
training speed; the laptop's CPU does ~270-650). Self-contained: needs only
`tendra_warp_bench.npz` (the compiled scene + one grasp trajectory), made by
`sim/colab/make_bench_file.py`. Used by `warp_benchmark.ipynb` on Google Colab (free T4 GPU).

    python warp_benchmark.py tendra_warp_bench.npz 256 1024 4096
"""

import sys
import tempfile
import time
import warnings
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx

warnings.filterwarnings("ignore")

CONTACTS_PER_WORLD = 96  # a grasping hand has ~10-40 contacts; Warp sizes the buffer for all worlds
CONSTRAINTS_PER_WORLD = 600


def load(path: str) -> tuple[mujoco.MjModel, dict]:
    f = dict(np.load(path))
    with tempfile.TemporaryDirectory() as tmp:
        mjb = Path(tmp) / "model.mjb"
        mjb.write_bytes(f.pop("model").tobytes())
        model = mujoco.MjModel.from_binary_path(str(mjb))
    return model, f


def benchmark(model: mujoco.MjModel, traj: dict, nworld: int) -> dict:
    d0 = mujoco.MjData(model)
    d0.qpos[:], d0.qvel[:], d0.ctrl[:] = traj["qpos"][0], traj["qvel"][0], traj["ctrl"][0]
    d0.mocap_pos[:], d0.mocap_quat[:] = traj["mocap_pos"][0], traj["mocap_quat"][0]
    mujoco.mj_forward(model, d0)
    mx = mjx.put_model(model, impl="warp")
    batch = jax.vmap(
        lambda i: mjx.put_data(model, d0, impl="warp", naconmax=CONTACTS_PER_WORLD * nworld,
                               njmax=CONSTRAINTS_PER_WORLD, dummy_arg_for_batching=i)
    )(jnp.arange(nworld))  # fmt: skip
    ctrl, mpos, mquat = (jnp.asarray(traj[k]) for k in ("ctrl", "mocap_pos", "mocap_quat"))
    substeps, frames = int(traj["substeps"]), len(traj["ctrl"])
    step = jax.vmap(lambda d: mjx.step(mx, d))

    def control_step(d, k):
        d = d.replace(ctrl=jnp.broadcast_to(ctrl[k], d.ctrl.shape),
                      mocap_pos=jnp.broadcast_to(mpos[k], d.mocap_pos.shape),
                      mocap_quat=jnp.broadcast_to(mquat[k], d.mocap_quat.shape))  # fmt: skip
        d = jax.lax.fori_loop(0, substeps, lambda _, x: step(x), d)
        return d, d.qpos[:, int(traj["obj_z_adr"])]

    run = jax.jit(lambda d: jax.lax.scan(control_step, d, jnp.arange(frames)))
    t0 = time.perf_counter()
    jax.block_until_ready(run(batch))
    compile_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    _, z = run(batch)
    z = np.asarray(jax.block_until_ready(z))
    wall = time.perf_counter() - t0
    lifted = float(np.mean(z[-1] - z[0] > 0.05))
    return {"worlds": nworld, "compile_s": compile_s, "wall_s": wall,
            "control_steps_per_s": nworld * frames / wall, "lifted": lifted}  # fmt: skip


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else "tendra_warp_bench.npz"
    sizes = [int(n) for n in sys.argv[2:]] or [256, 1024, 4096]
    model, traj = load(path)
    print(f"device: {jax.devices()[0]}  model: {model.nbody} bodies, {model.ntendon} tendons, "
          f"{len(traj['ctrl'])} control steps x {int(traj['substeps'])} substeps")  # fmt: skip
    for n in sizes:
        try:
            r = benchmark(model, traj, n)
        except Exception as e:  # noqa: BLE001  (e.g. out of GPU memory: report and go on)
            print(f"{n:6d} worlds: FAILED {type(e).__name__}: {str(e)[:200]}")
            continue
        print(f"{n:6d} worlds: {r['control_steps_per_s']:10,.0f} control steps/s "
              f"(compile {r['compile_s']:.0f} s, run {r['wall_s']:.1f} s), "
              f"lifted {r['lifted']:.0%}")  # fmt: skip


if __name__ == "__main__":
    main()
