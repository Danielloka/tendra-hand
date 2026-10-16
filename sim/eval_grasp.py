"""Evaluate, watch, film or record a trained grasp policy (from sim/train_grasp.py).

    uv run python sim/eval_grasp.py --run runs/grasp                 # success table
    uv run python sim/eval_grasp.py --run runs/grasp --watch         # live window (Q quits, N next)
    uv run python sim/eval_grasp.py --run runs/grasp --video grasp.mp4 --episodes 3
    uv run python sim/eval_grasp.py --run runs/grasp --record rl_grasps --episodes 50

A policy trained on the GPU (Brax, `tendra.gpu.train_brax`) runs in this CPU environment too, as a
sim-to-sim check; give its params file and the bundle it was trained with (the bundle holds the
environment config and the synergies), every other option works as above:

    uv run python sim/eval_grasp.py --brax params_final.pkl --bundle tendra_gpu.npz --watch

The evaluation uses a fixed set of seeds (the same spawns every time) and the nominal model, so
numbers from different runs compare fairly. Besides success (with its 95 % Wilson interval) it
reports how *calm* and human-like the grasps were (research/ai/smooth-control.md):
- `rate`, `jerk`: mean squared first and second difference of the action (wrist dims weighted
  as in the reward);
- `wrist`, `finger`: how much the wrist (mm) and the fingers (deg) move per step while holding
  the object at lift height (includes deliberate motion such as lifting higher), and `wshake`,
  `fshake`: the back-and-forth part of it (second difference): the "shaking" number, ~0 for a
  calm hold;
- `fft`: the CAPS smoothness score of the actions (Mysore et al. 2021: FFT amplitude weighted
  by frequency; ~ the amplitude of step-to-step flicker; lower = smoother);
- time to lift, effort (servo force), and the per-joint residual's size.

A Brax policy runs as its run was training (`tendra.gpu.policy.trained_settings`: the objects and
the object position noise at the curriculum's level), unless `--objects` / `--obs-noise` say
otherwise; extra noise the policy never saw makes any policy shake.

`--record` saves the successful rollouts as a teleop-format dataset (`tendra.dataset`, same
state/action as grasp_teleop.py): the RL policy becomes a demonstrator whose grasps can train an
imitation (vision) policy with LeRobot, the "privileged teacher -> vision student" recipe.
"""

import argparse
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
EVAL_SEED = 12345
EVAL_WRIST_WEIGHT = 4.0  # the wrist dims' weight in the `rate` / `jerk` metrics (as calm v2)


def find_checkpoint(run: Path | None, checkpoint: Path | None) -> Path:
    if checkpoint:
        return checkpoint
    for name in ("best.pt", "latest.pt"):
        if (run / name).exists():
            return run / name
    raise SystemExit(f"no best.pt or latest.pt in {run}")


def load_brax(
    path: Path,
    bundle: Path | None,
    hands: str | None,
    randomize: bool,
    obs_noise: float | None = None,
):
    """A Brax policy (`.pkl` / `.npz`) and a CPU env configured like the GPU env it came from
    (`obs_noise` in m; None = as the run was training, else the bundle's)."""
    import json

    from tendra.gpu.policy import BraxPolicy
    from tendra.grasp_env import GraspEnv, GraspEnvConfig, config_from_dict
    from tendra.synergy import Synergies

    policy = BraxPolicy.load(path)
    syn = None
    if bundle:
        with np.load(bundle) as b:
            meta = json.loads(bytes(b["meta"]).decode())
            syn = Synergies(b["syn_rest"], b["syn_basis"], tuple(meta["synergies"]))
        cfg = config_from_dict({**meta["env_config"], "rewards": meta.get("rewards", {})})
    else:
        print("no --bundle: default environment config and synergies (hope they match)")
        cfg = GraspEnvConfig()
    cfg.randomize, cfg.demo_prob = randomize, 0.0
    from tendra.gpu.policy import trained_settings

    trained = trained_settings(path)
    if not trained:
        print("warning: no curriculum.json / config.json next to the policy: using the bundle's "
              "settings (wrong for a policy older than the bundle)")  # fmt: skip
    # The settings the policy trained with (an old policy + a newer bundle: the old ones).
    for key, value in trained.get("env", {}).items():
        if hasattr(cfg, key):
            setattr(cfg, key, tuple(value) if isinstance(value, list) else value)
    if obs_noise is None:
        obs_noise = trained.get("obs_noise")
    if obs_noise is not None:
        cfg.obs_noise = obs_noise
    if hands:
        cfg.hands = hands
    env = GraspEnv(cfg)
    if syn is not None:
        env.syn = syn
    obs_dim = env.reset()[0].shape[-1]
    if obs_dim != policy.obs_dim or env.action_dim != policy.act_dim:
        raise SystemExit(
            f"policy: {policy.obs_dim} observations, {policy.act_dim} actions; this environment: "
            f"{obs_dim} and {env.action_dim}. Pass the --bundle it was trained with."
        )
    return policy, env


def smoothness_score(actions: np.ndarray, fs: float) -> float:
    """CAPS's smoothness score Sm = 2 / (n fs) * sum_i M_i f_i (Mysore et al. 2021; M_i = |FFT|
    at frequency f_i), averaged over the action dims. It weighs amplitude by frequency: a
    constant action scores 0, step-to-step flicker of amplitude x scores ~x, a slow motion of
    amplitude A at frequency f only ~2 A f / fs. Lower = smoother."""
    n = len(actions)
    if n < 4:
        return float("nan")
    amp = np.abs(np.fft.rfft(actions - actions.mean(axis=0), axis=0))
    freq = np.fft.rfftfreq(n, 1 / fs)
    return float(np.mean(2 / (n * fs) * (amp * freq[:, None]).sum(axis=0)))


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95 % Wilson score interval of a success rate k / n."""
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def hold_motion(wrists: np.ndarray, fingers: np.ndarray, holding: np.ndarray) -> dict:
    """Motion per step while holding at lift height. `wrist` (mm) / `finger` (deg): how far they
    move per step (includes deliberate motion, e.g. lifting higher). `wshake` / `fshake`: the
    second difference |p_t - 2 p_t-1 + p_t-2|, zero for steady motion: the shaking itself."""
    nan = float("nan")
    out = {"wrist": nan, "finger": nan, "wshake": nan, "fshake": nan}
    one = holding[1:] & holding[:-1]
    two = one[1:] & one[:-1]
    if one.any():
        out["wrist"] = float(1000 * np.linalg.norm(np.diff(wrists, axis=0), axis=1)[one].mean())
        out["finger"] = float(np.degrees(np.abs(np.diff(fingers, axis=0))).mean(axis=1)[one].mean())
    if two.any():
        d2w = wrists[2:] - 2 * wrists[1:-1] + wrists[:-2]
        d2f = fingers[2:] - 2 * fingers[1:-1] + fingers[:-2]
        out["wshake"] = float(1000 * np.linalg.norm(d2w, axis=1)[two].mean())
        out["fshake"] = float(np.degrees(np.abs(d2f)).mean(axis=1)[two].mean())
    return out


def run_episode(env, policy, rng, obj, stochastic, on_step=None):
    obs, _ = env.reset(obj=obj)
    w = env.config.rewards
    stats = {"effort": [], "residual": []}
    actions, wrists, fingers, holding = [], [], [], []
    done = False
    while not done:
        a = np.clip(policy(obs, deterministic=not stochastic, rng=rng), -1.0, 1.0)
        (obs, _), _, term, trunc, info = env.step(a)
        actions.append(a)
        t = info["terms"]
        stats["effort"].append(-t.get("effort", 0.0) / w.effort)
        stats["residual"].append(-t.get("residual", 0.0) / w.residual)
        wrists.append(env.scene.wrist_pose()[0].copy())
        fingers.append(env.scene.finger_positions().copy())
        holding.append(env.hold > 0)  # held at lift height: should be calm
        if on_step and on_step(env) is False:
            return None
        done = term or trunc
    ep = info["episode"]
    ep.update({k: float(np.mean(v)) for k, v in stats.items()})
    acts = np.array(actions)
    zero = np.zeros((2, env.action_dim))
    prev = np.vstack([zero[:1], acts[:-1]])
    prev2 = np.vstack([zero, acts])[: len(acts)]
    wgt = np.ones(env.action_dim)
    wgt[:6] = EVAL_WRIST_WEIGHT  # fixed, so policies with different reward weights compare
    ep["rate"] = float(np.mean(wgt * (acts - prev) ** 2))
    ep["jerk"] = float(np.mean(wgt * (acts - 2 * prev + prev2) ** 2))
    ep["fft"] = smoothness_score(acts, 1 / env.dt)
    ep.update(hold_motion(np.array(wrists), np.array(fingers), np.array(holding)))
    ep["time"] = ep["length"] * env.dt
    return ep


def main() -> int:

    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--run", type=Path, default=REPO / "runs" / "grasp")
    p.add_argument("--brax", type=Path, help="a Brax params .pkl or .npz from GPU training")
    p.add_argument("--bundle", type=Path, help="with --brax: the GPU bundle (.npz) it trained on")
    p.add_argument("--hands", choices=["right", "left", "any"], help="default: as trained")
    p.add_argument("--checkpoint", type=Path, help="a .pt file (default: best.pt, else latest.pt)")
    p.add_argument("--episodes", type=int, default=20, help="per object")
    p.add_argument(
        "--objects", nargs="+", help="default: the objects it trained on (Brax), else every object"
    )
    p.add_argument(
        "--obs-noise",
        type=float,
        help="object position noise in m (default: as trained (Brax), else 0.003)",
    )
    p.add_argument("--grasp", choices=["power", "precision"], help="default: as trained")
    p.add_argument("--stochastic", action="store_true", help="sample actions (default: mean)")
    p.add_argument("--randomize", action="store_true", help="randomised physics, as in training")
    p.add_argument("--watch", action="store_true", help="show it live in a window")
    p.add_argument("--video", type=Path, help="write an MP4 of the episodes")
    p.add_argument("--record", help="save successful episodes as a dataset with this name")
    p.add_argument("--seed", type=int, default=EVAL_SEED)
    p.add_argument(
        "--calm",
        nargs="?",
        const="light",
        choices=["light", "light+latch", "full"],
        help="deploy-time calm layer (tendra.calm presets) between the policy and the servos / "
        "wrist target; bare --calm = light",
    )
    args = p.parse_args()

    if args.brax:
        path = args.brax
        from tendra.gpu.policy import trained_settings

        policy, env = load_brax(path, args.bundle, args.hands, args.randomize, args.obs_noise)
        trained = [o for o in trained_settings(path).get("objects", [])
                   if o in env.scene.config.objects]  # fmt: skip
        args.objects = args.objects or trained or None
        header = (f"{path} (Brax, {policy.steps or '?'} steps, hands: {env.config.hands}, "
                  f"object noise {env.config.obs_noise * 1000:.1f} mm)")  # fmt: skip
    else:
        import torch
        from tendra.rl.ppo import Policy
        from tendra.rl.train import load_env_for

        path = find_checkpoint(args.run, args.checkpoint)
        ck = torch.load(path, map_location="cpu", weights_only=False)
        policy = Policy(ck)
        overrides = {"hands": args.hands} if args.hands else {}
        if args.obs_noise is not None:
            overrides["obs_noise"] = args.obs_noise
        env = load_env_for(ck, randomize=args.randomize, demo_prob=0.0, **overrides)
        header = (f"{path} ({ck['steps']:,} steps, curriculum stage {ck['curriculum']['stage']}"
                  f"hands: {env.config.hands})")  # fmt: skip
    objects = args.objects or list(env.scene.config.objects)
    env.configure(objects=objects)
    if args.grasp:
        env.configure(grasp_type=args.grasp)
    print(header)

    on_step, frames, recorder = None, [], None
    if args.watch or args.video:
        import cv2
        from tendra.hand_view import HandView

        view = HandView(env.model, 640, 480, mirror=False, camera="view")
        window = "Tendra grasp policy (Q quit, N next)"

        def on_step(e):
            img = view.render(e.data)
            if args.video:
                frames.append(img)
            if args.watch:
                cv2.imshow(window, img)
                key = cv2.waitKey(max(1, int(e.dt * 1000))) & 0xFF
                if key in (ord("q"), 27):
                    raise KeyboardInterrupt
                if key == ord("n"):
                    return False
            return True

    if args.record:
        from tendra.arm import ARM_JOINTS
        from tendra.dataset import EpisodeRecorder, default_dataset_root

        # The working hand's 16 servos, its wrist pose, then its 7 arm joints (the IK's targets
        # as the action); the side goes in each episode's extra.
        names = [*env.scene.spec.joint_names, "wrist_x", "wrist_y", "wrist_z",
                 "wrist_qw", "wrist_qx", "wrist_qy", "wrist_qz", *ARM_JOINTS]  # fmt: skip
        recorder = EpisodeRecorder(default_dataset_root(args.record), env.model,
                                   fps=round(1 / env.dt), state_names=names,
                                   action_names=[f"{n}_target" for n in names],
                                   info={"hand": "v1", "scene": "grasp", "source": str(path)})  # fmt: skip
        record_prev = on_step

        def on_step(e):
            s = e.scene
            pos, quat = s.wrist_pose()
            tpos, tquat = s.wrist_target()
            opos, oquat = s.object_pose()
            recorder.add_frame(e.data,
                               np.concatenate([s.finger_positions(), pos, quat, s.arm_positions()]),
                               np.concatenate([s.finger_targets(), tpos, tquat, s.arm_targets()]),
                               {"object.pos": opos, "object.quat": oquat})  # fmt: skip
            return record_prev(e) if record_prev else True

    rng = np.random.default_rng(args.seed)
    results = []
    calm_ctx = None
    if args.calm:
        from tendra.calm import calm_applied

        calm_ctx = calm_applied(env, args.calm)
        calm_ctx.__enter__()
        print(f"calm layer on: {args.calm}")
    try:
        for obj in objects:
            env.rng = np.random.default_rng(args.seed + objects.index(obj))  # fixed spawns
            for _ in range(args.episodes):
                if recorder:
                    recorder.start_episode(
                        f"pick up the {obj}", extra={"object": obj, "side": env.side}
                    )
                ep = run_episode(env, policy, rng, obj, args.stochastic, on_step)
                if recorder:
                    if ep and ep["success"]:
                        recorder.end_episode(success=True)
                    else:
                        recorder.discard_episode()
                if ep:
                    results.append(ep)
                    if args.watch:
                        print(
                            f"{obj}: {'success' if ep['success'] else 'fail'} in {ep['time']:.1f} s"
                        )
    except KeyboardInterrupt:
        pass
    finally:
        if calm_ctx:
            calm_ctx.__exit__(None, None, None)

    if args.video and frames:
        import cv2

        h, w = frames[0].shape[:2]
        out = cv2.VideoWriter(str(args.video), cv2.VideoWriter_fourcc(*"mp4v"),
                              round(1 / env.dt), (w, h))  # fmt: skip
        for f in frames:
            out.write(f)
        out.release()
        print(f"video: {args.video}")

    print(f"\n{'object':10s} {'n':>3s} {'success':>8s} {'95% CI':>11s} {'lift cm':>8s} "
          f"{'time s':>7s} {'rate':>7s} {'jerk':>7s} {'fft':>7s} {'wrist mm':>9s} "
          f"{'wshake mm':>10s} {'finger deg':>11s} {'fshake deg':>11s} {'effort':>7s} "
          f"{'resid':>7s}")  # fmt: skip

    def mean(eps: list[dict], key: str) -> float:
        vals = [e[key] for e in eps if np.isfinite(e[key])]
        return float(np.mean(vals)) if vals else float("nan")

    for obj in objects:
        eps = [e for e in results if e["object"] == obj]
        if not eps:
            continue
        ok = [e for e in eps if e["success"]]
        lo, hi = wilson(len(ok), len(eps))
        ci = f"{lo:.0%}-{hi:.0%}"
        print(f"{obj:10s} {len(eps):3d} {len(ok) / len(eps):8.0%} {ci:>11s} "
              f"{mean(eps, 'max_height') * 100:8.1f} {mean(ok, 'time'):7.2f} "
              f"{mean(eps, 'rate'):7.3f} {mean(eps, 'jerk'):7.3f} {mean(eps, 'fft'):7.3f} "
              f"{mean(eps, 'wrist'):9.2f} {mean(eps, 'wshake'):10.2f} "
              f"{mean(eps, 'finger'):11.2f} {mean(eps, 'fshake'):11.2f} "
              f"{mean(eps, 'effort'):7.3f} {mean(eps, 'residual'):7.3f}")  # fmt: skip
    print(
        "while holding at lift height: wrist mm / finger deg = motion per step (incl. lifting "
        "higher), wshake / fshake = its back-and-forth part (shaking; calm = ~0); "
        "fft: CAPS smoothness score of the actions (lower = smoother)"
    )
    if recorder:
        print(f"dataset: {recorder.root} ({recorder.num_episodes} episodes)")
    time.sleep(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
