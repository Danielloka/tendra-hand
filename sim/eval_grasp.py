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
numbers from different runs compare fairly. Besides success it reports how *human-like* and
careful the grasps were: time to lift, smoothness (mean squared action change), effort (servo
force), and how much the fingers used the per-joint residual instead of the synergies.

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


def find_checkpoint(run: Path | None, checkpoint: Path | None) -> Path:
    if checkpoint:
        return checkpoint
    for name in ("best.pt", "latest.pt"):
        if (run / name).exists():
            return run / name
    raise SystemExit(f"no best.pt or latest.pt in {run}")


def load_brax(path: Path, bundle: Path | None, hands: str | None, randomize: bool):
    """A Brax policy (`.pkl` / `.npz`) and a CPU env configured like the GPU env it came from."""
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


def run_episode(env, policy, rng, obj, stochastic, on_step=None):
    obs, _ = env.reset(obj=obj)
    stats = {"rate": [], "effort": [], "residual": []}
    done = False
    while not done:
        a = policy(obs, deterministic=not stochastic, rng=rng)
        (obs, _), _, term, trunc, info = env.step(a)
        t = info["terms"]
        stats["rate"].append(-t.get("action_rate", 0.0) / env.config.rewards.action_rate)
        stats["effort"].append(-t.get("effort", 0.0) / env.config.rewards.effort)
        stats["residual"].append(-t.get("residual", 0.0) / env.config.rewards.residual)
        if on_step and on_step(env) is False:
            return None
        done = term or trunc
    ep = info["episode"]
    ep.update({k: float(np.mean(v)) for k, v in stats.items()})
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
    p.add_argument("--objects", nargs="+", help="default: every object of the scene")
    p.add_argument("--grasp", choices=["power", "precision"], help="default: as trained")
    p.add_argument("--stochastic", action="store_true", help="sample actions (default: mean)")
    p.add_argument("--randomize", action="store_true", help="randomised physics, as in training")
    p.add_argument("--watch", action="store_true", help="show it live in a window")
    p.add_argument("--video", type=Path, help="write an MP4 of the episodes")
    p.add_argument("--record", help="save successful episodes as a dataset with this name")
    p.add_argument("--seed", type=int, default=EVAL_SEED)
    args = p.parse_args()

    if args.brax:
        path = args.brax
        policy, env = load_brax(path, args.bundle, args.hands, args.randomize)
        header = f"{path} (Brax, {policy.steps or '?'} steps, hands: {env.config.hands})"
    else:
        import torch
        from tendra.rl.ppo import Policy
        from tendra.rl.train import load_env_for

        path = find_checkpoint(args.run, args.checkpoint)
        ck = torch.load(path, map_location="cpu", weights_only=False)
        policy = Policy(ck)
        overrides = {"hands": args.hands} if args.hands else {}
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

    if args.video and frames:
        import cv2

        h, w = frames[0].shape[:2]
        out = cv2.VideoWriter(str(args.video), cv2.VideoWriter_fourcc(*"mp4v"),
                              round(1 / env.dt), (w, h))  # fmt: skip
        for f in frames:
            out.write(f)
        out.release()
        print(f"video: {args.video}")

    print(f"\n{'object':10s} {'n':>3s} {'success':>8s} {'lift cm':>8s} {'time s':>7s} "
          f"{'jerk':>7s} {'effort':>7s} {'resid':>7s}")  # fmt: skip
    for obj in objects:
        eps = [e for e in results if e["object"] == obj]
        if not eps:
            continue
        ok = [e for e in eps if e["success"]]
        print(f"{obj:10s} {len(eps):3d} {len(ok) / len(eps):8.0%} "
              f"{np.mean([e['max_height'] for e in eps]) * 100:8.1f} "
              f"{np.mean([e['time'] for e in ok]) if ok else float('nan'):7.2f} "
              f"{np.mean([e['rate'] for e in eps]):7.3f} {np.mean([e['effort'] for e in eps]):7.3f} "
              f"{np.mean([e['residual'] for e in eps]):7.3f}")  # fmt: skip
    if recorder:
        print(f"dataset: {recorder.root} ({recorder.num_episodes} episodes)")
    time.sleep(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
