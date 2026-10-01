"""Train the Tendra V1 hand to grasp and lift objects by itself (reinforcement learning, PPO).

    uv run python sim/train_grasp.py                         # laptop defaults, runs/grasp
    uv run python sim/train_grasp.py --steps 2e6 --run runs/try1
    uv run python sim/train_grasp.py --run runs/grasp        # same folder again = resume
    uv run python sim/train_grasp.py --smoke                 # 30 s check that everything runs

Watch it learn: progress.csv in the run folder (success, lift height, reward terms), and try the
current policy any time with `sim/eval_grasp.py --run <folder> --watch`.

How it works (details in research/ai/grasp-rl.md):
the hand moves its wrist and shapes its fingers through human-like synergies; rewards favour a
palm-first approach, thumb opposition, lifting and holding, and punish knocking the object,
jerky motion and needless force. Episodes sometimes start from states of successful scripted
grasps (or your own teleop demos, --dataset), less often as it improves; objects are added in
stages (cylinder, + cube, + ball).

Laptop speed: ~1,000-1,500 steps/s with 4 workers, so 5 M steps take ~1-1.5 h. For big runs use
a cloud machine with many CPU cores (--workers 32 --envs 256).
"""

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def main() -> None:
    from tendra.grasp_env import GraspEnvConfig
    from tendra.rl.ppo import PPOConfig
    from tendra.rl.train import TrainConfig, train

    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--run", type=Path, default=REPO / "runs" / "grasp", help="run folder (resumes)")
    p.add_argument("--steps", type=float, default=20e6, help="total env steps")
    p.add_argument("--envs", type=int, default=32)
    p.add_argument("--workers", type=int, default=4, help="processes (0 = single process)")
    p.add_argument("--horizon", type=int, default=64)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--grasp", choices=["power", "precision", "any"], default="power")
    p.add_argument("--objects", nargs="+", help="skip the curriculum: always these objects")
    p.add_argument("--demos", type=int, default=60, help="scripted demo grasps (0 = none)")
    p.add_argument("--dataset", action="append", default=[], help="teleop dataset as demos")
    p.add_argument("--synergies", type=Path, help=".npz synergies (e.g. from fit_synergies.py)")
    p.add_argument("--no-randomize", action="store_true", help="no domain randomisation")
    p.add_argument("--device", default="cpu", help="cpu or cuda")
    p.add_argument("--smoke", action="store_true", help="tiny run to check the setup")
    args = p.parse_args()

    env = GraspEnvConfig(grasp_type=args.grasp, randomize=not args.no_randomize,
                         synergies=str(args.synergies) if args.synergies else None)  # fmt: skip
    cfg = TrainConfig(run_dir=args.run, total_steps=int(args.steps), num_envs=args.envs,
                      workers=args.workers, horizon=args.horizon, seed=args.seed, env=env,
                      ppo=PPOConfig(device=args.device), scripted_demos=args.demos,
                      datasets=args.dataset)  # fmt: skip
    if args.objects:
        cfg.stages = [args.objects]
    if args.smoke:
        cfg.run_dir = REPO / "runs" / "smoke"
        cfg.total_steps, cfg.num_envs, cfg.horizon, cfg.scripted_demos = 2048, 8, 32, 3
        cfg.workers = min(args.workers, 2)
    print(f"training in {cfg.run_dir} ({cfg.num_envs} envs, {cfg.workers} workers)")
    try:
        train(cfg)
    except KeyboardInterrupt:
        print("\nstopped; run the same command again to resume from latest.pt")
        sys.exit(130)


if __name__ == "__main__":
    main()
