"""Learn hand synergies (eigengrasps) from your own teleop demos, for RL training.

    uv run python sim/fit_synergies.py grasp                     # dataset name or folder
    uv run python sim/fit_synergies.py grasp other --k 6 --out runs/my_synergies.npz
    uv run python sim/train_grasp.py --synergies runs/my_synergies.npz

PCA over every recorded finger posture: the mean becomes the rest posture, the top k components
the synergy directions. The policy then moves the Tendra hand in *your* hand's main patterns.
Prints how much of your posture variance k synergies explain (Santello 1998: 2 explain ~80% for
human grasps).
"""

import argparse
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def main() -> None:
    from tendra.dataset import default_dataset_root
    from tendra.synergy import explained_variance, fit_synergies, postures_from_datasets

    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("datasets", nargs="+", help="dataset names (~/tendra-data/datasets) or folders")
    p.add_argument("--k", type=int, default=6, help="number of synergies")
    p.add_argument("--out", type=Path, default=REPO / "runs" / "synergies.npz")
    args = p.parse_args()

    roots = [Path(d) if Path(d).exists() else default_dataset_root(d) for d in args.datasets]
    q = postures_from_datasets(roots)
    print(f"{len(q):,} postures from {len(roots)} dataset(s)")
    for k in (1, 2, 3, 4, 6, 8):
        print(f"  {k} synergies explain {explained_variance(q, k):.0%}")
    syn = fit_synergies(q, k=args.k)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"saved {args.k} synergies to {syn.save(args.out)}")


if __name__ == "__main__":
    main()
