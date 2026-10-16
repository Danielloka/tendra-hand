"""Measure a Brax grasp policy's jitter on the CPU env (read-only).

    uv run python jitter_probe.py POLICY.npz|params_best.pkl BUNDLE.npz [--episodes 5] [--objects cylinder]

Prints, per obs-noise setting (0 = as trained at dr_level 0, 0.003 = eval_grasp/rollout_video default):
 - mean |a_t - a_{t-1}| per action group (wrist pos, wrist turn, synergies, residual), all steps and
   while held; wrist step-to-step motion (mm); share of saturated actions (|a| > 0.95);
 - FFT peak frequency of the actions (4-5 Hz = arm-loop limit cycle, ~10.4 Hz = period-2 flicker);
 - the policy's own exploration std (softplus(raw)+0.001);
 - d a_t / d prev_action (finite differences, at held states): spectral radius > 1 with a
   negative eigenvalue = self-excited period-2 oscillation through the prev_action input.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "sim"))
from eval_grasp import load_brax

p = argparse.ArgumentParser()
p.add_argument("policy"); p.add_argument("bundle")
p.add_argument("--episodes", type=int, default=5)
p.add_argument("--objects", nargs="+", default=None)
p.add_argument("--hands", default="right")
args = p.parse_args()

policy, env = load_brax(Path(args.policy), Path(args.bundle), args.hands, randomize=False)
k = env.syn.k
groups = {"wrist_pos": slice(0, 3), "wrist_turn": slice(3, 6), "synergy": slice(6, 6 + k),
          "residual": slice(6 + k, None)}
# where prev_action sits in the 141-vector: tail = rot6d(6) obj_vel(6) touch(6) onehot(3) size(3) grasp(2) prev(28) t,h(2)
A = env.action_dim
prev_lo = env.obs_dim - 2 - A
objects = args.objects or list(env.scene.config.objects)

for noise in (0.0, env.config.obs_noise if env.config.obs_noise > 0 else 0.003):
    env.config.obs_noise = noise
    acts, held, wrist, stds, jac_rad = [], [], [], [], []
    for obj in objects:
        env.rng = np.random.default_rng(123 + objects.index(obj))
        for ep in range(args.episodes):
            obs, _ = env.reset(obj=obj)
            done = False
            while not done:
                a = policy(obs)
                loc, raw = np.split(policy._logits(obs), 2)
                stds.append(np.logaddexp(raw, 0) + 0.001)
                (obs, _), r, term, trunc, info = env.step(a)
                h = info["terms"].get("held", 0) > 0
                acts.append(a); held.append(h); wrist.append(env.scene.wrist_pose()[0].copy())
                if h and len(jac_rad) < 20:
                    J = np.zeros((A, A)); eps = 1e-3
                    base = np.arctanh(np.clip(policy(obs), -0.999999, 0.999999))
                    for i in range(A):
                        o2 = obs.copy(); o2[prev_lo + i] += eps
                        J[:, i] = (np.arctanh(np.clip(policy(o2), -0.999999, 0.999999)) - base) / eps
                    # include the tanh slope at the operating point
                    J = (1 - np.tanh(base) ** 2)[:, None] * J
                    ev = np.linalg.eigvals(J)
                    jac_rad.append((np.abs(ev).max(), ev[np.abs(ev).argmax()]))
                done = term or trunc
            acts.append(np.full(A, np.nan)); held.append(False); wrist.append(np.full(3, np.nan))
    a = np.array(acts); hmask = np.array(held); W = np.array(wrist)
    da = np.abs(np.diff(a, axis=0)); dW = np.linalg.norm(np.diff(W, axis=0), axis=1) * 1000
    hm = hmask[1:] & hmask[:-1]
    print(f"\n=== obs noise {noise*1000:.1f} mm ===")
    for g, sl in groups.items():
        print(f"{g:10s} mean|da| all {np.nanmean(da[:, sl]):.3f} held {np.nanmean(da[hm][:, sl]) if hm.any() else float('nan'):.3f}"
              f"  saturated {np.nanmean(np.abs(a[:, sl]) > 0.95):.2f}  policy std {np.mean(np.array(stds)[:, sl]):.3f}")
    print(f"wrist motion per step: all {np.nanmean(dW):.2f} mm, held {np.nanmean(dW[hm]) if hm.any() else float('nan'):.2f} mm")
    seg = a[~np.isnan(a[:, 0])]
    for g in ("wrist_pos", "synergy"):
        x = seg[:, groups[g]][:, 0]
        sp = np.abs(np.fft.rfft(x - x.mean())); f = np.fft.rfftfreq(len(x), env.dt)
        print(f"FFT peak {g}[0]: {f[sp[1:].argmax() + 1]:.1f} Hz (Nyquist {0.5 / env.dt:.1f})")
    if jac_rad:
        r = np.array([j[0] for j in jac_rad])
        print(f"d a/d prev_action at held states: spectral radius mean {r.mean():.2f} max {r.max():.2f};"
              f" dominant eigenvalue example {jac_rad[0][1]:.2f}")
