"""A proportional 'policy' on the observed wrist position: where does the wrist loop start to ring?"""
import numpy as np
from tendra.grasp_env import GraspEnv, GraspEnvConfig

cfg = GraspEnvConfig(randomize=False, hands="right", objects=("cylinder",))
env = GraspEnv(cfg, seed=0); s = env.scene
step = cfg.wrist_speed * env.dt
rng = np.random.default_rng(0)
zero = np.zeros(env.action_dim)
for use_vel in (False, True):
  for g in (0.3, 0.6, 1.0, 1.5):
    for noise in (0.0, 0.003):
        env.reset(obj="cylinder", side="right")
        for _ in range(5): env.step(zero)
        goal = s.wrist_pose()[0] + np.array([0.03, 0.0, 0.0])
        xs, acts = [], []
        for t in range(104):
            w = s.wrist_pose()[0] + rng.normal(0, noise, 3)
            v = s.wrist_velocity()[:3]
            err = goal - w
            if use_vel:
                err = err - 0.05 * v  # derivative term (5 ms*... lookahead 50 ms)
            a = zero.copy(); a[0:3] = np.clip(g * err / step, -1, 1)
            env.step(a); xs.append(s.wrist_pose()[0].copy()); acts.append(a[0:3])
        xs, acts = np.array(xs), np.array(acts)
        late = xs[52:]
        e = np.linalg.norm(late - goal, axis=1) * 1000
        da = np.abs(np.diff(acts[52:], axis=0)).mean()
        sp = np.abs(np.fft.rfft(acts[52:, 0] - acts[52:, 0].mean()))
        f = np.fft.rfftfreq(len(acts[52:]), env.dt)
        print(f"{'PD' if use_vel else 'P '} gain {g:3.1f} obs noise {noise*1000:.0f} mm: late |err| mean {e.mean():6.2f} mm, wrist step {np.linalg.norm(np.diff(late,axis=0),axis=1).mean()*1000:6.2f} mm, mean |da| {da:.3f}, FFT peak {f[sp[1:].argmax()+1]:.1f} Hz")
