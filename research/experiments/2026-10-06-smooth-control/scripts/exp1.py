"""Jitter experiments on the CPU GraspEnv (read-only)."""
import numpy as np
from tendra.grasp_env import GraspEnv, GraspEnvConfig

cfg = GraspEnvConfig(randomize=False, hands="right", objects=("cylinder",))
env = GraspEnv(cfg, seed=0)
s, m, d = env.scene, env.model, env.data
k = env.syn.k
print("dt", env.dt, "substeps", env.substeps, "action_dim", env.action_dim, "obs", env.obs_dim)

def meas():
    wv = s.wrist_velocity()
    arm = s.arm_positions()
    return dict(wlin=np.linalg.norm(wv[:3]), wang=np.linalg.norm(wv[3:]),
                armv=np.linalg.norm(s.arm_velocities()),
                fv=np.linalg.norm(d.qvel[[m.jnt_dofadr[m.joint(f'right_{n}').id] for n in ['index_pip','index_mcp_flex','middle_pip','thumb_ip','thumb_cmc_rot']]]),
                wpos=s.wrist_pose()[0].copy(), obj=s.object_pose()[0].copy())

def run(policy, n=104, label=""):
    rows=[]
    obs,_ = env.reset(obj="cylinder", side="right")
    for t in range(n):
        a = policy(t, obs)
        (obs,_), r, term, trunc, info = env.step(a)
        rows.append(meas())
        if term: break
    W = np.array([r_['wpos'] for r_ in rows]); O=np.array([r_['obj'] for r_ in rows])
    late = slice(len(rows)//2, None)
    dW = np.diff(W[late],axis=0)*1000
    print(f"{label:38s} steps {len(rows)} | late: wrist lin {np.mean([r_['wlin'] for r_ in rows[late]])*1000:6.2f} mm/s "
          f"ang {np.mean([r_['wang'] for r_ in rows[late]]):6.3f} rad/s arm|v| {np.mean([r_['armv'] for r_ in rows[late]]):6.3f} "
          f"finger|v| {np.mean([r_['fv'] for r_ in rows[late]]):6.3f} | wrist step-to-step mm {np.linalg.norm(dW,axis=1).mean():.3f} "
          f"| wrist 2nd diff mm {np.linalg.norm(np.diff(W[late],2,axis=0)*1000,axis=1).mean():.3f}")
    return rows

zero = np.zeros(env.action_dim)
run(lambda t,o: zero, label="A zero action (open hand, at home)")

# B: noisy wrist actions to calibrate visibility
rng = np.random.default_rng(0)
for sig in (0.05, 0.1, 0.3):
    a_fn = lambda t,o,sig=sig: np.concatenate([rng.normal(0,sig,6), np.zeros(env.action_dim-6)])
    run(a_fn, label=f"B iid wrist action noise std {sig}")
for sig in (0.05, 0.1, 0.3):
    a_fn = lambda t,o,sig=sig: np.concatenate([np.zeros(6), rng.normal(0,sig,env.action_dim-6)])
    run(a_fn, label=f"C iid finger action noise std {sig}")

# D: wrist step response: one step of a=+1 in x then zero; track wrist x
obs,_ = env.reset(obj="cylinder", side="right")
for _ in range(10): env.step(zero)
x0 = s.wrist_pose()[0].copy(); tgt0 = s.wrist_target()[0].copy()
a = zero.copy(); a[0]=1.0
env.step(a)
xs=[]
for _ in range(20):
    env.step(zero); xs.append((s.wrist_pose()[0]-x0)*1000)
xs=np.array(xs)
print("D target moved (mm):", np.round((s.wrist_target()[0]-tgt0)*1000,2))
print("D wrist response x (mm) per 48ms step after the command:", np.round(xs[:,0],2))
print("D wrist response |off-axis| (mm):", np.round(np.linalg.norm(xs[:,1:],axis=1),2))
# finer: substep-resolution response for a 14mm target step
obs,_ = env.reset(obj="cylinder", side="right")
for _ in range(10): env.step(zero)
p,q = s.wrist_target(); x0 = s.wrist_pose()[0].copy()
s.set_wrist_target(p + np.array([0.0144,0,0]), q)
tr=[]
for i in range(150):
    s.step(0.004); tr.append((s.wrist_pose()[0]-x0)[0]*1000)
tr=np.array(tr)
print(f"D2 14.4 mm target step: peak {tr.max():.2f} mm at {tr.argmax()*4} ms, overshoot {100*(tr.max()-14.4)/14.4:.1f}%, value at 48ms {tr[11]:.2f}, 96ms {tr[23]:.2f}, 200ms {tr[49]:.2f}, 600ms {tr[-1]:.2f}")
# rotation step: 0.096 rad about z (one full turn action)
obs,_ = env.reset(obj="cylinder", side="right")
for _ in range(10): env.step(zero)
p,q = s.wrist_target(); R0 = s.wrist_frame()[1].copy()
ang=0.096; dq=np.array([np.cos(ang/2),0,0,np.sin(ang/2)])
import mujoco

qq=np.zeros(4); mujoco.mju_mulQuat(qq,dq,q); s.set_wrist_target(p,qq)
tr=[]
for i in range(150):
    s.step(0.004); R=s.wrist_frame()[1]; Rr=R@R0.T; tr.append(np.degrees(np.arccos(np.clip((np.trace(Rr)-1)/2,-1,1))))
tr=np.array(tr)
print(f"D3 5.5 deg target turn: peak {tr.max():.2f} deg at {tr.argmax()*4} ms, at 48ms {tr[11]:.2f}, 200ms {tr[49]:.2f}, end {tr[-1]:.2f}")
