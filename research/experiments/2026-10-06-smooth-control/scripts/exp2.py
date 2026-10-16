import numpy as np
from tendra.grasp_env import GraspEnv, GraspEnvConfig

cfg = GraspEnvConfig(randomize=False, hands="right", objects=("cylinder",))
env = GraspEnv(cfg, seed=0)
s, m, d = env.scene, env.model, env.data
k = env.syn.k
demos = env.record_scripted_demos(3, seed=1)
print("demos", len(demos), [len(x) for x in demos])
dm = demos[0]
env.set_demos([dm]); env.configure(demo_prob=1.0)
act = s.sides["right"].act
# pick late frames (holding)
for frac in (0.8, 0.95):
    # force a specific frame: monkeypatch rng.integers
    idx = int(frac*len(dm))
    class R:
        def __init__(s_, r): s_.r=r
        def __getattr__(s_, n): return getattr(s_.r, n)
        def integers(s_, n, *a, **kw): return idx if n==len(dm) else s_.r.integers(n,*a,**kw)
        def random(s_): return 0.0
    env.rng = R(np.random.default_rng(0))
    obs,_ = env.reset(obj="cylinder", side="right")
    env.rng = np.random.default_rng(0)
    q = d.ctrl[act].copy()
    syn = env.syn.project(q)
    resid = np.clip((q - env.syn.posture(syn))/cfg.residual_scale, -1, 1)
    a = np.concatenate([np.zeros(6), syn, resid])
    print(f"frame {idx}: synergy coeffs {np.round(syn,2)}  max|resid| {np.abs(resid).max():.2f}; target err after mapping {np.abs(env.syn.posture(np.clip(syn,-1,1))+cfg.residual_scale*resid - q).max():.4f} rad")
    W=[];O=[];F=[]; hs=[]
    for t in range(60):
        (obs,_), r, term, trunc, info = env.step(a)
        W.append(s.wrist_pose()[0].copy()); O.append(s.object_pose()[0].copy()); F.append(s.finger_positions())
        hs.append(info["terms"].get("held",0))
        if term: print("terminated", t); break
    W,O,F=map(np.array,(W,O,F))
    late=slice(20,None)
    print(f"  held frac {np.mean(np.array(hs)>0):.2f}; late wrist step mm {np.linalg.norm(np.diff(W[late],axis=0),axis=1).mean()*1000:.3f}, obj step mm {np.linalg.norm(np.diff(O[late],axis=0),axis=1).mean()*1000:.3f}, finger step rad {np.abs(np.diff(F[late],axis=0)).mean():.5f}; obj z drift mm {(O[-1,2]-O[20,2])*1000:.2f}")
