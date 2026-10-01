"""PPO (Proximal Policy Optimization, Schulman et al. 2017) with the tricks dexterous-hand work
relies on (OpenAI Dactyl 2019, IsaacGymEnvs/rl_games, DexPBT, Playground 2025):

- **Asymmetric actor-critic:** the critic sees privileged simulator state, the actor doesn't.
- **Running observation normalisation** (each input scaled to ~unit variance) and **reward
  scaling** by the running std of the discounted return: rewards of any size train alike.
- **GAE** (generalised advantage estimation) with **time-limit bootstrapping**: an episode cut by
  the clock is not a failure, so its last value is added back.
- **Adaptive learning rate** from the KL divergence (step size stays just below "too big").
- **Bounds loss:** keeps the action mean inside [-1, 1] (the env clips; outside it the gradient
  would vanish).

Plain PyTorch, runs on a laptop CPU; the same code runs on a cloud GPU (`device="cuda"`).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np
import torch
from torch import nn


class RunningNorm:
    """Running mean/variance (parallel Welford), normalises and clips inputs."""

    def __init__(self, dim: int, clip: float = 5.0) -> None:
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4
        self.clip = clip

    def update(self, x: np.ndarray) -> None:
        x = x.reshape(-1, self.mean.size).astype(np.float64)
        n, mean, var = len(x), x.mean(axis=0), x.var(axis=0)
        delta, total = mean - self.mean, self.count + n
        self.mean = self.mean + delta * n / total
        self.var = (self.var * self.count + var * n + delta**2 * self.count * n / total) / total
        self.count = total

    def __call__(self, x: np.ndarray) -> np.ndarray:
        y = (x - self.mean) / np.sqrt(self.var + 1e-8)
        return np.clip(y, -self.clip, self.clip).astype(np.float32)

    def state_dict(self) -> dict:
        return {"mean": self.mean, "var": self.var, "count": self.count, "clip": self.clip}

    def load_state_dict(self, s: dict) -> None:
        self.mean, self.var = np.asarray(s["mean"]), np.asarray(s["var"])
        self.count, self.clip = float(s["count"]), float(s["clip"])


class ReturnScaler:
    """Divides rewards by the running std of each env's discounted return (like SB3's
    VecNormalize): keeps value targets near unit scale while the reward mix changes."""

    def __init__(self, num_envs: int, gamma: float) -> None:
        self.ret = np.zeros(num_envs)
        self.gamma = gamma
        self.norm = RunningNorm(1)

    def __call__(self, reward: np.ndarray, done: np.ndarray) -> np.ndarray:
        self.ret = self.ret * self.gamma + reward
        self.norm.update(self.ret[:, None])
        self.ret[done] = 0.0
        return (reward / np.sqrt(self.norm.var[0] + 1e-8)).astype(np.float32)


def gae(rewards: np.ndarray, values: np.ndarray, dones: np.ndarray, last_value: np.ndarray,
        gamma: float, lam: float) -> np.ndarray:  # fmt: skip
    """Generalised advantage estimation (Schulman et al. 2016) over (T, N) arrays. `dones[t]`
    means the episode ended at step t, so nothing after it counts."""
    adv = np.zeros_like(rewards, dtype=np.float32)
    running = np.zeros_like(last_value, dtype=np.float32)
    for t in reversed(range(len(rewards))):
        next_v = last_value if t == len(rewards) - 1 else values[t + 1]
        not_done = 1.0 - dones[t].astype(np.float32)
        delta = rewards[t] + gamma * next_v * not_done - values[t]
        running = delta + gamma * lam * not_done * running
        adv[t] = running
    return adv


def mlp(sizes: list[int], out_gain: float) -> nn.Sequential:
    layers: list[nn.Module] = []
    for i in range(len(sizes) - 1):
        lin = nn.Linear(sizes[i], sizes[i + 1])
        last = i == len(sizes) - 2
        nn.init.orthogonal_(lin.weight, out_gain if last else np.sqrt(2))
        nn.init.zeros_(lin.bias)
        layers.append(lin)
        if not last:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


@dataclass
class PPOConfig:
    actor_hidden: list[int] = field(default_factory=lambda: [256, 256, 128])
    critic_hidden: list[int] = field(default_factory=lambda: [512, 256, 128])
    init_log_std: float = -0.7  # std ~0.5 of the [-1, 1] action range
    lr: float = 3e-4
    lr_min: float = 1e-5
    lr_max: float = 1e-3
    adaptive_lr: bool = True
    kl_target: float = 0.016
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    epochs: int = 5
    minibatches: int = 4
    value_coef: float = 1.0
    entropy_coef: float = 0.0
    bounds_coef: float = 1e-3
    max_grad_norm: float = 1.0
    device: str = "cpu"


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, critic_dim: int, act_dim: int, cfg: PPOConfig) -> None:
        super().__init__()
        self.actor = mlp([obs_dim, *cfg.actor_hidden, act_dim], out_gain=0.01)
        self.critic = mlp([critic_dim, *cfg.critic_hidden, 1], out_gain=1.0)
        self.log_std = nn.Parameter(torch.full((act_dim,), cfg.init_log_std))

    def dist(self, obs: torch.Tensor) -> torch.distributions.Normal:
        std = self.log_std.clamp(-4.0, 0.5).exp()
        return torch.distributions.Normal(self.actor(obs), std)

    def value(self, critic_obs: torch.Tensor) -> torch.Tensor:
        return self.critic(critic_obs).squeeze(-1)


class PPO:
    def __init__(self, obs_dim: int, critic_dim: int, act_dim: int,
                 cfg: PPOConfig | None = None) -> None:  # fmt: skip
        self.cfg = cfg = cfg or PPOConfig()
        self.device = torch.device(cfg.device)
        self.net = ActorCritic(obs_dim, critic_dim, act_dim, cfg).to(self.device)
        self.opt = torch.optim.Adam(self.net.parameters(), lr=cfg.lr)
        self.lr = cfg.lr
        self.obs_norm = RunningNorm(obs_dim)
        self.critic_norm = RunningNorm(critic_dim)
        self.dims = (obs_dim, critic_dim, act_dim)

    def _t(self, x: np.ndarray) -> torch.Tensor:
        return torch.as_tensor(x, dtype=torch.float32, device=self.device)

    @torch.no_grad()
    def act(self, obs: np.ndarray, critic_obs: np.ndarray, deterministic: bool = False
            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:  # fmt: skip
        """(action, log-prob, value) for a batch; inputs are raw (unnormalised) observations."""
        dist = self.net.dist(self._t(self.obs_norm(obs)))
        a = dist.mean if deterministic else dist.sample()
        logp = dist.log_prob(a).sum(-1)
        v = self.net.value(self._t(self.critic_norm(critic_obs)))
        return a.cpu().numpy(), logp.cpu().numpy(), v.cpu().numpy()

    @torch.no_grad()
    def value(self, critic_obs: np.ndarray) -> np.ndarray:
        return self.net.value(self._t(self.critic_norm(critic_obs))).cpu().numpy()

    def update(self, buf: dict[str, np.ndarray], last_value: np.ndarray) -> dict[str, float]:
        """One PPO update from a rollout. `buf` arrays are (T, N, ...): obs, critic (normalised
        already), actions, logp, values, rewards (scaled, bootstrapped), dones (episode ended)."""
        cfg = self.cfg
        val = buf["values"]
        steps = len(val)
        adv = gae(buf["rewards"], val, buf["dones"], last_value, cfg.gamma, cfg.lam)
        ret = adv + val

        flat = {k: self._t(v.reshape(steps * v.shape[1], *v.shape[2:])) for k, v in
                {"obs": buf["obs"], "critic": buf["critic"], "actions": buf["actions"],
                 "logp": buf["logp"], "values": val, "adv": adv, "ret": ret}.items()}  # fmt: skip
        n = len(flat["adv"])
        mb = max(1, n // cfg.minibatches)
        stats: dict[str, list[float]] = {k: [] for k in
                                         ("policy", "value", "kl", "clipfrac", "entropy")}  # fmt: skip
        for _ in range(cfg.epochs):
            epoch_kl = []
            perm = torch.randperm(n, device=self.device)
            for start in range(0, n, mb):
                idx = perm[start : start + mb]
                a_mb = flat["adv"][idx]
                a_mb = (a_mb - a_mb.mean()) / (a_mb.std() + 1e-8)
                dist = self.net.dist(flat["obs"][idx])
                logp = dist.log_prob(flat["actions"][idx]).sum(-1)
                ratio = (logp - flat["logp"][idx]).exp()
                policy_loss = -torch.min(ratio * a_mb,
                                         ratio.clamp(1 - cfg.clip, 1 + cfg.clip) * a_mb).mean()  # fmt: skip
                v = self.net.value(flat["critic"][idx])
                v_old = flat["values"][idx]
                v_clip = v_old + (v - v_old).clamp(-cfg.clip, cfg.clip)
                r = flat["ret"][idx]
                value_loss = torch.max((v - r) ** 2, (v_clip - r) ** 2).mean()
                entropy = dist.entropy().sum(-1).mean()
                mu = dist.mean
                bounds = (
                    ((mu - 1.1).clamp(min=0) ** 2 + (mu + 1.1).clamp(max=0) ** 2).sum(-1).mean()
                )
                loss = (policy_loss + cfg.value_coef * value_loss - cfg.entropy_coef * entropy
                        + cfg.bounds_coef * bounds)  # fmt: skip
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.net.parameters(), cfg.max_grad_norm)
                self.opt.step()

                with torch.no_grad():
                    log_ratio = logp - flat["logp"][idx]
                    kl = ((ratio - 1) - log_ratio).mean().item()  # low-variance KL estimate
                stats["policy"].append(policy_loss.item())
                stats["value"].append(value_loss.item())
                stats["kl"].append(kl)
                epoch_kl.append(kl)
                stats["clipfrac"].append(((ratio - 1).abs() > cfg.clip).float().mean().item())
                stats["entropy"].append(entropy.item())
            # Adapt the step size once per epoch; stop early if the policy moved too far.
            kl_epoch = float(np.mean(epoch_kl))
            if cfg.adaptive_lr:
                if kl_epoch > 2.0 * cfg.kl_target:
                    self.lr = max(self.lr / 1.5, cfg.lr_min)
                elif kl_epoch < 0.5 * cfg.kl_target:
                    self.lr = min(self.lr * 1.5, cfg.lr_max)
                for g in self.opt.param_groups:
                    g["lr"] = self.lr
            if kl_epoch > 4.0 * cfg.kl_target:
                break
        out = {k: float(np.mean(v)) for k, v in stats.items()}
        out["lr"] = self.lr
        out["std"] = float(self.net.log_std.clamp(-4.0, 0.5).exp().mean().item())
        out["explained_var"] = float(1 - np.var(ret - val) / max(np.var(ret), 1e-8))
        return out

    # ----- saving -----

    def state_dict(self) -> dict:
        return {
            "net": self.net.state_dict(), "opt": self.opt.state_dict(), "lr": self.lr,
            "obs_norm": self.obs_norm.state_dict(), "critic_norm": self.critic_norm.state_dict(),
            "dims": self.dims, "config": asdict(self.cfg),
        }  # fmt: skip

    def load_state_dict(self, s: dict, optimizer: bool = True) -> None:
        self.net.load_state_dict(s["net"])
        self.obs_norm.load_state_dict(s["obs_norm"])
        self.critic_norm.load_state_dict(s["critic_norm"])
        if optimizer:
            self.opt.load_state_dict(s["opt"])
            self.lr = s["lr"]
            for g in self.opt.param_groups:
                g["lr"] = self.lr


class Policy:
    """A trained actor for deployment: raw actor observation in, action in [-1, 1] out.
    Needs only the checkpoint (no critic, no simulator privileges)."""

    def __init__(self, checkpoint: dict) -> None:
        ppo = checkpoint["ppo"]
        cfg = PPOConfig(**ppo["config"])
        obs_dim, critic_dim, act_dim = ppo["dims"]
        net = ActorCritic(obs_dim, critic_dim, act_dim, cfg)
        net.load_state_dict(ppo["net"])
        self.actor = net.actor.eval()
        self.log_std = net.log_std.detach()
        self.norm = RunningNorm(obs_dim)
        self.norm.load_state_dict(ppo["obs_norm"])
        self.obs_dim, self.act_dim = obs_dim, act_dim

    @classmethod
    def load(cls, path: str) -> Policy:
        return cls(torch.load(path, map_location="cpu", weights_only=False))

    @torch.no_grad()
    def __call__(self, obs: np.ndarray, deterministic: bool = True,
                 rng: np.random.Generator | None = None) -> np.ndarray:  # fmt: skip
        mean = self.actor(torch.as_tensor(self.norm(obs), dtype=torch.float32)).numpy()
        if not deterministic:
            rng = rng or np.random.default_rng()
            mean = mean + rng.normal(size=mean.shape) * self.log_std.exp().numpy()
        return np.clip(mean, -1.0, 1.0)
