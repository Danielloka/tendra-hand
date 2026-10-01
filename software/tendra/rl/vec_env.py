"""Run many `GraspEnv`s at once: in worker processes (all CPU cores) or in this process (tests).

Physics is the bottleneck (~2 ms per control step on the owner's laptop). Worker processes put
every CPU core to work, on the physics and on the Python reward/observation code alike. Each
worker owns several envs, so one message carries a whole batch (less overhead per step). The
factory must be picklable (a top-level function or a `functools.partial` of one).

Envs reset themselves when an episode ends; the step result then holds the *new* episode's first
observation, and `info["final_critic"]` the last observation of the finished one (the trainer
needs it to bootstrap the value of time-limit endings).
"""

from __future__ import annotations

import multiprocessing as mp
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

EnvFactory = Callable[[int], Any]  # seed -> env with reset/step/configure/set_demos


class _Batch:
    """Several envs stepped in a loop, with auto-reset."""

    def __init__(self, factory: EnvFactory, seeds: Sequence[int]) -> None:
        self.envs = [factory(s) for s in seeds]

    def reset(self) -> tuple[np.ndarray, np.ndarray]:
        obs = [env.reset() for env in self.envs]
        return np.stack([o[0] for o in obs]), np.stack([o[1] for o in obs])

    def step(self, actions: np.ndarray) -> tuple:
        actor, critic, rew, term, trunc, infos = [], [], [], [], [], []
        for env, a in zip(self.envs, actions, strict=True):
            (oa, oc), r, te, tr, info = env.step(a)
            if te or tr:
                info["final_critic"] = oc
                oa, oc = env.reset()
            actor.append(oa), critic.append(oc), rew.append(r), term.append(te), trunc.append(tr)
            infos.append(info)
        return (np.stack(actor), np.stack(critic), np.array(rew, dtype=np.float32),
                np.array(term), np.array(trunc), infos)  # fmt: skip

    def call(self, method: str, *args: Any, **kwargs: Any) -> list[Any]:
        return [getattr(env, method)(*args, **kwargs) for env in self.envs]


def _worker(conn, factory: EnvFactory, seeds: Sequence[int]) -> None:
    try:
        batch = _Batch(factory, seeds)
        conn.send(("ok", None))
    except Exception as e:  # noqa: BLE001  (report build errors instead of hanging the parent)
        conn.send(("error", repr(e)))
        return
    while True:
        cmd, payload = conn.recv()
        try:
            if cmd == "step":
                conn.send(("ok", batch.step(payload)))
            elif cmd == "reset":
                conn.send(("ok", batch.reset()))
            elif cmd == "call":
                method, args, kwargs = payload
                conn.send(("ok", batch.call(method, *args, **kwargs)))
            elif cmd == "close":
                conn.send(("ok", None))
                return
        except Exception as e:  # noqa: BLE001  (send any env error to the parent)
            conn.send(("error", repr(e)))


class VecEnv:
    """`num_envs` envs split over `workers` processes (0 = all in this process)."""

    def __init__(self, factory: EnvFactory, num_envs: int, workers: int = 0, seed: int = 0) -> None:
        self.num_envs = num_envs
        seeds = [seed * 10_000 + i for i in range(num_envs)]
        self.workers = min(workers, num_envs)
        if self.workers == 0:
            self._local = _Batch(factory, seeds)
            self._conns: list = []
            return
        self._local = None
        ctx = mp.get_context("spawn")  # Windows has only spawn; also safest with MuJoCo
        chunks = np.array_split(np.array(seeds), self.workers)
        self._conns, self._procs, self._sizes = [], [], []
        for chunk in chunks:
            parent, child = ctx.Pipe()
            proc = ctx.Process(target=_worker, args=(child, factory, chunk.tolist()), daemon=True)
            proc.start()
            self._conns.append(parent)
            self._procs.append(proc)
            self._sizes.append(len(chunk))
        for conn in self._conns:
            self._check(conn.recv())

    @staticmethod
    def _check(msg: tuple[str, Any]) -> Any:
        status, value = msg
        if status != "ok":
            raise RuntimeError(f"env worker failed: {value}")
        return value

    def _all(self, cmd: str, payloads: Sequence[Any]) -> list[Any]:
        for conn, p in zip(self._conns, payloads, strict=True):
            conn.send((cmd, p))
        return [self._check(conn.recv()) for conn in self._conns]

    def reset(self) -> tuple[np.ndarray, np.ndarray]:
        if self._local:
            return self._local.reset()
        parts = self._all("reset", [None] * self.workers)
        return np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts])

    def step(self, actions: np.ndarray) -> tuple:
        if self._local:
            return self._local.step(actions)
        splits = np.split(actions, np.cumsum(self._sizes)[:-1])
        parts = self._all("step", splits)
        return (np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts]),
                np.concatenate([p[2] for p in parts]), np.concatenate([p[3] for p in parts]),
                np.concatenate([p[4] for p in parts]), [i for p in parts for i in p[5]])  # fmt: skip

    def call(self, method: str, *args: Any, **kwargs: Any) -> list[Any]:
        """Call a method on every env (e.g. configure, set_demos); returns all results."""
        if self._local:
            return self._local.call(method, *args, **kwargs)
        parts = self._all("call", [(method, args, kwargs)] * self.workers)
        return [r for p in parts for r in p]

    def close(self) -> None:
        if self._local:
            return
        for conn in self._conns:
            try:
                conn.send(("close", None))
                conn.recv()
            except (BrokenPipeError, EOFError, OSError):
                pass
        for proc in self._procs:
            proc.join(timeout=5)
            if proc.is_alive():
                proc.terminate()

    def __enter__(self) -> VecEnv:  # noqa: PYI034
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
