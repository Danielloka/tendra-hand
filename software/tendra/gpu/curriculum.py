"""Adaptive curriculum for the GPU grasp training: a small state machine, no jax needed.

Training runs in *chunks* (a few million steps each). After every chunk `Curriculum.decide` reads
the evaluation success (normal starts, no demo help, deterministic policy) and picks the settings
of the next chunk. One change per chunk, so its effect can be told apart:

1. **objects**: when success on the current objects passes `add_object_at`, the next object is
   added (cylinder, then cube, then ball). Success then drops: that is expected, not a regression.
2. **penalty_scale**: raised by `penalty_step` while success stays above `ramp_at`, up to 1.0.
3. **demo_prob**: halved while success stays above `ramp_at` (the scripted-grasp starts are a crutch
   for exploration; at the end the policy must work from normal starts), down to `demo_final`.
4. **dr_scale** (only if the env has it): domain-randomisation width, raised by `dr_step`.

Guard: if success drops by more than `regress_drop` right after a *penalty / demo / DR* change,
that change is reverted and nothing changes for `cooldown` chunks. Training is done when every
setting has reached its final value and success stays above `target` (or the step budget ends).

The state is a plain dataclass saved as JSON after every evaluation; loading it resumes a
disconnected session exactly (including the evaluations already seen in an unfinished chunk).
Standalone (stdlib only), like the other files in the Colab kit.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

OBJECT_ORDER = ("cylinder", "cube", "ball")


@dataclass
class Rules:
    """Thresholds of the state machine (success = share of evaluation episodes that lifted)."""

    add_object_at: float = 0.60
    ramp_at: float = 0.60
    regress_drop: float = 0.20
    penalty_start: float = 0.3
    penalty_step: float = 0.2
    demo_start: float = 0.5
    demo_final: float = 0.02
    dr_step: float = 0.25
    cooldown: int = 1
    target: float = 0.90
    score_evals: int = 2  # the score is the mean of the chunk's last N evaluations
    patience: int = 6  # chunks without a new best (at the final settings) before a warning


@dataclass
class State:
    """Everything needed to resume. `chunk_*` describe the unfinished chunk, if any."""

    objects: list[str] = field(default_factory=lambda: [OBJECT_ORDER[0]])
    demo_prob: float = 0.5
    penalty_scale: float = 0.3
    dr_scale: float = 0.0
    chunk: int = 0  # index of the chunk being trained
    chunk_steps_done: int = 0
    chunk_scores: list[float] = field(default_factory=list)
    total_steps: int = 0
    lr_scale: float = 1.0
    rollbacks: int = 0
    cooldown_left: int = 0
    last_change: dict | None = None  # {"what", "old", "new", "score_before"}
    best_key: list[float] = field(default_factory=lambda: [0, 0.0, -1.0])
    stale_chunks: int = 0
    done: bool = False
    log: list[dict] = field(default_factory=list)  # every decision, newest last

    def settings(self) -> dict:
        return {"objects": list(self.objects), "demo_prob": self.demo_prob,
                "penalty_scale": self.penalty_scale, "dr_scale": self.dr_scale}  # fmt: skip

    def level(self) -> tuple[int, float]:
        """What makes evaluation harder: number of objects, DR width (penalty/demo do not)."""
        return len(self.objects), self.dr_scale


class Curriculum:
    def __init__(self, rules: Rules | None = None, has_dr: bool = False,
                 path: Path | None = None, state: State | None = None) -> None:  # fmt: skip
        self.rules = rules or Rules()
        self.has_dr = has_dr
        self.path = path
        self.state = state or State(demo_prob=self.rules.demo_start,
                                    penalty_scale=self.rules.penalty_start)  # fmt: skip
        if not self.has_dr:
            self.state.dr_scale = 0.0

    # ----- persistence -----

    @classmethod
    def load_or_new(cls, path: Path, rules: Rules | None = None, has_dr: bool = False,
                    log=print) -> Curriculum:  # fmt: skip
        path = Path(path)
        if path.exists():
            raw = json.loads(path.read_text())
            raw["best_key"] = list(raw.get("best_key", [0, 0.0, -1.0]))
            state = State(**{k: v for k, v in raw.items() if k in State.__dataclass_fields__})
            log(
                f"curriculum: resuming chunk {state.chunk}, {state.total_steps:,} steps, "
                f"settings {state.settings()}"
            )
            return cls(rules, has_dr, path, state)
        return cls(rules, has_dr, path)

    def save(self) -> None:
        if self.path is None:
            return
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps(asdict(self.state), indent=1))
        tmp.replace(self.path)

    # ----- events -----

    def note(self, event: str, log=print, **info) -> None:
        entry = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "chunk": self.state.chunk,
                 "steps": self.state.total_steps, "event": event, **info}  # fmt: skip
        self.state.log.append(entry)
        log("curriculum: " + event + " " + json.dumps(info))

    def record_eval(self, steps_in_chunk: int, success: float) -> None:
        """An evaluation inside the chunk (not the one at step 0)."""
        st = self.state
        st.chunk_scores.append(float(success))
        st.chunk_steps_done = steps_in_chunk
        self.save()

    def record_best(self, success: float) -> bool:
        """True if this evaluation is the best so far at the current difficulty level."""
        key = [*self.state.level(), float(success)]
        if key > self.state.best_key:
            self.state.best_key = key
            return True
        return False

    def score(self) -> float:
        s = self.state.chunk_scores[-self.rules.score_evals :]
        return sum(s) / len(s) if s else 0.0

    def is_final(self) -> bool:
        st, r = self.state, self.rules
        return (
            len(st.objects) == len(OBJECT_ORDER)
            and st.penalty_scale >= 1.0
            and st.demo_prob <= r.demo_final + 1e-9
            and (not self.has_dr or st.dr_scale >= 1.0)
        )

    # ----- the decision -----

    def decide(self, log=print) -> dict:
        """Close the finished chunk, change at most one setting; returns the next settings."""
        st, r = self.state, self.rules
        score = self.score()
        change: dict | None = None
        why = ""

        last = st.last_change
        if last and last["what"] != "objects" and score < last["score_before"] - r.regress_drop:
            setattr(st, last["what"], last["old"])
            st.cooldown_left = r.cooldown
            change, why = (
                {"what": last["what"], "old": last["new"], "new": last["old"]},
                (
                    f"success fell {last['score_before']:.2f} -> {score:.2f} after the change: revert"
                ),
            )
            st.last_change = None
        elif st.cooldown_left > 0:
            st.cooldown_left -= 1
            why = f"cooldown ({st.cooldown_left} left)"
            st.last_change = None
        else:
            change, why = self._next_change(score)
            st.last_change = None if change is None else {**change, "score_before": score}
            if change is not None:
                setattr(st, change["what"], change["new"])

        if change is None and self.is_final():
            if score >= r.target:
                st.done = True
                why = f"all settings final and success {score:.2f} >= {r.target}"
            else:
                st.stale_chunks += 1
                if st.stale_chunks >= r.patience:
                    why += " (no progress for a while at the final settings: consider stopping)"
        self.note("decision", log, score=round(score, 3), change=change, why=why,
                  settings=st.settings())  # fmt: skip
        st.chunk += 1
        st.chunk_steps_done = 0
        st.chunk_scores = []
        self.save()
        return st.settings()

    def _next_change(self, score: float) -> tuple[dict | None, str]:
        st, r = self.state, self.rules
        if len(st.objects) < len(OBJECT_ORDER):
            if score >= r.add_object_at:
                new = [*st.objects, OBJECT_ORDER[len(st.objects)]]
                return ({"what": "objects", "old": list(st.objects), "new": new},
                        f"success {score:.2f} >= {r.add_object_at}: add {new[-1]}")  # fmt: skip
            return None, f"success {score:.2f} < {r.add_object_at}: keep the objects"
        if score < r.ramp_at:
            return None, f"success {score:.2f} < {r.ramp_at}: hold the settings"
        if st.penalty_scale < 1.0:
            new = round(min(1.0, st.penalty_scale + r.penalty_step), 3)
            return {"what": "penalty_scale", "old": st.penalty_scale, "new": new}, "raise penalties"
        if st.demo_prob > r.demo_final + 1e-9:
            new = round(max(r.demo_final, st.demo_prob / 2), 4)
            return {"what": "demo_prob", "old": st.demo_prob, "new": new}, "fewer demo starts"
        if self.has_dr and st.dr_scale < 1.0:
            new = round(min(1.0, st.dr_scale + r.dr_step), 3)
            return {"what": "dr_scale", "old": st.dr_scale, "new": new}, "widen randomisation"
        return None, "nothing left to change"
