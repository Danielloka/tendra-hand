"""Adaptive curriculum of the GPU trainer (tendra/gpu/curriculum.py): plain python, no jax."""

import itertools
import json

from tendra.gpu.curriculum import Curriculum, Rules


def chunk(cur: Curriculum, *scores: float) -> dict:
    for i, s in enumerate(scores, 1):
        cur.record_eval(i * 1000, s)
    return cur.decide(log=lambda *_: None)


def test_adds_objects_when_success_is_high():
    cur = Curriculum()
    assert cur.state.objects == ["cylinder"]
    assert chunk(cur, 0.3, 0.4)["objects"] == ["cylinder"]  # below 0.6: hold
    assert chunk(cur, 0.5, 0.7)["objects"] == ["cylinder", "cube"]  # mean 0.6
    assert chunk(cur, 0.1, 0.5)["objects"] == ["cylinder", "cube"]  # mean 0.3: hold
    assert chunk(cur, 0.7, 0.7)["objects"] == ["cylinder", "cube", "ball"]
    assert cur.state.chunk == 4


def test_score_uses_the_last_evaluations_of_the_chunk():
    cur = Curriculum(Rules(score_evals=2))
    cur.record_eval(1, 0.0)
    cur.record_eval(2, 0.8)
    cur.record_eval(3, 0.6)
    assert abs(cur.score() - 0.7) < 1e-9


def test_one_change_per_chunk_and_full_ramp():
    cur = Curriculum()
    seen = []
    for _ in range(40):
        s = chunk(cur, 0.9, 0.9)
        seen.append(dict(s))
        if cur.state.done:
            break
    # objects first, then penalties, then demos; never two at once
    for a, b in itertools.pairwise(seen):
        changed = sum(a[k] != b[k] for k in ("objects", "demo_prob", "penalty_scale"))
        assert changed <= 1
    st = cur.state
    assert st.objects == ["cylinder", "cube", "ball"]
    assert st.penalty_scale == 1.0 and st.demo_prob == cur.rules.demo_final
    assert st.done  # final settings and success >= target


def test_not_done_without_target_success():
    cur = Curriculum()
    for _ in range(40):
        chunk(cur, 0.7, 0.7)
    assert cur.is_final() and not cur.state.done


def test_regression_after_a_penalty_change_is_reverted():
    cur = Curriculum()
    cur.state.objects = ["cylinder", "cube", "ball"]
    s = chunk(cur, 0.8, 0.8)  # raises penalties 0.3 -> 0.5
    assert s["penalty_scale"] == 0.5
    s = chunk(cur, 0.3, 0.3)  # fell by 0.5: revert and wait
    assert s["penalty_scale"] == 0.3
    assert cur.state.cooldown_left == cur.rules.cooldown
    s = chunk(cur, 0.8, 0.8)  # cooldown chunk: no change
    assert s["penalty_scale"] == 0.3
    s = chunk(cur, 0.8, 0.8)
    assert s["penalty_scale"] == 0.5


def test_a_drop_after_adding_an_object_is_not_reverted():
    cur = Curriculum()
    assert chunk(cur, 0.9, 0.9)["objects"] == ["cylinder", "cube"]
    assert chunk(cur, 0.2, 0.2)["objects"] == ["cylinder", "cube"]  # kept, trains on


def test_domain_randomisation_only_with_env_support():
    for has_dr in (False, True):
        cur = Curriculum(has_dr=has_dr)
        cur.state.objects = ["cylinder", "cube", "ball"]
        cur.state.penalty_scale, cur.state.demo_prob = 1.0, cur.rules.demo_final
        s = chunk(cur, 0.9, 0.9)
        assert s["dr_level"] == (0.25 if has_dr else 0.0)


def test_best_is_per_difficulty_level():
    cur = Curriculum()
    assert cur.record_best(0.5)
    assert not cur.record_best(0.4)
    cur.state.objects = ["cylinder", "cube"]  # harder evaluation: lower success still counts
    assert cur.record_best(0.2)
    assert cur.record_best(0.3)


def test_resume_mid_chunk_from_json(tmp_path):
    path = tmp_path / "curriculum.json"
    cur = Curriculum(path=path)
    cur.record_eval(2000, 0.4)
    cur.state.total_steps = 2000
    cur.save()
    again = Curriculum.load_or_new(path, log=lambda *_: None)
    assert again.state.chunk_scores == [0.4] and again.state.chunk_steps_done == 2000
    assert again.state.total_steps == 2000
    json.loads(path.read_text())  # valid json
    chunk(again, 0.9)
    assert again.state.chunk == 1 and again.state.chunk_scores == []


def test_every_decision_is_logged():
    cur = Curriculum()
    chunk(cur, 0.9, 0.9)
    entry = cur.state.log[-1]
    assert entry["event"] == "decision" and entry["change"]["what"] == "objects"


def test_start_probs_shift_toward_normal_starts():
    cur = Curriculum(has_starts=True)
    assert cur.state.start_probs() == {"normal": 0.4, "near": 0.3, "demo": 0.3}
    cur.state.objects = ["cylinder", "cube", "ball"]
    cur.state.penalty_scale = 1.0
    seen = []
    for _ in range(10):
        chunk(cur, 0.9, 0.9)
        seen.append(cur.state.start_normal)
        if cur.state.done:
            break
    assert seen[:3] == [0.55, 0.7, 0.85] and seen[3] == 0.9
    p = cur.state.start_probs()
    assert abs(sum(p.values()) - 1.0) < 1e-9 and p["normal"] == 0.9
    assert cur.state.done and cur.state.demo_prob == cur.rules.demo_start  # demo_prob untouched
