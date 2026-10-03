"""Staggered resets (tendra/gpu/stagger.py) on a tiny MJX env; needs jax + mujoco-playground."""

import pytest

pytest.importorskip("jax")
pytest.importorskip("mujoco_playground")
pytest.importorskip("brax")

import jax
import jax.numpy as jp
import mujoco
from ml_collections import config_dict
from mujoco import mjx
from mujoco_playground._src import mjx_env
from tendra.gpu.stagger import wrap_for_training

XML = """<mujoco><worldbody><body><joint name="j" type="hinge"/>
<geom size="0.1"/></body></worldbody></mujoco>"""


class Toy(mjx_env.MjxEnv):
    def __init__(self):
        super().__init__(config_dict.create(impl="jax", ctrl_dt=0.02, sim_dt=0.02))
        self._m = mujoco.MjModel.from_xml_string(XML)
        self._mx = mjx.put_model(self._m)

    def reset(self, rng):
        data = mjx.make_data(self._m)
        return mjx_env.State(data, {"state": jp.zeros(2)}, jp.array(0.0), jp.array(0.0), {}, {})

    def step(self, state, action):
        return state.replace(reward=jp.array(1.0), done=jp.array(0.0))

    xml_path = "toy"
    action_size = 1
    mj_model = property(lambda self: self._m)
    mjx_model = property(lambda self: self._mx)


def test_steps_are_spread_over_the_episode_and_eval_is_not_staggered():
    n, length = 64, 50
    env = wrap_for_training(Toy(), episode_length=length)
    state = jax.jit(env.reset)(jax.random.split(jax.random.PRNGKey(0), n))
    steps = state.info["steps"]
    assert steps.shape == (n,)
    assert 0 <= steps.min() and steps.max() < length
    assert len(set(steps.tolist())) > 20  # spread, not all equal

    eval_env = Toy()
    eval_env.no_stagger = True
    plain = wrap_for_training(eval_env, episode_length=length)
    state = jax.jit(plain.reset)(jax.random.split(jax.random.PRNGKey(0), n))
    assert float(state.info["steps"].max()) == 0.0


def test_episodes_end_at_different_steps_then_keep_the_phase():
    n, length = 32, 20
    env = wrap_for_training(Toy(), episode_length=length)
    state = jax.jit(env.reset)(jax.random.split(jax.random.PRNGKey(1), n))
    step = jax.jit(env.step)
    done_at = {}
    for t in range(1, 3 * length):
        state = step(state, jp.zeros((n, 1)))
        for i in jp.nonzero(state.done)[0].tolist():
            done_at.setdefault(i, []).append(t)
    firsts = {v[0] for v in done_at.values()}
    assert len(firsts) > 8  # the first episodes end all over the place
    for v in done_at.values():
        if len(v) > 1:
            assert v[1] - v[0] == length  # then full-length episodes, phase kept
