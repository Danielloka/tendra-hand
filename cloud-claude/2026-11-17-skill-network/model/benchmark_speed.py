"""MuJoCo CPU speed of v21_scene.xml, one thread (mj_step only, arm holding the `table` pose).

uv run python model/benchmark_speed.py
"""

import sys
import time
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_robot as br


def main() -> None:
    m = mujoco.MjModel.from_xml_path(str(br.SCENE_XML))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("table").id)
    for _ in range(500):
        mujoco.mj_step(m, d)
    n = 5000
    t = time.perf_counter()
    for _ in range(n):
        mujoco.mj_step(m, d)
    dt = time.perf_counter() - t
    print(
        f"{n / dt:.0f} steps/s (x{n / dt * m.opt.timestep:.1f} real time), nbody {m.nbody} ngeom {m.ngeom} nv {m.nv}"
    )


if __name__ == "__main__":
    main()
