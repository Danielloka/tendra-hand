"""Validate actuators.json and simulate each joint as a 1-DOF hinge (30 deg step) in MuJoCo.

Run from the study root: uv run python model/check_actuators.py
Reports 10-90 % rise time, overshoot, 2 % settling time and steady-state error (coulomb friction makes a small one).
"""
import json
import math
import sys
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
SERVOS = ("index_pip index_mcp_flex index_mcp_abd thumb_ip thumb_mcp_flex thumb_cmc_flex thumb_cmc_rot "
          "middle_pip middle_mcp_flex middle_mcp_abd ring_pip ring_mcp_flex ring_mcp_abd "
          "little_pip little_mcp_flex little_mcp_abd").split()
WRIST = ("wrist_flex", "wrist_dev")
KEYS = ("kp", "damping", "armature", "frictionloss", "velocity_limit_rad_s", "ratio", "efficiency",
        "stall_torque_servo_Nm", "load_inertia_kgm2")


def validate(d):
    errs = []
    for grp, names in (("servos", SERVOS), ("wrist", WRIST)):
        for n in names:
            v = d.get(grp, {}).get(n)
            if v is None:
                errs.append(f"missing {grp}/{n}")
                continue
            for k in KEYS:
                if not (k in v and v[k] > 0):
                    errs.append(f"{n}.{k} missing or not positive")
            fr = v.get("forcerange", [0, 0])
            if not (fr[0] < 0 < fr[1] and abs(fr[0] + fr[1]) < 1e-9):
                errs.append(f"{n}.forcerange not symmetric")
            if not 0 < v.get("efficiency", 0) <= 1:
                errs.append(f"{n}.efficiency outside (0,1]")
    if d.get("servo_model") != "HLS3606M":
        errs.append("servo_model must be HLS3606M")
    for k in ("servos", "wrist", "randomization"):
        if k not in d:
            errs.append(f"missing section {k}")
    return errs


def step(v, ang=30.0, dt=0.002, T=1.5):
    xml = f"""<mujoco><option timestep="{dt}" gravity="0 0 0"/>
    <worldbody><body><joint name="j" type="hinge" axis="0 0 1" armature="{v['armature']}"
      damping="{v['damping']}" frictionloss="{v['frictionloss']}"/>
      
      <inertial pos="0 0 0" mass="1e-3" diaginertia="{v['load_inertia_kgm2']} {v['load_inertia_kgm2']} {v['load_inertia_kgm2']}"/></body></worldbody>
    <actuator><position name="a" joint="j" kp="{v['kp']}" forcerange="{v['forcerange'][0]} {v['forcerange'][1]}"
      ctrllimited="false"/></actuator></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    tgt = math.radians(ang)
    d.ctrl[0] = tgt
    q = []
    for _ in range(int(T / dt)):
        mujoco.mj_step(m, d)
        q.append(d.qpos[0])
    q = [x / tgt for x in q]
    t10 = next((i for i, x in enumerate(q) if x >= 0.1), None)
    t90 = next((i for i, x in enumerate(q) if x >= 0.9), None)
    rise = (t90 - t10) * dt if t10 is not None and t90 is not None else float("nan")
    os_ = max(0.0, (max(q) - 1.0) * 100)
    settle = float("nan")
    for i in range(len(q) - 1, -1, -1):
        if abs(q[i] - 1) > 0.02:
            settle = (i + 1) * dt
            break
    else:
        settle = 0.0
    return rise, os_, settle, (q[-1] - 1) * 100


def main():
    d = json.load(open(HERE / "actuators.json"))
    errs = validate(d)
    if errs:
        print("INVALID:", *errs, sep="\n  ")
        sys.exit(1)
    print("actuators.json: all 18 contract joints present, positive values")
    print(f"{'joint':16s} {'kp':>7s} {'tau_max':>8s} {'damp':>7s} {'arm':>8s} {'I_load':>8s} {'rise ms':>8s} {'OS %':>6s} {'settle ms':>9s} {'ss err %':>8s} {'vmax rad/s':>10s}")
    for grp in ("servos", "wrist"):
        for n, v in d[grp].items():
            r, o, s, e = step(v, 30.0 if grp == "servos" else 20.0)
            print(f"{n:16s} {v['kp']:7.3f} {v['forcerange'][1]:8.4f} {v['damping']:7.4f} {v['armature']:8.5f} "
                  f"{v['load_inertia_kgm2']:8.1e} {r*1e3:8.0f} {o:6.1f} {s*1e3:9.0f} {e:8.2f} {v['velocity_limit_rad_s']:10.2f}")
    print("(fingers: 30 deg step; wrist: 20 deg step)")
    # cross-check: index pinch force = (tau_pip - pretension) / lever, vs the reports' HLS3606M numbers
    pip = d["servos"]["index_pip"]
    pre = 5.0 * 7.235e-3
    f100 = (pip['forcerange'][1] - pre) / 0.0462
    f80 = (pip['forcerange_cap80'][1] - pre) / 0.0462
    print(f"index pinch: stall {f100:.1f} N, 80 % cap {f80:.1f} N "
          "(reports, HLS3606M: 11.6 N (review) / 15.7 N hand / 16.6 N bench / 17.6 N after the roller fix; 80 %: 12.4-13.1 N)")
    # strand tension at stall vs part ratings (reports 02, 12, 13)
    print("\nstrand tension at servo stall (5 mm spool):")
    for n, v in d["servos"].items():
        t = v["strand_tension_stall_N"]
        break
    print(f"  {t:.1f} N on every joint (80 % cap {0.8*t:.1f} N). Fuse cup HLS version trips at 148 N (1.26x stall, 111 N at -25 % scatter = BELOW stall); "
          "SCS version (67 N) would trip at 0.57x stall.")
    print("  other limits: PE#8 445 N break (SF 3.8), MR63 roller/idler s0 0.48/0.59 at 80 %, MR84 lug 0.93, dog slots 27 MPa on PETG, eye lip 418 MPa")
    # power
    print(f"\nstall current 16 x 1.3 A = {16*1.3:.1f} A at 6 V (+ 2 x 2.7 A at 12 V for the wrist)")


if __name__ == "__main__":
    main()
