"""assemble.py — end-to-end assembly from CAD: plan -> CAM -> motion -> track -> verify.

Ties the whole stack together for one sample assembly:

  1. REFERENCE  load the CAD assembly (parts + nominal poses)        [tracking.assembly]
  2. PLAN       an op-graph: place parts, then glue the seam         [orchestration.opgraph]
  3. CAM        the glue op -> a bead toolpath on the seam surface   [toolpath + sim/glue_cell]
  4. MOTION     toolpath -> joint waypoints via IK (Placo-pluggable) [toolpath + sim/ik]
  5. EXECUTE    walk the plan; track each part, re-observe on place  [tracking.world]
  6. VERIFY     tracked poses vs the CAD-nominal, within tolerance   [tracking.assembly.verify]

Runs entirely in sim (observations synthesized from ground truth); the real arms
(so101-lab, composed by reference) and a camera replace steps 4-5's backends later.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for sub in ("", "orchestration", "sim"):
    p = str(ROOT / sub) if sub else str(ROOT)
    if p not in sys.path:
        sys.path.insert(0, p)

from opgraph import Operation, OperationGraph, schedule  # noqa: E402
import toolpath as tp  # noqa: E402
import glue_cell  # noqa: E402
from workcell import DATUM_POS  # noqa: E402
from tracking import Pose, load_assembly, verify  # noqa: E402

CAPS = {"arm_hold": 1, "arm_glue": 1, "cure": 1}
CARRY_STEPS = 6           # dead-reckoned moves between observations (pose drifts)
PLACE_NOISE_MM = 0.7      # residual placement error the observation reports


def plan(program="seam"):
    r = glue_cell.apply_bead(program)
    ops = [
        Operation("place_bracket", "arm_hold", 4.0),
        Operation("present_staple", "arm_hold", 3.0, needs=("place_bracket",)),
        Operation("glue_seam", "arm_glue", r["apply_s"], needs=("present_staple",), tool="glue"),
        Operation("hold_set", "arm_hold", r["set_s"], needs=("present_staple",)),
        Operation("cure", "cure", r["set_s"], needs=("glue_seam",)),
        Operation("release", "arm_hold", 2.0, needs=("hold_set", "cure")),
    ]
    return OperationGraph(ops), r


def run(program="seam", solve=None):
    report = {"program": program}

    # 1. REFERENCE
    world = load_assembly()

    # 2. PLAN
    g, r = plan(program)
    sched, makespan = schedule(g, capacity=CAPS)
    report["makespan_s"] = round(makespan, 1)

    # 3. CAM — the glue op becomes a bead toolpath on the seam surface (at the datum)
    seam_origin = DATUM_POS + np.array([0.0, 0.0, 0.0])
    path = tp.bead_toolpath(r["points"], seam_origin, label=f"glue:{program}")
    report["toolpath_points"] = len(path.poses)

    # 4. MOTION — toolpath -> joint waypoints via IK (built-in; Placo pluggable)
    q_wps, ik_residual_mm = tp.to_joint_traj(path, solve=solve)
    report["ik_residual_mm"] = round(ik_residual_mm, 2)
    report["reachable"] = ik_residual_mm < 5.0

    # 5. EXECUTE + TRACK — grasp, carry (dead-reckon -> pose drifts), observe, place
    stamps = {}
    for name in ("bracket", "staple"):
        world.tick()
        world.grasp(name, "arm_hold")
        for _ in range(CARRY_STEPS):                 # move it without seeing it
            world.tick()
            world.carry(name, world.parts[name].est)
        stamps[name] = {"before_observe": world.staleness(name)}   # should be drifted
        # re-observe at placement (fiducial / CAD-pose) near nominal, then place
        nom = world.parts[name].nominal
        obs = Pose((nom.xyz[0] + PLACE_NOISE_MM / 1000.0, nom.xyz[1], nom.xyz[2]), nom.quat)
        world.observe(name, obs)
        world.place(name, obs)
        stamps[name]["after_observe"] = world.staleness(name)      # re-anchored -> FRESH
    report["staleness"] = stamps

    # 6. VERIFY — tracked poses vs the CAD assembly nominal
    ok, results = verify(world)
    report["verify_ok"] = ok
    report["verify"] = results
    return report


if __name__ == "__main__":
    rep = run("seam")
    print(f"assemble '{rep['program']}': makespan {rep['makespan_s']}s | "
          f"toolpath {rep['toolpath_points']} pts, IK residual {rep['ik_residual_mm']}mm "
          f"({'reachable' if rep['reachable'] else 'OUT OF REACH'})")
    for name, s in rep["staleness"].items():
        print(f"  {name}: carry -> {s['before_observe']['verdict']} "
              f"(σ{s['before_observe']['sigma_mm']}mm) -> observe -> {s['after_observe']['verdict']}")
    print(f"  verify vs CAD nominal: {'OK' if rep['verify_ok'] else 'FAIL'} {rep['verify']}")
