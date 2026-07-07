"""assemble.py — end-to-end assembly from CAD: plan -> CAM -> motion -> track -> verify.

Ties the whole stack together for one sample assembly:

  1. REFERENCE  load the CAD assembly (parts + nominal poses)        [tracking.assembly]
  2. PLAN       an op-graph: place parts, then glue the seam         [orchestration.opgraph]
  3. CAM        the glue op -> a bead toolpath on the seam surface   [toolpath + sim/glue_cell]
  4. CLEARANCE  sweep the tool over the workpiece; refuse if it fouls[sim/interference]
  5. MOTION     toolpath -> joint waypoints via IK (Placo-pluggable) [toolpath + sim/ik]
  6. EXECUTE    walk the plan; track each part, re-observe on place  [tracking.world]
  7. VERIFY     tracked poses vs the CAD-nominal, within tolerance   [tracking.assembly.verify]

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
import interference as ix  # noqa: E402
from workcell import DATUM_POS  # noqa: E402
from tracking import Pose, load_assembly, verify  # noqa: E402
from bridge import solve_path as bridge_solve  # noqa: E402

CAPS = {"arm_hold": 1, "arm_glue": 1, "cure": 1}
CARRY_STEPS = 6           # dead-reckoned moves between observations (pose drifts)
PLACE_NOISE_MM = 0.7      # residual placement error the observation reports

WORKPIECE_STL = ROOT / "exports" / "example_plate.stl"   # the bracket at the datum
GLUE_TOOL_STL = ROOT / "exports" / "glue_body.stl"       # the effector sweeping the bead
TOOL_STANDOFF_MM = 18.0   # nozzle-tip standoff: how far the tool body rides above the seam


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


def clearance_check(path):
    """Solid interference check for the glue op: sweep the tool body along the bead vs the
    workpiece. Returns {ok, first_hit, tool_offset_mm, approximate, ...}. Missing STLs ->
    skipped (honest), so the pipeline still runs where CAD isn't exported."""
    if not (WORKPIECE_STL.exists() and GLUE_TOOL_STL.exists()):
        return {"ok": True, "skipped": "no STL exports", "first_hit": None}
    scene = ix.Scene().place("workpiece", str(WORKPIECE_STL), ix.pose_matrix_mm(DATUM_POS * 1000.0))
    xf = ix.transforms_along(path.poses, tool_offset_mm=(0.0, 0.0, TOOL_STANDOFF_MM))
    res = ix.sweep(str(GLUE_TOOL_STL), xf, scene)
    res["tool_offset_mm"] = TOOL_STANDOFF_MM
    return res


def run(program="seam"):
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

    # 4. CLEARANCE — sweep the glue tool along the bead vs the workpiece. Solid interference
    #    checking: the tool body rides TOOL_STANDOFF_MM above the seam (its nozzle length);
    #    a foul here means motion would drive the effector into the part, so refuse.
    report["clearance"] = clearance_check(path)
    report["clearance_ok"] = report["clearance"]["ok"]

    # 5. MOTION — toolpath -> joint waypoints. The bridge picks the backend: so101-lab
    #    Placo when its venv+URDF are set up, else the built-in positional IK.
    motion = bridge_solve(path.poses)
    report["ik_residual_mm"] = motion["max_err_mm"]
    report["reachable"] = motion["max_err_mm"] < 5.0
    report["motion_backend"] = motion["backend"]
    report["motion_status"] = motion["status"]

    # 6. EXECUTE + TRACK — grasp, carry (dead-reckon -> pose drifts), observe, place
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

    # 7. VERIFY — tracked poses vs the CAD assembly nominal
    ok, results = verify(world)
    report["verify_ok"] = ok
    report["verify"] = results
    return report


if __name__ == "__main__":
    rep = run("seam")
    print(f"assemble '{rep['program']}': makespan {rep['makespan_s']}s | "
          f"toolpath {rep['toolpath_points']} pts, IK residual {rep['ik_residual_mm']}mm "
          f"({'reachable' if rep['reachable'] else 'OUT OF REACH'})")
    cl = rep["clearance"]
    if cl.get("skipped"):
        print(f"  clearance: skipped ({cl['skipped']})")
    else:
        hit = cl["first_hit"]
        print(f"  clearance: {'CLEAR' if cl['ok'] else 'FOUL'} — tool {cl['tool_offset_mm']}mm "
              f"standoff, {cl['tested_pairs']} tested / {cl['pruned']} pruned"
              + (f", HIT {hit['part']} @wp{hit['index']} ({hit['volume_mm3']}mm³)" if hit else "")
              + ("  [approx]" if cl.get("approximate") else ""))
    print(f"  motion: {rep['motion_backend']} — {rep['motion_status']}")
    for name, s in rep["staleness"].items():
        print(f"  {name}: carry -> {s['before_observe']['verdict']} "
              f"(σ{s['before_observe']['sigma_mm']}mm) -> observe -> {s['after_observe']['verdict']}")
    print(f"  verify vs CAD nominal: {'OK' if rep['verify_ok'] else 'FAIL'} {rep['verify']}")
