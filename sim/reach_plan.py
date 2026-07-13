"""reach_plan.py — the merged base+arm kinematic model: WHERE to park the base so the arm can reach
a target without the base hitting the obstacle.

The robot is a macro-micro mobile manipulator: the holonomic base (SE(2), from bridge/omni_kinematics)
does coarse positioning, the SO-101 arm does the fine reach. Combined:

    T_world_ee = T_world_base(x, y, θ) · T_base_arm · FK_arm(q)

Given a TARGET (a world point to reach) sitting on/near an OBSTACLE, "drive up to it and reach"
resolves to a **standoff band** for the base along the approach ray:

    d_min = closest the base disk can get without the OBSTACLE (footprint collision)   [don't-hit]
    d_max = farthest the base can be and still have the ARM reach the target           [reach]

If d_min ≤ d_max the target is doable — park at the *closest safe* standoff (least arm extension,
best manipulability) and reach. If d_min > d_max the arm can't reach from any safe spot (obstacle
too big / target too deep) — reported honestly, the same "flag, don't fake" as the recognizer.

This is the planner (pure geometry: reach shell + disk-vs-OBB footprint clearance); the sim
(sim/lekiwi_sim.py --reach) then DRIVES to the pose and runs the real arm IK to verify it executes.
Obstacle/target poses come from tracking/ (sim ground-truth now; a camera via scanning.py later).

    python sim/reach_plan.py --demo       # render a printer-pick plan -> build/reach_plan.png
    python sim/reach_plan.py --selftest    # assert feasible plans clear + reach; infeasible flagged
    from reach_plan import plan, Base, Arm, Obstacle
"""
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bridge.omni_kinematics import OmniGeometry

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"


@dataclass
class Base:
    """The mobile base as a disk footprint + where the arm shoulder sits on it (base frame, +x fwd)."""
    radius_m: float = 0.12          # circular footprint that clears the wheels (base_r + wheel_r + gap)
    shoulder_fwd_m: float = 0.02    # arm shoulder forward of base centre (SO-101 sits ~central)
    shoulder_z_m: float = 0.13      # arm shoulder height above the floor

    @classmethod
    def from_omni(cls, geo: OmniGeometry, wheel_gap_m=0.012, **kw):
        return cls(radius_m=geo.base_r + geo.wheel_r + wheel_gap_m, **kw)


@dataclass
class Arm:
    """SO-101 reach shell: the ee can reach a point whose distance from the shoulder is in
    [reach_min, reach_max]. (Approximate; the sim verifies with the real DLS/Placo IK.)"""
    reach_max_m: float = 0.35       # max shoulder->ee (SO-101 ~arm span; refine from the URDF)
    reach_min_m: float = 0.08       # dead zone near the shoulder


@dataclass
class Obstacle:
    """A solid on the floor the base must not hit — an oriented box footprint (XY) + a height."""
    cx: float = 0.0
    cy: float = 0.0
    hx: float = 0.15                # half-extent along the box x
    hy: float = 0.15                # half-extent along the box y
    yaw: float = 0.0                # box orientation (rad)
    top_z: float = 0.25             # height (for reach-over checks / the elevation view)


@dataclass
class Plan:
    reachable: bool
    reason: str = ""
    base_pose: tuple = (0.0, 0.0, 0.0)     # (x, y, yaw) world
    standoff_m: float = 0.0                # base centre -> target, along the approach ray
    closest_safe_m: float = 0.0            # nearest the base can get without hitting (don't-hit limit)
    reach_used_m: float = 0.0              # shoulder -> target horizontal distance at the chosen pose
    reach_max_h_m: float = 0.0             # horizontal reach available at the target height
    footprint_clearance_m: float = 0.0     # base disk edge -> obstacle surface (>=0 = clear)
    approach: tuple = (1.0, 0.0)           # unit approach direction (target -> base)


def _obb_signed_dist(px, py, o: Obstacle):
    """Signed distance from point (px,py) to the obstacle OBB in XY (>0 outside, <0 inside)."""
    dx, dy = px - o.cx, py - o.cy
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    lx, ly = c * dx + s * dy, -s * dx + c * dy      # into the box's local frame
    qx, qy = abs(lx) - o.hx, abs(ly) - o.hy
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside


def _closest_safe_d(tx, ty, ax, ay, o: Obstacle, need):
    """Smallest d >= 0 along the approach ray where the base CENTRE clears the obstacle by `need`
    (= base.radius + margin). Distance-to-OBB grows monotonically as we back away, so bisection is
    exact once we bracket a clear d; if the target itself is already clear, d = 0."""
    def clear(d):
        return _obb_signed_dist(tx + d * ax, ty + d * ay, o) - need
    if clear(0.0) >= 0.0:
        return 0.0
    hi = max(o.hx, o.hy) + need + math.hypot(tx - o.cx, ty - o.cy) + 1.0   # guaranteed clear
    lo = 0.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if clear(mid) >= 0.0:
            hi = mid
        else:
            lo = mid
    return hi


def plan(target, obstacle: Obstacle, base: Base, arm: Arm, approach=None, margin=0.02) -> Plan:
    """Merged-model base placement. `target` = (x,y,z) world point to reach. `approach` = (ax,ay)
    direction the base comes FROM (default: radially out from the obstacle centre through the target).
    Returns a Plan with the base pose at the closest safe, reachable standoff — or reachable=False."""
    tx, ty, tz = target
    if approach is None:
        ax, ay = tx - obstacle.cx, ty - obstacle.cy
        if math.hypot(ax, ay) < 1e-6:
            ax, ay = 1.0, 0.0                        # target at obstacle centre -> pick +x
    else:
        ax, ay = approach
    n = math.hypot(ax, ay) or 1.0
    ax, ay = ax / n, ay / n                          # unit: points from target toward the base
    byaw = math.atan2(-ay, -ax)                      # base +x faces the target

    # vertical reach: is the target within the arm's spherical shell at any horizontal distance?
    dz = tz - base.shoulder_z_m
    if abs(dz) > arm.reach_max_m:
        return Plan(False, f"target height Δz={dz:+.3f}m exceeds arm reach {arm.reach_max_m}m",
                    approach=(ax, ay))
    horiz_max = math.sqrt(max(arm.reach_max_m ** 2 - dz ** 2, 0.0))
    horiz_min = math.sqrt(max(arm.reach_min_m ** 2 - dz ** 2, 0.0))

    # standoff band. shoulder sits shoulder_fwd toward the target from the base centre, so at standoff
    # d the shoulder->target horizontal distance is (d - shoulder_fwd).
    d_reach_near = base.shoulder_fwd_m + horiz_min
    d_reach_far = base.shoulder_fwd_m + horiz_max
    d_safe = _closest_safe_d(tx, ty, ax, ay, obstacle, base.radius_m + margin)

    lo = max(d_reach_near, d_safe)
    if lo > d_reach_far + 1e-9:
        why = (f"can't hit AND reach: closest safe standoff {d_safe:.3f}m needs the shoulder "
               f"{d_safe - base.shoulder_fwd_m:.3f}m out, but the arm reaches only {horiz_max:.3f}m "
               f"at this height — obstacle too big or target too deep for the arm")
        return Plan(False, why, standoff_m=d_safe, closest_safe_m=d_safe,
                    reach_max_h_m=horiz_max, approach=(ax, ay))

    d = lo                                            # closest safe, reachable -> least arm extension
    bx, by = tx + d * ax, ty + d * ay
    clr = _obb_signed_dist(bx, by, obstacle) - base.radius_m
    return Plan(True, "ok", base_pose=(bx, by, byaw), standoff_m=d, closest_safe_m=d_safe,
                reach_used_m=d - base.shoulder_fwd_m, reach_max_h_m=horiz_max,
                footprint_clearance_m=clr, approach=(ax, ay))


# --- a scenario: pick a part off a printer bed -----------------------------------------------
def printer_pick(base: Base = None):
    """A P1S-like printer (a box) with the part to grab at the front edge of the bed."""
    base = base or Base.from_omni(OmniGeometry())
    printer = Obstacle(cx=0.0, cy=0.6, hx=0.18, hy=0.18, yaw=0.0, top_z=0.25)     # ~0.36 m cube
    target = (0.0, 0.6 - 0.18 + 0.03, 0.22)     # part at the near face of the bed, up near the top
    return target, printer, base, Arm()


def render(target, obstacle, base, arm, pl: Plan, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, Wedge
    fig, (axp, axe) = plt.subplots(1, 2, figsize=(13, 6))

    # ---- top-down ----
    axp.set_title("base placement (top-down)")
    ang = math.degrees(obstacle.yaw)
    axp.add_patch(Rectangle((obstacle.cx - obstacle.hx, obstacle.cy - obstacle.hy),
                            2 * obstacle.hx, 2 * obstacle.hy, angle=ang,
                            rotation_point='center', fc="#b0b4bb", ec="#6a6e76", label="obstacle"))
    tx, ty, tz = target
    axp.plot(tx, ty, "*", ms=16, color="#e04b4b", label="target", zorder=5)
    ax_, ay_ = pl.approach
    if pl.reachable:
        bx, by, byaw = pl.base_pose
        sx, sy = bx - base.shoulder_fwd_m * ax_, by - base.shoulder_fwd_m * ay_   # shoulder
        axp.add_patch(Circle((bx, by), base.radius_m, fc="#4a90d9", ec="#2c6bb0", alpha=0.6,
                             label="base @ standoff"))
        # reach annulus at the target height, around the shoulder
        axp.add_patch(Wedge((sx, sy), pl.reach_max_h_m, 0, 360, width=pl.reach_max_h_m,
                            fc="none", ec="#3fae5a", ls="--", label="arm reach"))
        axp.plot([sx, tx], [sy, ty], "-", color="#3fae5a", lw=2, zorder=4)          # arm to target
        axp.plot(bx, by, "+", color="#12315a", ms=12, mew=2)
        # closest-safe vs chosen standoff, along the approach ray
        cs = (tx + pl.closest_safe_m * ax_, ty + pl.closest_safe_m * ay_)
        axp.plot(*cs, "o", mfc="none", mec="#e08a3c", ms=10, label="closest-safe")
    else:
        cs = (tx + pl.closest_safe_m * ax_, ty + pl.closest_safe_m * ay_)
        axp.add_patch(Circle(cs, base.radius_m, fc="none", ec="#e04b4b", ls=":",
                             label="closest safe (can't reach)"))
        axp.annotate("INFEASIBLE", (tx, ty), (tx + 0.15, ty + 0.15), color="#e04b4b",
                     fontsize=11, weight="bold")
    axp.set_aspect("equal"); axp.grid(alpha=0.3); axp.legend(loc="upper right", fontsize=8)
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")

    # ---- elevation (approach-plane side view: horizontal distance from base centre vs height) ----
    axe.set_title("reach vs stand-off (elevation)")
    axe.add_patch(Rectangle((-obstacle.hy, 0), 2 * obstacle.hy, obstacle.top_z,
                            fc="#b0b4bb", ec="#6a6e76"))
    axe.plot(0, tz, "*", ms=16, color="#e04b4b")
    if pl.reachable:
        d = pl.standoff_m
        axe.plot(-d, 0, "s", color="#4a90d9", ms=12, label="base")
        axe.plot(-d + base.shoulder_fwd_m, base.shoulder_z_m, "o", color="#12315a", label="shoulder")
        axe.plot([-d + base.shoulder_fwd_m, 0], [base.shoulder_z_m, tz], "-", color="#3fae5a", lw=2,
                 label=f"reach {pl.reach_used_m:.2f}/{arm.reach_max_m:.2f} m")
    axe.axhline(0, color="k", lw=0.8); axe.set_aspect("equal"); axe.grid(alpha=0.3)
    axe.legend(loc="upper left", fontsize=8); axe.set_xlabel("along approach (m)"); axe.set_ylabel("z (m)")

    verdict = (f"REACHABLE — park {pl.standoff_m:.3f} m out (closest safe {pl.closest_safe_m:.3f} m), "
               f"arm {pl.reach_used_m:.3f}/{arm.reach_max_m:.2f} m, clearance {pl.footprint_clearance_m*1000:.0f} mm"
               if pl.reachable else f"INFEASIBLE — {pl.reason}")
    fig.suptitle("drive-up-and-reach plan:  " + verdict, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    BUILD.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    return path


def selftest():
    ok = True
    base = Base.from_omni(OmniGeometry())

    # 1. printer pick: reachable, and the base disk clears the printer footprint
    target, printer, base, arm = printer_pick(base)
    pl = plan(target, printer, base, arm)
    if not pl.reachable:
        print("FAIL printer-pick not reachable:", pl.reason); ok = False
    elif pl.footprint_clearance_m < -1e-6:
        print(f"FAIL base overlaps printer (clearance {pl.footprint_clearance_m:.3f})"); ok = False
    else:
        # base is farther from the target than the arm-only min AND clears the obstacle
        if pl.standoff_m < pl.closest_safe_m - 1e-6:
            print("FAIL chose a standoff inside the collision limit"); ok = False
        if pl.reach_used_m > arm.reach_max_m + 1e-6:
            print("FAIL arm asked to over-reach"); ok = False
        print(f"  printer-pick: park {pl.standoff_m:.3f}m out (closest-safe {pl.closest_safe_m:.3f}), "
              f"arm {pl.reach_used_m:.3f}/{arm.reach_max_m}m, clearance {pl.footprint_clearance_m*1000:.0f}mm")

    # 2. a target at the CENTRE-top of a wide table: the base must stand off the edge (~0.5 m + its
    #    radius), leaving the arm far short of the middle -> honest infeasible.
    table = Obstacle(cx=0.0, cy=0.6, hx=0.5, hy=0.5, top_z=0.4)
    centre_top = (0.0, 0.6, 0.4)
    pl2 = plan(centre_top, table, base, Arm(reach_max_m=0.35))
    if pl2.reachable:
        print("FAIL table-centre should be infeasible (arm can't reach the middle from the edge)"); ok = False
    else:
        print(f"  wide-table centre: correctly INFEASIBLE — {pl2.reason[:72]}...")

    # 3. closest-safe monotonicity: a bigger base must stand off at least as far
    small = plan(*printer_pick(Base(radius_m=0.08, shoulder_fwd_m=0.02, shoulder_z_m=0.13)))
    if small.reachable and small.closest_safe_m > pl.closest_safe_m + 1e-6:
        print("FAIL smaller base stood off farther than the bigger one"); ok = False

    print("PASS: merged base+arm reach planner — clears the obstacle, reaches when it can, "
          "flags when it can't" if ok else "FAIL")
    return 0 if ok else 1


def demo():
    target, printer, base, arm = printer_pick()
    pl = plan(target, printer, base, arm)
    p = render(target, printer, base, arm, pl, str(BUILD / "reach_plan.png"))
    print(("REACHABLE" if pl.reachable else "INFEASIBLE"),
          f"-> park {pl.standoff_m:.3f}m out, arm {pl.reach_used_m:.3f}/{arm.reach_max_m}m; wrote {p}")


if __name__ == "__main__":
    mode = "demo" if "--demo" in sys.argv else "selftest"
    sys.exit(selftest() if mode == "selftest" else (demo() or 0))
