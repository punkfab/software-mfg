"""opgraph.py — the operation graph + resource scheduler.

Manufacturing = a DAG of operations over resources (CONCEPT.md §6). This is the
"program" the orchestrator runs and optimizes. Each Operation names the resource
(cell/machine) it runs on, how long it takes, what must finish first, and
optionally a callable that drives the real sim.

The scheduler is a resource-constrained list scheduler: each resource is a single
server (one op at a time); ops on different resources run in parallel. The
makespan is the cycle time. `sequential_makespan` is the no-overlap baseline —
the gap between them is the scheduling win.
"""

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass(frozen=True)
class Operation:
    name: str
    resource: str
    duration: float                 # seconds
    needs: tuple = ()               # names of ops that must finish first
    tool: Optional[str] = None
    action: Optional[Callable] = field(default=None, compare=False)


class OperationGraph:
    def __init__(self, ops):
        self.ops = {o.name: o for o in ops}
        self._validate()

    def _validate(self):
        for o in self.ops.values():
            for n in o.needs:
                if n not in self.ops:
                    raise ValueError(f"op '{o.name}' needs unknown op '{n}'")
        self.topo_order()  # raises on a cycle

    def topo_order(self):
        """Kahn's algorithm; raises ValueError on a cycle."""
        indeg = {n: 0 for n in self.ops}
        for o in self.ops.values():
            for _ in o.needs:
                indeg[o.name] += 1
        ready = sorted(n for n, d in indeg.items() if d == 0)
        order = []
        while ready:
            n = ready.pop(0)
            order.append(n)
            for m in self.ops.values():
                if n in m.needs:
                    indeg[m.name] -= 1
                    if indeg[m.name] == 0:
                        ready.append(m.name)
            ready.sort()
        if len(order) != len(self.ops):
            raise ValueError("operation graph has a cycle")
        return order


def schedule(graph: OperationGraph):
    """Return (schedule, makespan). schedule[name] = (start, finish, resource)."""
    res_free = defaultdict(float)
    finish = {}
    sched = {}
    for name in graph.topo_order():
        op = graph.ops[name]
        ready = max((finish[n] for n in op.needs), default=0.0)
        start = max(ready, res_free[op.resource])
        fin = start + op.duration
        sched[name] = (start, fin, op.resource)
        finish[name] = fin
        res_free[op.resource] = fin
    makespan = max((f for _, f, _ in sched.values()), default=0.0)
    return sched, makespan


def sequential_makespan(graph: OperationGraph) -> float:
    """No-overlap baseline: every op back-to-back on one server."""
    return sum(o.duration for o in graph.ops.values())


def ascii_gantt(sched, makespan, width=46) -> str:
    scale = (width / makespan) if makespan else 1.0
    rows = []
    for res in sorted({r for _, _, r in sched.values()}):
        items = sorted(((s, f, n) for n, (s, f, r) in sched.items() if r == res))
        for s, f, n in items:
            lead = int(round(s * scale))
            bar = max(1, int(round((f - s) * scale)))
            rows.append(f"{res:8} {n:16} |{' ' * lead}{'█' * bar}  {s:5.1f}–{f:5.1f}s")
    return "\n".join(rows)
