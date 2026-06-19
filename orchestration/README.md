# orchestration/

The operation-graph + scheduler — the core "software-defined" layer. A DAG of
physical operations over resources (machines, tools, fixtures); optimizing it
(sequencing, parallelism, cycle time) is the payload.

- `opgraph.py` — `Operation` / `OperationGraph`, the resource-constrained list
  `schedule()` (→ per-op start/finish + makespan = cycle time), a
  `sequential_makespan()` baseline, and an `ascii_gantt()`.
- `job.py` — a concrete multi-cell job (printer + bender + arm) embedding the
  Phase-3 `bend → present → shear` chain; two ops carry real sim actions.

Run it:

```bash
make opgraph        # schedule + Gantt + cycle time
make opgraph-run    # also execute the real sim actions (eject, shear) in order
make opgraph-check  # gate: precedence + resource limits + overlap win
```

Current result: overlapping the wire work under the 40 s print cuts a unit from
**65 s sequential to 52 s (20% faster)**. Next: tool-change-aware sequencing, a
multi-unit pipeline, and durations measured live from sim runs.
