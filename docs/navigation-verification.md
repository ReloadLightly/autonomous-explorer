# Milestone 02 verification record

Verified locally on 2026-10-25 with Python 3.13.5, NumPy 2.5.4, Matplotlib 3.11.2, Pillow 12.3.0 and pytest 9.1.1, using the existing lockfile. No new runtime dependency was added. Upstream revision and licensing were inspected before implementation; see the [source audit](navigation-sources.md).

## Automated checks

The final suite passed **62 tests, with no skips**: all 35 original tests plus 27 planning/navigation checks, including parameterized geometry cases. Commands executed:

```bash
uv run --frozen pytest -q
uv run --frozen ruff check src tests
uv run --frozen ruff format --check src tests
uv run --frozen explorer plan-verify --results results/navigation
uv run --frozen explorer verify --results results/milestone
```

New checks establish:

- Continuous segment/box intersections, including parallel edges, closed contacts, zero-length segments and thin obstacles between free endpoints.
- The full rover body/wheel bounding rectangle fitting the conservative radius at all sampled headings, with dimensions checked against the original renderer.
- Obstacle inflation, inward wall offsets, signed clearance, and diagonal grid-corner rules.
- A* matching Dijkstra's optimal graph cost, deterministic paths/expansions, and explicit off-grid/invalid endpoint rejection.
- Shortcuts preserving endpoints and validating full replacement segments.
- A counterexample where unvalidated corner rounding collides, while the free-hull certificate shrinks the curve safely.
- C1 joins, positive endpoint speeds, analytic derivatives, and continuous speed/acceleration/yaw extrema respecting the timing bounds.
- Reversal/zero-speed corner rejection, with no hidden regularization.
- Identical paired references, grids, exogenous arrays and actuator limits, plus exact seed-7 replay of states and controls.
- Unsafe execution retained with a distinct `envelope_violation` status despite completed numerical integration.
- Sealed-world/narrow-passage failures saved without rover execution; a blocked primary configuration also saves an explicit planning failure.
- A between-sample collision counterexample caught by the interval clearance audit even with both endpoints clear.
- Metrics and clearance recomputation, evidence-directory preservation, saved-input/media hashes and the README table matching the generated outcomes.
- Rendering with planner/simulator entry points replaced by failures, proving it uses saved evidence.

All original physics, integration convergence, cubic constraints, control algebra, singularity handling, paired replay, GIF palette/timing and Milestone 01 provenance tests still pass. The CI definition remains a [documented template](checks-workflow.yml): the available GitHub token lacks `workflow` scope, so no remote CI success is claimed.

## Independent clean reproduction

The documented command ran with `build/navigation-reproduced/` initially absent:

```bash
uv run --frozen explorer plan-reproduce --results build/navigation-reproduced/results --media build/navigation-reproduced/media
```

The first invocation needed network access for uv's editable-project build dependency; retrying with that access completed the full command. This was environment setup, not a changed experiment. Relative to the earlier independent run, **120 saved arrays matched exactly**, as did the complete planning summary and all metric values. Numerical source fingerprints match the delivered sources. Both searches, the resolution study, explicit failure cases, reference construction, all six executions, and the gallery were produced by this route.

The saved results were promoted to `results/navigation/`, and the inspected gallery to `media/navigation/`. The separate route was also executed:

```bash
uv run --frozen explorer plan-render --results results/navigation --media build/navigation-rendered
```

It reads verified saved evidence and does not mutate the numerical results. The renderer's entry-point tests additionally prevent planning or simulation calls. Platform-dependent media encodings and SVG identifiers need not be byte-identical on regeneration; numerical arrays, metrics and saved-data provenance are the reproducibility target.

## Preserving Milestone 01

Every original numerical evidence file and curated image/GIF was checked against its stored hash and remains unchanged. The existing robot dynamics, controller, simulator and plotting theme are unchanged. The shared animation exporter gained optional review times and palette accents with unchanged defaults; all its original tests pass. The CLI gained `plan-*` commands while retaining the original tracking commands and defaults.

The complete earlier README is preserved as [MILESTONE_01.md](../MILESTONE_01.md). The original walkthrough/report links now point to that archive where needed. The earlier README-table test follows the archived table; the new table has its own saved-data provenance test.

## Rendered-media inspection

All eleven new PNG figures were opened and reviewed, including planning-space boundaries, exact edge/path differences, timed-reference joins, execution failures, common snapshot bounds, error scales, requested/applied inputs, failure cases and resolution axes. Selected figures also have SVG exports. Review moved the envelope-violation annotation away from the path, protected inflation labels from background lines, separated the reference-speed legend from its curve, and moved the clearance legend away from the low-clearance trace.

Both GIFs were decoded. Initial, disturbance-time/intermediate, and final frames were inspected for orientation, trails, ghost pose, fixed bounds, clock/expansion labels, palette integrity and persistent violation status. Each contains **120 frames and 10.8 seconds** including the 0.8 s final hold. The exporter reserves the additional warning accent so it survives GIF quantization even when absent in the initial frame. Search playback uses a common expansion-count budget, not normalized progress or wall-clock time.

The navigation hero is **398,109 bytes**; the search replay is **332,929 bytes**. All new curated media totals about **3.1 MB**, and numerical evidence about **1.44 MB**. Six full-size review PNGs accompany the GIFs. The original media remains separate and unchanged. No PDF, virtual environment, cache, or scratch output is committed.

## Outcomes and limits

A* and Dijkstra both found graph cost 22.363961 m; their expansion counts were 660 and 868. The 54.66 s rounded reference passed continuous geometric/kinematic certificates. All six numerical integrations completed without saturation or invalid-speed events. All three feedback executions had positive interval clearance audit bounds. The disturbed open-loop execution did not: its first disk overlap is at 28.76 s, with 553 sampled envelope violations and 404 centre-in-obstacle/outside-map samples. It is retained without contact response or replanning.

The sealed world and narrow passage produced explicit rover-planning failures and no rover motion. The seed-7 open-loop execution remained clear despite larger tracking error; feedback is not presented as universally necessary or sufficient for safety from a single example.

These results assume a known static world, exact state, conservative disk geometry and the original kinematic abstraction. The interval bound is conditioned on numerical endpoints and omits a formal global RK4 error enclosure. No runtime collision-avoidance guarantee, sensor/localization capability, map building, robust disturbance guarantee, parking, or hardware validation is claimed. The endpoint remains moving.
