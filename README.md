# Autonomous Explorer

A wheeled-rover learning project accompanying *Principles of Robot Autonomy*. We are building toward navigation, localization, mapping, exploration, and a return home—one reproducible milestone at a time.

<!-- Keep the cumulative gallery link and visual history when advancing milestones. Append new milestones to GALLERY.md without removing earlier figures. -->
**[Complete visual gallery: all 40 figures and seven animations](GALLERY.md)**

[Tracking](GALLERY.md#milestone-01-trajectory-tracking) · [Navigation](GALLERY.md#milestone-02-obstacle-aware-navigation) · [Localization](GALLERY.md#milestone-03-sensing-and-localization) · [SLAM](GALLERY.md#milestone-04-landmark-mapping-and-slam) · [Exploration](GALLERY.md#milestone-05-decision-making-and-exploration)

![An unsupervised mission continues through a sensor outage while a supervised mission waits and recovers](media/exploration/exploration_hero.gif)

**Milestone 05 · Decision-Making and Viewpoint Exploration.** The rover now selects observation goals online, plans checked routes from its estimated position, builds its landmark map with SLAM, and decides when to return home. A localization-health supervisor brakes, waits for fresh observations, and replans after recovery. Persistent observation loss produces an explicit abort.

Across three seeds, supervised nominal and temporary-outage missions complete with positive clearance audits and meet the actual home-arrival tolerances. Persistent-outage missions stop exploring and abort. The unsupervised baseline exposes a failure: it reports completion during persistent observation loss while actually ending **2.1–4.0 m from home**, with envelope violations. Mission status, actual arrival, and actual clearance are audited separately.

## Project status

| Milestone | Status | Evidence |
| :-- | :-- | :-- |
| 01 · Trajectory tracking | Complete | [Walkthrough and gallery](MILESTONE_01.md) |
| 02 · Obstacle-aware navigation | Complete | [Walkthrough and gallery](MILESTONE_02.md) |
| 03 · Sensing and localization | Complete | [Walkthrough and gallery](MILESTONE_03.md) |
| 04 · Landmark mapping and SLAM | Complete | [Walkthrough and gallery](MILESTONE_04.md) |
| 05 · Decision-making and viewpoint exploration | Complete | [Protocol](docs/exploration.md), [18 missions](results/exploration/), [verification](docs/exploration-verification.md) |
| 06 · Unknown-space occupancy mapping and exploration | Next | Sense obstacle geometry, build free/occupied/unknown space, and select goals from that evolving map |
| Later · Precise goal regulation | Planned | Stationary parking with final-pose regulation and stronger stopping guarantees |

Exploration here means choosing among supplied **candidate observation viewpoints** using estimated coverage and map uncertainty. The obstacle map remains supplied. Point-landmark positions remain initially unknown to SLAM. There is no obstacle occupancy learning or frontier discovery in this milestone.

## Reproduce it

Use [uv](https://docs.astral.sh/uv/) and the committed lockfile:

```bash
uv sync --frozen --extra dev
uv run --frozen explorer explore-reproduce --results build/exploration-reproduced/results --media build/exploration-reproduced/media
```

This runs three seeds × three observation conditions × two supervisor modes, then renders six figures and one GIF. Numerical result directories must be empty or new.

```bash
uv run --frozen explorer explore-verify --results results/exploration
uv run --frozen pytest -q
uv run --frozen ruff check src tests
uv run --frozen ruff format --check src tests
```

Verification reconstructs SLAM, goal utilities, plans, supervisor transitions, and requested controls from saved observations and applied commands. It separately recalculates metrics and physical clearance from actual saved states. Rendering uses the same saved evidence:

```bash
uv run --frozen explorer explore-render --results results/exploration --media build/exploration-rendered
```

`explore-simulate` saves numerical evidence without the gallery. [configs/exploration.json](configs/exploration.json) controls candidates, utility weights, motion limits, health thresholds, outages, and seeds. Its SLAM configuration resolves relative to that file; the complete configuration is embedded in the evidence. The curated gallery expects `nominal`, `temporary_outage`, and `persistent_outage` cases. Runtime dependencies remain NumPy, Matplotlib, and Pillow.

Earlier commands and evidence remain available:

```bash
uv run --frozen explorer verify --results results/milestone
uv run --frozen explorer plan-verify --results results/navigation
uv run --frozen explorer localize-verify --results results/localization
uv run --frozen explorer slam-verify --results results/slam
```

## What changed

- **Goal selection:** a deterministic utility combines unseen modelled observation cells, estimated landmark uncertainty, and checked route length. Every candidate score and selected goal is recorded. This is a greedy heuristic without a global optimality claim.
- **Online planning:** the planner connects the continuous pose estimate to a lattice through a checked segment, uses A* where needed, and validates all shortcut segments. Goals are replanned after recovery or a large pose correction.
- **Mission supervision:** `exploring → returning → complete`, with `braking → waiting → resume/abort` when landmark information is stale or position uncertainty grows. Fresh observations and recovery hysteresis govern resumption; the policy cannot read the outage schedule.
- **Zero-speed control:** a heading/speed waypoint controller permits braking, waiting, and turning at rest. The earlier flatness controller and all of its evidence remain intact. Low-speed arrival is distinct from precise parked-pose regulation.

## Measured outcomes

“Reports complete” is the mission's decision from estimated state. “Home audit” additionally requires actual final distance ≤0.30 m and actual absolute speed ≤0.05 m/s. “Clear audit” requires positive numerical interval clearance over the entire recorded history and can also hold for an aborted mission. Coverage is a modelled visibility footprint from estimated sensor poses, not measured unknown-space occupancy.

<!-- exploration-summary:start -->
| Case | Supervisor | Reports complete | Home audit | Aborted | Clear audit | Mean model coverage |
| :-- | :-- | --: | --: | --: | --: | --: |
| nominal | unguarded | 3/3 | 3/3 | 0/3 | 3/3 | 99.1% |
| nominal | guarded | 3/3 | 3/3 | 0/3 | 3/3 | 99.1% |
| temporary outage | unguarded | 0/3 | 0/3 | 3/3 | 2/3 | 77.3% |
| temporary outage | guarded | 3/3 | 3/3 | 0/3 | 3/3 | 99.1% |
| persistent outage | unguarded | 3/3 | 0/3 | 0/3 | 0/3 | 44.5% |
| persistent outage | guarded | 0/3 | 0/3 | 3/3 | 3/3 | 44.5% |
<!-- exploration-summary:end -->

The table is generated from [summary.md](results/exploration/summary.md); [metrics.json](results/exploration/metrics.json) retains each run. The temporary outage suppresses landmarks on **[14,22) s**. The persistent outage suppresses them from **14 s** onward. Both disturbed conditions add +0.12 rad/s yaw bias on **[14,18) s**, and all conditions have shared pre-generated plant and sensor noise.

Supervised nominal and temporary-outage completions end **0.114–0.167 m from actual home**, at actual absolute stored speeds **0.024–0.042 m/s**. Persistent-loss aborts occur at **29.85 s**, with actual absolute stored speeds below **0.004 m/s**. These are observed outcomes of three seeds, not a formal stopping or collision-avoidance guarantee. The model's translational disturbance can still move a rover whose stored speed is zero.

## Visual gallery

All empirical figures and replay frames use saved data. Amber is the baseline without health supervision; cyan is the supervised mission. The supplied obstacle map and candidate viewpoints remain explicit.

![Actual and estimated online mission paths during a temporary outage](media/exploration/01_online_missions.png)

**01 · Online missions.** Tinted cells show estimated model coverage. Goal selection, recovery, and return produce a mission trajectory rather than following a fixed loop. [SVG](media/exploration/01_online_missions.svg).

![Supervisor timelines show braking, waiting, recovery, returning, completion and abort](media/exploration/02_supervisor_states.png)

**02 · State histories.** Temporary loss leads to recovery; persistent loss leads to abort. Short logs end at their actual terminal decision.

![Recorded viewpoint utilities before and after new observation coverage](media/exploration/03_goal_utilities.png)

**03 · Goal utilities.** Selected goals follow the recorded candidate scores as model coverage and the landmark belief evolve. [SVG](media/exploration/03_goal_utilities.svg).

![Truth-based interval clearance through all three mission conditions](media/exploration/04_clearance.png)

**04 · Clearance audit.** Unsafe baseline histories remain visible. A completed mission does not establish clear motion. [SVG](media/exploration/04_clearance.svg).

![Landmark observation age and actual speed show the health intervention](media/exploration/05_localization_health.png)

**05 · Health intervention.** The supervisor uses accepted-observation age and covariance. Actual speed is shown afterward for evaluation.

![Per-seed model coverage and actual home error distinguish reported completion from arrival](media/exploration/06_completion_audit.png)

**06 · Completion audit.** A belief-based return can end metres from home. Aborted runs do not claim arrival; they remain included.

## Visual history

Earlier milestones remain part of the project gallery. The [complete visual gallery](GALLERY.md) displays every figure and animation with its original explanation, SVG links, and animation review frames.

### Milestone 01: Trajectory tracking

![Synchronized open-loop and feedback tracking under a shared yaw disturbance](media/hero.gif)

![Tracking replay synchronized with position error and control histories](media/diagnostic_replay.gif)

[All 11 tracking figures and both animations](GALLERY.md#milestone-01-trajectory-tracking) · [Full walkthrough](MILESTONE_01.md)

### Milestone 02: Obstacle-aware navigation

![Obstacle-aware open-loop and feedback navigation with clearance failures retained](media/navigation/navigation_hero.gif)

![Recorded A* and Dijkstra search expansions](media/navigation/search_replay.gif)

[All 11 navigation figures and both animations](GALLERY.md#milestone-02-obstacle-aware-navigation) · [Full walkthrough](MILESTONE_02.md)

### Milestone 03: Sensing and localization

![Exact-state, dead-reckoning and EKF feedback during a landmark outage](media/localization/localization_hero.gif)

[All six localization figures and the animation](GALLERY.md#milestone-03-sensing-and-localization) · [Full walkthrough](MILESTONE_03.md)

### Milestone 04: Landmark mapping and SLAM

![First-sighting mapping and joint EKF SLAM build landmark maps while moving](media/slam/slam_hero.gif)

[All six SLAM figures and the animation](GALLERY.md#milestone-04-landmark-mapping-and-slam) · [Full walkthrough](MILESTONE_04.md)

## Learn and inspect

Read the [mission protocol and transition rules](docs/exploration.md), [source notes](docs/exploration-sources.md), and [verification record](docs/exploration-verification.md). The book anchor is [Chapter 16 · Finite State Machines](https://porabook.com/book/finite-state-machines/).

| Location | Responsibility |
| :-- | :-- |
| [mission.py](src/autonomous_explorer/mission.py) | Belief-only utilities, checked plans, waypoint commands, supervisor and return logic |
| [exploration_evidence.py](src/autonomous_explorer/exploration_evidence.py) | Synthetic sensor/SLAM integration, mission replay, actual-state audits and checksums |
| [exploration_render.py](src/autonomous_explorer/exploration_render.py) | Saved-data mission gallery and animation |
| [tests/test_exploration.py](tests/test_exploration.py) | Goal changes, route checks, state transitions, zero speed, truth-free replay and honest completion |

Assumptions remain a supplied static obstacle map, known feature IDs, synchronous synthetic sensors, a local initial-pose prior, and kinematic actuation. Estimated coverage is a proxy. The health rules are heuristics, and the clearance audit remains conditioned on numerical integration endpoints without a formal global RK4 error enclosure. There is no hardware validation or exact stationary parking claim.

The next milestone is **unknown-space occupancy mapping and exploration**. The [optional CI template](docs/checks-workflow.yml) includes all five milestones. Original code and figures use the [MIT license](LICENSE); study sources are attributed separately.
