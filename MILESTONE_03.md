# Milestone 03 · Sensing and Localization

Archived milestone walkthrough and gallery. See the [current project status](README.md) for the completed landmark SLAM milestone.

A wheeled-rover learning project accompanying *Principles of Robot Autonomy*. We are building toward navigation, localization, mapping, exploration, and a return home—one reproducible milestone at a time.

![Exact-state, dead-reckoning, and EKF feedback during a landmark sensor outage](media/localization/localization_hero.gif)

**Milestone 03 · Sensing and Localization.** The rover now drives from an estimated state. Noisy speed and identified-landmark range/bearing observations feed a four-state extended Kalman filter (EKF). The existing planner, motion model, actuator limits, and tracking controller continue from [Milestone 02](MILESTONE_02.md).

With landmarks available, estimated-state feedback achieves about **4.3 cm localization RMS** and **3.6–3.7 cm tracking RMS**, averaged over three seeds. Its smallest interval clearance bound is **0.4445 m**. During a 12-second landmark outage overlapping an unmodeled yaw bias, all three EKF-driven executions violate the rover envelope. The animation and logs retain those failures; localization uncertainty does not protect this controller from obstacles.

## Project status

| Milestone | Status | Evidence |
| :-- | :-- | :-- |
| 01 · Trajectory tracking foundations | Complete | [Walkthrough and original gallery](MILESTONE_01.md) |
| 02 · Obstacle-aware navigation | Complete | [Walkthrough and original gallery](MILESTONE_02.md) |
| 03 · Sensing and localization | Complete | [Protocol](docs/localization.md), [27 saved runs](results/localization/), [verification](docs/localization-verification.md) |
| 04 · Mapping and SLAM | Next | Estimate motion and initially unknown landmark positions jointly |
| Later · Exploration and return home | Planned | Goal selection, new-map replanning, and return behavior |
| Later · Goal regulation | Planned | Explicit stop/park transition through the controller's zero-speed singularity |

The previous milestone was already complete. Its six executions, planning failures, clearance audits, and gallery remain intact.

## Reproduce it

Use [uv](https://docs.astral.sh/uv/) and the committed lockfile:

```bash
uv sync --frozen --extra dev
uv run --frozen explorer localize-reproduce --results build/localization-reproduced/results --media build/localization-reproduced/media
```

The experiment plans a route in the same known world, certifies its reference, draws three independent noise streams per seed, runs all 27 executions, saves observations and state estimates, and renders six figures and one GIF. Results directories must be empty or new.

Verify the saved runs, replay every estimator from recorded observations and applied commands, and recalculate metrics and clearance:

```bash
uv run --frozen explorer localize-verify --results results/localization
uv run --frozen pytest -q
uv run --frozen ruff check src tests
uv run --frozen ruff format --check src tests
```

Render from saved evidence without rerunning the plant, planner, or sensor simulation:

```bash
uv run --frozen explorer localize-render --results results/localization --media build/localization-rendered
```

`localize-simulate` saves numerical evidence without rendering. `--config configs/localization.json` selects the [sensor, filter, disturbance, and seed configuration](configs/localization.json). Its navigation configuration path resolves relative to the configuration file; the full resolved navigation configuration is embedded in each result set. Sensor periods are integer multiples of the 0.02 s integration step. The curated gallery expects the `yaw_bias` and `landmark_outage` cases.

NumPy, Matplotlib, and Pillow remain the only runtime dependencies. Without uv, install in a virtual environment with `python -m pip install -e '.[dev]'`, then use `explorer` and `pytest` directly. Both earlier milestones retain their original CLI commands:

```bash
uv run --frozen explorer verify --results results/milestone
uv run --frozen explorer plan-verify --results results/navigation
```

## What changed

- **Sensors:** speed at 10 Hz; range/bearing to known landmarks at 5 Hz; 6 m maximum range; full 360° field of view; static obstacles block sight lines. Measurement standard deviations are 0.025 m/s, 0.05 m, and 0.015 rad. Landmark identities and positions are supplied.
- **Estimator:** EKF over `(x, y, v, theta)`, with Jacobians of the actual RK4 transition, wrapped bearing innovations, and Joseph covariance updates. Predictions use applied commands. The estimator receives no actual state or hidden plant disturbance.
- **Feedback:** three modes use the same tracking law and actuator limits: exact-state feedback (`oracle`), command-only dead reckoning, and EKF feedback. Only the explicit oracle baseline can use actual state. It also runs a shadow EKF for evaluation.
- **Evidence:** three seeds × three conditions × three feedback modes. Each seed shares pre-generated plant and sensor noise across comparisons. Observation values and visibility can differ because the rover follows different trajectories.

## Measured outcomes

These are arithmetic means of per-run RMS values, not pooled-sample estimates. “Clear” means a completed execution with a positive numerical interval clearance bound. All 27 integrations completed; several completed unsafely.

<!-- localization-summary:start -->
| Case | Feedback state | Mean localization RMS [m] | Mean tracking RMS [m] | Worst interval clearance [m] | Clear runs |
| :-- | :-- | --: | --: | --: | --: |
| nominal | oracle | 0.0420 | 0.0037 | 0.4636 | 3/3 |
| nominal | dead reckoning | 0.5814 | 0.5811 | -0.0331 | 2/3 |
| nominal | ekf | 0.0432 | 0.0360 | 0.4445 | 3/3 |
| yaw bias | oracle | 0.0421 | 0.0067 | 0.4636 | 3/3 |
| yaw bias | dead reckoning | 1.2012 | 1.2010 | -1.1190 | 0/3 |
| yaw bias | ekf | 0.0433 | 0.0373 | 0.4445 | 3/3 |
| landmark outage | oracle | 0.3497 | 0.0067 | 0.4636 | 3/3 |
| landmark outage | dead reckoning | 1.2012 | 1.2010 | -1.1190 | 0/3 |
| landmark outage | ekf | 0.7065 | 0.7879 | -0.8814 | 0/3 |
<!-- localization-summary:end -->

Generated from [summary.md](results/localization/summary.md); [outcomes.md](results/localization/outcomes.md) and [metrics.json](results/localization/metrics.json) retain every run. The oracle's localization column measures its shadow EKF; the oracle controller still receives exact state. Dead reckoning ignores both sensors. Its yaw-bias and outage runs are identical because loss of an unused sensor cannot change its commands.

Every case has sampled plant speed/yaw disturbances. The yaw-bias cases additionally receive +0.12 rad/s on **[19.14, 23.14) s**. The outage suppresses landmark observations on **[18, 30) s**, while the speed sensor continues. Landmark reacquisition also depends on range and occlusion. The fixed bias is intentionally absent from the EKF state and violates its zero-mean process assumption.

## Visual gallery

All empirical figures and replay frames use saved evidence. Navy, cyan, amber, the oriented rover, and equal spatial aspect ratios continue from the previous milestones.

![Landmark sensing geometry and recorded observation availability](media/localization/01_sensing.png)

**01 · Sensing.** Which supplied landmarks can the rover actually observe? Range, occlusion, and the sensor schedule determine availability. [SVG](media/localization/01_sensing.svg).

![Actual paths with available landmarks and during an outage, including collisions](media/localization/02_execution.png)

**02 · Estimated-state execution.** Landmarks correct drift in the left panel. During the outage, the estimated trajectory can look reasonable while the actual rover crosses an obstacle. Red crosses mark first envelope overlaps. [SVG](media/localization/02_execution.svg).

![Localization and tracking errors for three feedback modes](media/localization/03_errors.png)

**03 · Two errors.** Estimation error compares estimate to truth; tracking error compares truth to the reference. Both are measured independently.

![Truth-based clearance audits retain negative unsafe values](media/localization/04_clearance.png)

**04 · Clearance.** The existing disk-envelope audit evaluates actual motion. It does not stop, replan, or alter a controller. [SVG](media/localization/04_clearance.svg).

![EKF uncertainty and normalized estimation error through the landmark outage](media/localization/05_uncertainty.png)

**05 · Confidence under model mismatch.** Position errors leave the declared marginal uncertainty bands during the outage. NEES and the recorded visibility history expose the mismatch; covariance is not presented as calibrated safety evidence.

![Individual seed outcomes, means and ranges for localization and tracking](media/localization/06_seed_comparison.png)

**06 · Reproducibility across seeds.** Individual runs remain visible beside means and min–max ranges. Three fixed seeds are a small study, not a probabilistic robustness guarantee.

## Learn and inspect

Read the [experiment protocol and equations](docs/localization.md), [primary sources](docs/localization-sources.md), and [verification record](docs/localization-verification.md). This milestone connects the book's [sensor chapter](https://porabook.com/book/robot-sensors/), [approximate filters](https://porabook.com/book/approximate-filters/), and [robot localization](https://porabook.com/book/robot-localization/).

| Location | Responsibility |
| :-- | :-- |
| [sensors.py](src/autonomous_explorer/sensors.py) | Synthetic sensor boundary, visibility, range/bearing model and Jacobian |
| [localization.py](src/autonomous_explorer/localization.py) | RK4 transition sensitivities and four-state EKF |
| [localization_evidence.py](src/autonomous_explorer/localization_evidence.py) | Causal simulation, comparisons, logs, replay, metrics, and checksums |
| [localization_render.py](src/autonomous_explorer/localization_render.py) | Saved-data figures and animation |
| [tests/test_localization.py](tests/test_localization.py) | Numerical checks, occlusion, angle wrapping, covariance, causal replay, and failures |

Current assumptions remain a known static obstacle map, known landmark positions/IDs, synchronous ideal timestamps, Gaussian sensor noise, conservative collision geometry, and kinematic actuation. There is no learned map, data-association system, runtime collision avoidance, stopping policy, or hardware validation. The endpoint remains moving. The next milestone is **mapping and SLAM**; the outage results also motivate a later localization-health and replanning policy.

The optional [CI template](docs/checks-workflow.yml) includes checks for all three milestones and remains outside active GitHub workflows. Original code and figures use the [MIT license](LICENSE); study sources are attributed separately.
