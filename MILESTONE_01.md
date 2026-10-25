# Autonomous Explorer

A wheeled-rover learning project accompanying *Principles of Robot Autonomy* by Gammelli, Lorenzetti, Luo, Zardini, and Pavone. The rover will eventually navigate, localize, map unfamiliar surroundings, explore, and return home. The first milestone builds the motion and control foundation.

![Synchronized open-loop and feedback rover execution with a shared yaw disturbance](media/hero.gif)

**Milestone 01 · Trajectory Tracking Foundations.** An extended unicycle follows a cubic reference for 25 seconds. Open loop executes a precomputed command sequence; feedback uses exact simulator state to correct deviations. The comparison above applies the same +0.18 rad/s yaw disturbance from 8–12 seconds to both robots. Ghosts show the desired pose; trails reveal executed history. Playback is approximately 2.5× simulation time.

This release includes working dynamics and RK4 integration, flatness-based reference generation and PD tracking, three paired experiments, numerical logs, eleven distinct figures, and two GIFs rendered from those logs. The target speed at 25 seconds is **0.5 m/s**: this is finite-time tracking, not parking. Sensing, localization, maps, and autonomous exploration remain future milestones.

## Reproduce it

Use Python 3.10+; the committed `uv.lock` records the environment used for this release (Python 3.13.5). From the repository root, with [uv](https://docs.astral.sh/uv/) installed:

```bash
uv sync --frozen --extra dev
```

One command runs all experiments, computes metrics from saved logs, and renders the complete gallery into new output directories:

```bash
uv run --frozen explorer reproduce --results build/reproduced/results --media build/reproduced/media
```

Rendering alone reads existing logs and checks their hashes; it never reruns the simulator:

```bash
uv run --frozen explorer render --results results/milestone --media build/rendered
```

Run the tests and verify the committed evidence:

```bash
uv run --frozen pytest -q
uv run --frozen explorer verify --results results/milestone
```

Results directories must be empty or new, to preserve earlier runs. Choose a new path for another experiment. Configuration lives in [`configs/milestone.json`](configs/milestone.json): endpoints, duration, timestep, gains, speed threshold, optional shared actuator limits, disturbances, seed, and playback settings. For a changed configuration, use `--config path/to/config.json` with `reproduce` or `simulate`. The curated gallery focuses on the `yaw_bias` comparison; the diagnostic disturbance figure also shows `seeded`. Keep those scenario names when adapting this gallery.

Without uv, `python3 -m venv .venv`, activate it, and run `python -m pip install -e '.[dev]'`; the same `explorer` and `pytest` commands then work. This installs only NumPy, Matplotlib, Pillow, and development tools. SciPy is unnecessary for the explicit RK4 implementation. The book's full environment is not required.

## Measured outcomes

All six runs completed on the same 501-point time grid, with identical initial states and paired disturbances. The table is generated from [`metrics.json`](results/milestone/metrics.json); [`outcomes.md`](results/milestone/outcomes.md) contains the reproducible Markdown version. RMS is over all saved state samples, including both endpoints.

<!-- outcomes:start -->
| Scenario | Controller | RMS position [m] | Maximum [m] | Final [m] |
| :-- | :-- | --: | --: | --: |
| nominal | open loop | 0.0573 | 0.0840 | 0.0733 |
| nominal | feedback | 0.0007 | 0.0015 | 0.0008 |
| yaw bias | open loop | 2.2768 | 3.9395 | 3.5877 |
| yaw bias | feedback | 0.0195 | 0.0560 | 0.0008 |
| seeded | open loop | 3.5510 | 6.2572 | 5.4747 |
| seeded | feedback | 0.0743 | 0.2468 | 0.0888 |
<!-- outcomes:end -->

Feedback reduced error in these three declared cases. This is one stochastic realization, not a general performance guarantee. The nominal open-loop error comes mainly from holding sampled feedforward commands for 0.05 s; the continuous reference and a held-input execution are different trajectories. Feedback also corrects that discretization mismatch.

No main run hit the invalid-speed domain; the smallest feedback speed was 0.0902 m/s in the seeded case. There are no actuator limits in this configuration, so saturation counts are zero. A separate [zero-speed probe](results/milestone/zero_speed_probe.json) deliberately fails at t = 0 and is retained. [Detailed results and limitations](docs/experiment.md) include speed/heading errors, separate acceleration/angular-rate effort, and sampling assumptions.

## Visual gallery

The same navy, pale-blue, cyan, and amber theme and the same two-wheel rover geometry appear throughout. Cyan solid denotes feedback, amber dashed denotes open loop, and pale dotted denotes the reference. Spatial scenes use equal aspect ratios; paired scenes share fixed bounds. Figure 01 is a labelled conceptual diagram. All empirical plots use saved data.

### Model and numerical foundations

![Rover state, coordinate frames, velocity, and controls](media/01_robot_geometry.png)

**01 · Robot geometry and state.** The axle centre defines (x, y), heading rotates body axes into the world, and velocity follows the forward body axis. The body and wheels provide orientation cues; no sensors are implied. [SVG](media/01_robot_geometry.svg).

![Straight, turning, circular, and composed motions from the implemented model](media/02_motion_primitives.png)

**02 · Motion primitives.** Four saved dynamics rollouts show how straight travel, constant turning, a full circle, and a sequence of acceleration/turn commands build motion. Each panel uses the same world scale.

![Euler and RK4 final-position error versus integration timestep](media/03_integration_accuracy.png)

**03 · Integration accuracy.** Error against an analytical constant-turn solution falls approximately linearly for Euler and with the fourth power of timestep for RK4. This isolates integration error from controller sampling. [SVG](media/03_integration_accuracy.svg).

![Cubic reference path with endpoint headings, speed, and curvature](media/04_reference_construction.png)

**04 · Reference construction.** Endpoint positions and Cartesian velocities determine two cubics. Their derivatives determine speed, heading, and controls through differential flatness. The initial downward motion and final overshoot follow from the prescribed downward endpoint headings. [SVG](media/04_reference_construction.svg).

### Execution and recovery

![Full reference, open-loop, and feedback trajectories with disturbance boundary markers](media/05_trajectory_comparison.png)

**05 · Full trajectory comparison.** Diamonds locate the reference at disturbance onset and removal. The actual final bodies reveal how much orientation error open loop retains. [SVG](media/05_trajectory_comparison.svg).

![Four recorded snapshots before, during, at the end of, and after the disturbance](media/06_temporal_progression.png)

**06 · Temporal progression.** The same run at 6, 10, 12, and 20 seconds. Histories stop at each snapshot time; none is an independently simulated scene. The disturbance interval is half open, so it is inactive at 12 seconds.

![Rover replay synchronized with position error, acceleration, and yaw-rate histories](media/diagnostic_replay.gif)

**Diagnostic replay.** A moving cursor ties physical motion to error and control activity. The vertical range stays fixed and the traces reveal only recorded history. [Representative frame during the disturbance](media/diagnostic_replay_during.png).

### Diagnose the behavior

![Time histories of position-error norm and signed components](media/07_position_error.png)

**07 · Position tracking.** Open loop keeps its original commands after the yaw disturbance rotates the rover; position error accumulates. Feedback introduces a compensating turn and reduces the error after removal. Shading marks 8–12 s. [SVG](media/07_position_error.svg).

![Speed, continuous heading, and wrapped heading error](media/08_state_tracking.png)

**08 · State tracking.** Heading is unwrapped for continuous display, while the heading error uses the shortest signed angle in [−π, π). The nonzero endpoint speed is visible.

![Requested acceleration and yaw rate with applied-command markers](media/09_control_activity.png)

**09 · Control activity.** Requested and applied inputs coincide here. Markers independently show applied samples; the continuous yaw-bias correction primarily changes angular-rate effort. The disturbance is added downstream and is not part of the commanded yaw rate.

![Actual delivered bounded and seeded disturbance signals with physical units](media/10_disturbance_profile.png)

**10 · Disturbance profile.** Actual held samples from the logs, including the seed-7 Gaussian speed offset and yaw-rate offset. These are finite-duration sampled disturbances, not a timestep-independent stochastic differential equation.

![RMS and final-position errors across all three declared scenarios](media/11_scenario_outcomes.png)

**11 · Scenario outcomes.** All three comparisons appear together. Logarithmic axes preserve the nominal errors rather than rounding them to zero. [SVG](media/11_scenario_outcomes.svg).

## Learn from the implementation

Read the [learning walkthrough](docs/walkthrough.md) in the order physical problem → equations → numerical method → code → observed behavior → limitations. The [source audit](docs/sources.md) pins exact upstream links, records the license inspection, and explains the controller-signature inconsistency and other differences from the exercise scaffolds.

| Location | Responsibility |
| :-- | :-- |
| [`model.py`](src/autonomous_explorer/model.py) | Named states, derivatives, RK4/Euler steps, analytical benchmark |
| [`reference.py`](src/autonomous_explorer/reference.py) | Cubic constraints and reference derivatives; continuous minimum-speed check |
| [`control.py`](src/autonomous_explorer/control.py) | State-independent open loop and flatness-based PD feedback |
| [`simulation.py`](src/autonomous_explorer/simulation.py) | Shared exogenous signals, sampled control, integration, status |
| [`evidence.py`](src/autonomous_explorer/evidence.py) | Saved arrays, metrics, hashes, primitives and accuracy study |
| [`render.py`](src/autonomous_explorer/render.py), [`theme.py`](src/autonomous_explorer/theme.py) | Saved-data gallery and shared rover appearance |
| [`results/milestone/`](results/milestone/) | Compact numerical evidence and complete configuration |
| [`tests/`](tests/) | Physics, algebra, pairing, reproducibility, failures, and provenance checks |

The experiment is anchored in **Chapter 3, Exercise 5: Extended Unicycle Trajectory Tracking Control**, with Chapter 1 modeling/integration and Chapter 2 differential flatness as prerequisites. See the [book](https://porabook.com/), [exercise index](https://porabook.com/exercises/), and [upstream code](https://github.com/StanfordASL/pora-exercises). No book PDF or upstream exercise implementation is vendored here.

## Next milestones

1. **Chapter 4 — motion planning:** collision geometry, obstacles, and a planner that supplies feasible references for this same rover.
2. **Sensing and localization:** explicit sensor models, noise, and an estimated state in place of the exact simulator state.
3. **SLAM:** build and update a map while estimating rover motion.
4. **Decision-making:** choose exploration goals, track progress, and plan a return home.

Milestone 01 stops at motion and trajectory tracking. It has no map, uncertainty estimate, perception model, learned policy, or real hardware validation.

Original project code and figures are provided under the [MIT license](LICENSE). Attribution to the study sources is separate; their absent license grant is not treated as permission to redistribute their code.
