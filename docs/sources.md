# Source audit and attribution

Inspected on **2026-10-25**, before implementation. The original repository and its remote contained no commits or project files; there was no existing implementation to preserve. No applicable ancestor or repository `AGENTS.md` was present.

Primary study sources:

- [Principles of Robot Autonomy](https://porabook.com/), by Daniele Gammelli, Joseph Lorenzetti, Katie Luo, Gioele Zardini, and Marco Pavone (Cambridge University Press, 2026).
- [Official exercises index](https://porabook.com/exercises/).
- [StanfordASL/pora-exercises](https://github.com/StanfordASL/pora-exercises), pinned at [`02ee1421786b73ae4810b086cd1f59882ab972bc`](https://github.com/StanfordASL/pora-exercises/commit/02ee1421786b73ae4810b086cd1f59882ab972bc).

## Exact upstream files

| Source at the inspected revision | What was inspected |
| :-- | :-- |
| [ch01/exercises/nonholonomic_wheeled_robot_dynamics.ipynb](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch01/exercises/nonholonomic_wheeled_robot_dynamics.ipynb) | Three-state unicycle, speed/yaw inputs, unfinished step and rollout exercises |
| [ch01/exercises/dynamics.py](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch01/exercises/dynamics.py) | Base interface, 0.01 s default timestep, noisy flag |
| [ch01/numerical_integration.ipynb](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch01/numerical_integration.ipynb) | IVP example and Euler, midpoint, RK4 interfaces |
| [ch03/exercises/unicycle_trajectory_tracking.ipynb](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch03/exercises/unicycle_trajectory_tracking.ipynb) | Exercise 5; four-state extension; cubic constraints; 25 s, 501 samples; endpoint states; gains; unfinished controller |
| [ch03/exercises/unicycle_utils.py](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch03/exercises/unicycle_utils.py) | State properties, injection of speed/yaw offsets, per-interval sampled disturbances, simulation/logging conventions |
| [environment.yml](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/environment.yml) | Broad book environment; most packages unnecessary for this milestone |

[`upstream-audit.json`](upstream-audit.json) stores file hashes and the inspected revision. The upstream checkout used for study stayed outside this repository; no notebook, helper, or book PDF is committed.

## License inspection and implementation boundary

The inspected Git tree contained no LICENSE, COPYING, or NOTICE file, and the inspected Python/Markdown sources contained no license grant or copyright notice. The repository's public availability was not treated as a code-reuse license. We wrote our own dynamics, reference solve, controller, simulator, logging, tests, plotting theme, and animations from the mathematical specification. No upstream implementation was incorporated. This records what was found at the pinned revision, not a claim about future upstream licensing. Our [MIT license](../LICENSE) covers the original project work only.

## Interface and mathematical discrepancies resolved

1. **Controller signature.** Notebook cell 10 defines `compute_control(self, s: State, t: float)`. The helper's executable feedback branch calls `compute_control(x[0], x[1], x[2], x[3], t)`; even its prose mentions a different scalar signature. These cannot be interchanged. We use `Feedback.command(actual: State, desired: ReferenceSample)` everywhere, with the simulator sampling the reference once per control interval. End-to-end tests exercise this exact interface.
2. **Control-array length.** The helper describes N−1 inputs, allocates N controls, and leaves the last row unused. We save N state/reference samples and exactly N−1 requested/applied controls and disturbances. There is no fictitious final command.
3. **Noise injection.** Despite calling the perturbation “control noise,” the helper adds its first component to translational **speed**, not to acceleration. We preserve this distinction explicitly: $d_v$ [m/s] enters translation, and $d_\omega$ [rad/s] enters heading rate. The exercise uses `sqrt(0.2)` as the standard-deviation multiplier. Our seed-7 case uses that numeric standard deviation independently for both channels with their own units, NumPy `default_rng`/PCG64, and a single shared array for each controller pair. It is informed by upstream, not a claim to reproduce its unspecified global RNG state.
4. **Bounds.** The notebook clips only feedback yaw rate at 1 rad/s. Our default runs use unbounded commands for both controllers to study the stated inversion directly. Optional symmetric limits, if configured as `[a_max, omega_max]`, are applied identically to both, with saturation and requested/applied inputs logged.
5. **Near zero speed.** The scaffold suggests avoiding a singularity below 0.0001 m/s without supplying the implementation. We make that threshold an explicit forward-domain guard and retain a stopped-start failure. We do not invent a regularized controller and present it as the original law.
6. **State naming.** Upstream uses `th`; our named field is `theta`. Both refer to the fourth array entry. Chapter 1's basic three-state unicycle is a prerequisite; this milestone consistently uses Chapter 3's extended `(x, y, v, theta)` state and `(a, omega)` input.
7. **Example numbering.** The notebook refers to Example 3.2.2; the current [Chapter 3 web text](https://porabook.com/book/closed-loop-control/) calls the dynamically extended-unicycle PD example **3.1.2** and retains it as **Exercise/Problem 5**. We identify the topic and exercise explicitly rather than silently repeating the stale example number.

## Study map

| Book topic | Project functions | Evidence |
| :-- | :-- | :-- |
| [Chapter 1, kinematic unicycle; §1.4 numerical integration](https://porabook.com/book/modeling-robot-dynamics/) | `State`, `derivative`, `step`, `constant_turn_exact` | Figures 01–03, analytical-motion tests |
| [Chapter 2, differential flatness](https://porabook.com/book/open-loop-control/) | `CubicReference`, `sample`, `minimum_speed`, `invert_acceleration` | Figure 04, endpoint/derivative/domain tests |
| [Chapter 3, extended-unicycle PD tracking; Exercise 5](https://porabook.com/book/closed-loop-control/) | `OpenLoop`, `Feedback.command`, `paired_runs` | Figures 05–11, both GIFs, metrics, paired/replay tests |

```bibtex
@book{pora,
  title = {Principles of Robot Autonomy},
  author = {Gammelli, Daniele and Lorenzetti, Joseph and Luo, Katie
            and Zardini, Gioele and Pavone, Marco},
  year = {2026},
  publisher = {Cambridge University Press}
}
```
