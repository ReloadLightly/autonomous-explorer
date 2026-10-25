# Experiment protocol, results, and evidence

The scenario configuration was chosen before interpreting the runs and is committed in [`configs/milestone.json`](../configs/milestone.json). The source and mathematical specification are described in the [audit](sources.md) and [walkthrough](walkthrough.md). This is a small controlled simulation study, not hardware validation or a robustness benchmark.

## Declared protocol

All comparisons start at `(x, y, v, theta) = (0, 0, 0.5, -pi/2)` and track the same cubic reference ending at `(5, 5, 0.5, -pi/2)`. Both controllers use a 25 s horizon, 0.05 s control/integration intervals, 501 saved state samples, and 500 held commands. The feedback gains are `Kp = (2, 2) s^-2`, `Kd = (2, 2) s^-1`. These match the upstream example gains and were not tuned against these results. The inversion's forward-speed threshold is 0.0001 m/s. Default actuator limits are null for both controllers.

| Case | Delivered disturbance | Seed | Interpretation |
| :-- | :-- | :-- | :-- |
| `nominal` | Both channels identically zero | none | Integration and command-sampling baseline |
| `yaw_bias` | $d_\omega=+0.18$ rad/s for $8\le t<12$ s; $d_v=0$ | none | Bounded, interpretable persistent turn error |
| `seeded` | Independent zero-mean Gaussian $d_v$ [m/s] and $d_\omega$ [rad/s], each with numeric standard deviation $\sqrt{0.2}\approx0.4472136$ in its respective units | 7 | One upstream-informed sampled perturbation realization |

The bias adds 0.72 rad to uncorrected heading over four seconds. The seeded signals act over the full horizon. `numpy.random.default_rng(7)` uses PCG64 in the recorded environment and draws one `(500, 2)` array in row-major order. The **same pre-generated array** is passed to both controllers, without RNG calls in the dynamics or RK stages. It is zero-order held over each half-open interval. At 12 s the bias is inactive.

The injection is downstream of applied commands: $d_v$ offsets the speed used in x/y derivatives and $d_\omega$ offsets heading rate. The v-state derivative remains the applied acceleration. The controller receives exact x, y, stored v, and heading, not the exogenous values. Controls update only at saved timestamps; RK4 stages reuse each held command and disturbance.

The Gaussian distribution is unbounded; only the yaw-bias case is bounded. Since the standard deviations are held fixed per sample, changing dt changes the cumulative stochastic perturbation. This is not continuous white noise and does not have timestep-invariant SDE scaling. Seed 7 was specified as the reproducible demonstration, not selected from an ensemble after inspecting animations.

## Observed results

The [Milestone 01 table](../MILESTONE_01.md#measured-outcomes) reports position errors, generated from [`metrics.json`](../results/milestone/metrics.json). Additional metrics are:

| Case / controller | RMS speed [m/s] | RMS wrapped heading [rad] | $\int a^2dt$ [m²/s³] | $\int\omega^2dt$ [rad²/s] |
| :-- | --: | --: | --: | --: |
| Nominal / open loop | 0.004661 | 0.008908 | 0.162022 | 2.951900 |
| Nominal / feedback | 0.000209 | 0.001335 | 0.162249 | 2.963884 |
| Yaw bias / open loop | 0.004661 | 0.550502 | 0.162022 | 2.951900 |
| Yaw bias / feedback | 0.000367 | 0.018290 | 0.162274 | 3.088469 |
| Seed 7 / open loop | 0.004661 | 0.999994 | 0.162022 | 2.951900 |
| Seed 7 / feedback | 0.050218 | 0.103473 | 0.315680 | 4.490743 |

Acceleration and angular-rate effort remain separate because they have different units. These are input-effort summaries, not battery energy, motor work, or a combined optimal-control objective. No arbitrary sum of them is reported.

All six runs completed. No saturation or invalid-domain event occurred in these comparisons. The continuous reference minimum speed is 0.1373606 m/s (first minimum at about 3.14796 s; symmetry gives the other at about 21.85204 s). Minimum actual feedback speeds were approximately 0.1367342, 0.1367311, and 0.0901823 m/s in nominal, yaw-bias, and seeded cases. The first two match closely because the bias occurs away from the slow endpoint turns.

Nominal feedforward execution has 0.0573 m RMS error despite matching the initial state. Its held commands approximate the controls of the continuous cubic: sampling causes drift that RK4 cannot remove. Feedback corrects that small drift too. Under the bounded yaw bias, open-loop heading stays rotated after the perturbation ends; feedback counters the rotation and removes most position error. In the seeded case feedback uses more acceleration and angular-rate effort and retains 0.0888 m final position error. It does not reject all random disturbances perfectly.

The separate [`zero_speed_probe`](../results/milestone/zero_speed_probe.json) starts feedback at zero speed. It fails at t = 0 with one invalid-domain event, retains one state and zero commands, and has no claimed full-horizon final error. This failure is deliberately outside the main protocol and is not included in the comparison table. No failed case was discarded from the declared three comparisons.

## Metric definitions

For every saved sample k, position error is $e_k=p_k-p_{d,k}$ and its norm is $r_k=\|e_k\|_2$.

- RMS position: $\sqrt{\frac1N\sum_k r_k^2}$, over all state samples, including both endpoints. This is a sample statistic rather than a trapezoid-weighted continuous-time RMS.
- Maximum position: $\max_k r_k$. This is a sampled maximum; intermediate continuous motion could have a slightly larger error.
- Final position: $r_{N-1}$ only if the full run completed. For failed runs `final_position_m` is null, and `last_position_m` describes the retained prefix.
- Speed: stored $v_k-v_{d,k}$; RMS and signed final error. This excludes the direct speed disturbance in instantaneous translation.
- Heading: $\operatorname{wrap}(\theta_k-\theta_{d,k})$ in $[-\pi,\pi)$; RMS and signed final error. Integrated headings stay unwrapped; display headings use `unwrap` for continuity.
- Saturation: number of intervals with any requested/applied channel difference. Per-channel flags are stored too.
- Domain: count of recorded invalid-domain exits and sampled speeds outside the declared forward branch. Actual stored v is affine within held-command intervals, so endpoint monitoring suffices to detect a threshold crossing at sample resolution.
- Effort: separate held-input integrals $\sum a_k^2\Delta t_k$ and $\sum\omega_k^2\Delta t_k$, computed from applied commands before exogenous offsets.

## Numerical integration evidence

`integration.npz` records a constant-speed turn with v = 0.8 m/s, omega = 0.7 rad/s, initial heading 0.3 rad, and horizon 4 s. Its final position is checked against the analytical circular-motion solution. These measured errors produce Figure 03:

| Timestep [s] | Euler position error [m] | RK4 position error [m] |
| --: | --: | --: |
| 0.50000 | 0.394853 | 1.17794e-5 |
| 0.25000 | 0.197174 | 7.34196e-7 |
| 0.12500 | 0.098555 | 4.58559e-8 |
| 0.06250 | 0.049274 | 2.86550e-9 |
| 0.03125 | 0.024636 | 1.79082e-10 |

Halving h reduces RK4 error by approximately 16× in this range. Straight motion with constant acceleration is also tested analytically. The numerical study isolates the integrator; it does not assert fourth-order convergence of the entire sampled controller.

`primitives.npz` stores four independent command-sequence rollouts, all with initial state `(0,0,0.5,0)`: straight travel for 4 s, turning at 0.35 rad/s for 4 s, a full circle at 0.5 rad/s for $2\pi/0.5$ s, and a 10 s composed sequence. The composed sequence accelerates at 0.1 m/s² over [0,2), turns at 0.6 rad/s over [3,6), turns at −0.4 rad/s over [7,9), and decelerates at −0.1 m/s² over [8,10). Other channels/intervals are zero. Uniform primitive timesteps are about 0.02 s; the circle grid fits its exact horizon.

## Saved evidence schema

Each `scenario__controller.npz` is an ordinary compressed NumPy archive, loadable with `np.load(path, allow_pickle=False)`:

| Array | Shape | Convention |
| :-- | :-- | :-- |
| `times` | `(N,)` | Simulation timestamps [s], including initial and final saved state |
| `states` | `(N,4)` | x [m], y [m], stored v [m/s], unwrapped theta [rad] |
| `reference_states` | `(N,4)` | Same order; reference heading from atan2 |
| `reference_velocity` | `(N,2)` | Desired world x/y velocity [m/s] |
| `reference_acceleration` | `(N,2)` | Desired world x/y acceleration [m/s²] |
| `requested` | `(N-1,2)` | Acceleration [m/s²], yaw rate [rad/s] at interval starts |
| `applied` | `(N-1,2)` | Commands after any shared limits, before disturbances |
| `disturbance` | `(N-1,2)` | Delivered speed offset [m/s], yaw-rate offset [rad/s] |
| `saturated` | `(N-1,2)` | Boolean requested/applied difference per channel |

Each matching JSON records controller, scenario, seed, complete configuration, and run status. Failed runs save their actual prefix rather than padding future states with zeros. `config.json` records the declared protocol. `manifest.json` records UTC generation time, state/control conventions, upstream revision, coefficients, reference minimum speed, package/runtime versions, source fingerprints, and hashes for saved files. `metrics.json` and `outcomes.md` are computed by reloading the saved archives.

`media/provenance.json` binds every PNG/SVG/GIF to the results manifest, exact metric file, input checksums, and renderer sources. Image encoding and SVG identifiers can differ across versions/platforms; numerical arrays and metric values are the reproducibility target, not universal byte-identical media. The lockfile and shared theme narrow environment differences.

## Rendering and verification

The reproduction command in the README was run into a previously absent `build/reproduced/` directory. Its saved numerical arrays and metrics were compared with the earlier independent run. The final logs and gallery were promoted to the curated directories only after review. Rendering alone was run separately against those saved results. [Verification details](verification.md) record the executed checks and visual review.

The hero and replay use the yaw-bias logs, sample nearest recorded states at 120 uniformly spaced indices, and encode at 12 fps with an additional 0.8 s final hold. GIF durations alternate between 80 and 90 ms to accommodate centisecond timing; total playback is 10.8 s including the hold. No animated trail includes future executed positions. The full dotted reference remains visible as context. Both sides of the hero use identical bounds and one shared simulation index. Six PNG review frames retain before/during/after states from both animations.

The gallery uses the exact declared reference, not a manually redrawn path. Labels, equal spatial aspect ratios, fixed comparison bounds, continuous heading display, and disturbance timing were visually reviewed. Figure 01 alone is conceptual, with an arbitrary explanatory pose; it is labelled accordingly.

## Limits on interpretation

Feedback improves these three cases with the chosen model and exact state. This does not establish global stability under arbitrary disturbances, robustness across seeds, safe control near standstill, or performance under sensor/actuator constraints. Optional saturation changes the closed-loop acceleration mapping and may reduce performance. The chosen noise model can produce unrealistic instantaneous motion. There are no maps, obstacle checks, uncertainties, sensors, or learned behavior. The final speed remains nonzero, so a later milestone must explicitly design stopping and goal regulation.
