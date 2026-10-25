# Milestone 03 · Sensing and localization

This milestone replaces exact-state feedback with an explicit sensor → estimator → controller interface. The known obstacle map and collision-certified timed reference from Milestone 02 are retained. It studies local localization around a supplied initial pose, including failure during lost observations.

## Experiment contract

The configuration is [configs/localization.json](../configs/localization.json). Its navigation file is resolved relative to that file and copied into the saved configuration. The obstacle world, 0.329 m enclosing rover disk, 54.66 s route, 0.02 s control period, gains, and actuator limits match Milestone 02. The numerical state order remains `(x [m], y [m], v [m/s], theta [rad])`; inputs are acceleration and yaw rate.

Every estimator starts from the reference initial state plus a fixed `[0.12 m, −0.08 m, 0.02 m/s, 0.08 rad]` offset. The initial covariance is diagonal, with standard deviations `[0.20 m, 0.20 m, 0.05 m/s, 0.12 rad]`. This is a known initial-pose prior with error; it is not global localization. The actual rover starts on the reference.

Three independent PCG64 streams are spawned from each seed (7, 19, 41): plant disturbance, speed noise, and landmark noise. Arrays are drawn before simulation and saved once per seed. Invisible landmarks still have preallocated indexed noise, so visibility changes do not shift the random sequence. Each mode and scenario receives the same underlying arrays for that seed. Observation values and visibility depend on each actual trajectory and need not match between modes.

All cases have zero-order-held forward-speed offsets and yaw-rate offsets with standard deviations 0.03 m/s and 0.04 rad/s per integration interval. The two disturbed cases also have a +0.12 rad/s bias on `[19.14, 23.14) s`. The outage case suppresses landmarks on `[18, 30) s`; speed readings continue.

## Sensors

The synthetic speed sensor measures the stored `v`, with independent additive 0.025 m/s Gaussian noise every five integration steps (10 Hz). The body-forward disturbance affects translation but is absent from stored `v`, as in the original rover model. The speed reading therefore does not measure this slip-like offset.

Five known point landmarks have perfect identities. Every ten integration steps (5 Hz), the synthetic sensor reports range and body-relative bearing for each landmark within 6 m and with an unobstructed line segment to the actual rover center. Its field of view is a full 360°. The given static rectangles are opaque; endpoints and obstacle boundaries are excluded by the existing continuous geometry routines. Landmark positions and the map are supplied, never estimated here.

For landmark coordinates `(l_x,l_y)`, let `d_x=l_x−x`, `d_y=l_y−y`, `q=d_x²+d_y²`, and `r=sqrt(q)`. The observation model is:

```text
h(x) = [r, wrap(atan2(d_y,d_x) − theta)]
H    = [[−d_x/r, −d_y/r, 0,  0],
        [ d_y/q, −d_x/q, 0, −1]]
```

Independent measurement standard deviations are 0.05 m and 0.015 rad. Noisy bearings are wrapped; range noise is not truncated. Coincident landmarks are skipped by the synthetic sensor and explicitly rejected by the mathematical observation model. This idealized feature sensor is not a camera or lidar perception implementation. There are no false detections, unknown identities, timestamp jitter, or delayed packets.

Missing values are saved as NaN plus an explicit `landmark_visible` mask. NaN is an absence marker, never an observation of zero. Sensor opportunities are distinguished from range/occlusion losses in the gallery.

## Filter and timing

The EKF uses the same RK4 nominal state transition as the simulator. Its analytic state-transition Jacobian `F` and held-error Jacobian `G` propagate sensitivities through all four RK stages. This avoids combining an RK4 mean with an unrelated Euler covariance model. Central-difference tests independently check both Jacobians, including zero and nonzero yaw rates.

The process error order is `(acceleration error, forward-speed offset, yaw-rate offset)`. Assumed standard deviations `[0.02 m/s², 0.08 m/s, 0.10 rad/s]` describe independent errors held for one control interval. They are not continuous-time white-noise densities. With `W=diag(std²)`, prediction is:

```text
mean_next = RK4(mean, applied_command, dt)
P_next    = F P Fᵀ + G W Gᵀ
```

The acceleration uncertainty is an estimator allowance; the simulated acceleration has no extra error beyond the held applied command. The model is deliberately approximate. The deterministic yaw bias is neither estimated as a state nor included in the filter's mean prediction.

At each timestamp:

1. Generate the available sensor packet from actual state in the synthetic sensor layer.
2. Correct the prior using the speed reading and then each observed landmark in fixed ID order.
3. Supply the posterior to the tracking controller (or actual state for the explicit oracle).
4. Record requested commands, clip using shared actuator limits, and integrate the plant over the interval.
5. Predict the next prior using the applied command, without the hidden plant disturbance.

The final timestamp includes a measurement update but no fabricated final command. State/estimate/covariance histories have N entries; applied commands, disturbances, and interval audits have N−1.

Each correction solves the innovation system rather than inverting it explicitly. Bearing innovations use the principal angle, so crossing ±π cannot produce a spurious 2π residual. Covariance uses Joseph form, followed by symmetry averaging; eigenvalues are never clipped to conceal failures. The filter has no truth, reference trajectory, or obstacle map in its update API. The experiment supplies only the initial prior, applied commands, sensor configuration, and observation packets.

## Three feedback modes

| Mode | Controller state | Estimator behavior |
| :-- | :-- | :-- |
| `oracle` | Actual simulator state | A shadow EKF processes the sensors for evaluation but does not drive the rover |
| `dead_reckoning` | Nominal command-integrated estimate | Propagates state/covariance; ignores both sensors |
| `ekf` | EKF posterior | Fuses the available speed and landmark observations |

All three call the original `Feedback.command` law with identical gains and ±0.5 m/s², ±1.2 rad/s actuator bounds. This dead-reckoning baseline uses commanded motion, not a hidden wheel-odometry measurement. The saved `controller_states` array makes the feedback boundary auditable. Outside the forward-speed domain, the experiment records an explicit failure and retains the partial history. It does not silently regularize the inversion.

The oracle's shadow EKF makes estimation possible on a well-tracked trajectory. The EKF-driven run measures estimation and control together; it cannot be interpreted as a same-trajectory offline estimator comparison. Both views remain in the evidence.

## Measurements and failure interpretation

Localization RMS compares estimated position with actual position. Tracking RMS compares actual position with the timed reference. Wrapped heading error, control saturation, actual disk clearance, interval clearance bounds, and first overlap are reported separately.

Position coverage uses the 2D quadratic form `e_xyᵀ P_xy⁻¹ e_xy ≤ 5.99146` (a nominal 95% Gaussian ellipse). NEES uses the four-state error and covariance; landmark NIS uses each sequential 2D innovation. These are descriptive diagnostics. Fixed prior offsets, correlated time samples, conservative assumed noise, and the unmodeled fixed bias preclude treating three seeds or averaged time samples as an independent consistency certification. The marginal ±1.96σ bands are distinct from a joint 2D ellipse.

Normal-sensing EKF feedback remains clear across all six nominal/yaw-bias runs. In all three outage runs it crosses the conservative envelope, and its covariance understates the actual position error during the biased interval. Re-enabling the landmark sensor at 30 s does not guarantee immediate observations: occlusion and range still apply. In the seed-7 run, landmark observations resume only after the rover has drifted far from its estimated path. Subsequent recovery does not erase earlier overlaps.

Clearance is calculated from actual states after execution with the Milestone 02 audit. The interval travel bound is conditioned on numerical endpoints and omits a formal global RK4 error enclosure. No contact response, stop, replanning, or online safety monitor changes the motion. Negative clearance is retained. All 27 integrations complete, including unsafe executions; completion is not a safety label.

## Reproduction and audit

`explorer localize-reproduce` creates an entirely new result directory and gallery. `localize-simulate` omits rendering. `localize-render` consumes saved data; its verification phase replays estimator updates but never invokes a planner, plant integrator, or synthetic sensor to generate new empirical evidence. `localize-verify` verifies hashes, regenerates both Markdown tables, recomputes metrics/clearance, and replays each estimator from observations and applied commands without reading actual states for filtering.

Saved `plan.npz` holds the reference, RK4 time grid, grid path, and Bézier data. Each run's NPZ records actual states, estimates, covariances, exact controller inputs, controls, disturbances, sensor values/masks, NIS values, and clearance audits. JSON metadata records seed, mode, scenario, source noise file, and status. The manifest hashes all numerical evidence and records source hashes and the runtime. Gallery provenance hashes inputs and rendered outputs.

The six figures and synchronized animation use the established rover/theme. The outage hero is intentionally a failed experiment. Its three retained review frames show before, during, and after the outage; the entire unsafe history remains available.

## Boundary and next milestone

This is known-map, known-landmark local localization. It adds neither SLAM nor unknown-object perception. Next, mapping/SLAM should put initially unknown landmark positions in the estimation problem and retain the same truth/observation/controller boundary. Runtime localization-health handling and replanning are also motivated by the outage failures. Parking still requires a separate transition through the tracking controller's singular-speed domain.
