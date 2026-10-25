# Milestone 04 · Landmark mapping and SLAM

The new estimation problem has unknown landmark coordinates as well as uncertain rover motion. The estimator receives an initial rover prior, applied commands, speed readings, feature IDs, and noisy range/bearing readings. A dynamic joint Gaussian represents the current rover state and all features observed so far. Its posterior rover state drives the existing controller.

## Scope and experiment

[configs/slam.json](../configs/slam.json) defines a 12×10 m world with two supplied freestanding obstacles and a loop through four supplied checkpoints. The obstacle map remains known to the planner and synthetic visibility model. The **point-landmark map** is initially unknown to SLAM. This experiment does not infer occupied/free space, choose exploration goals, or plan using the learned landmarks.

A* plans each leg in configuration space; validated shortcuts and the existing continuously certified quadratic rounding form one timed reference. The route is 66 s long at a 0.05 s control period. A 20 Hz period keeps the dense-covariance experiment compact; previous milestone evidence retains its original periods. The path returns to its starting position while still moving, with a different final heading. This is a prescribed loop, not an autonomous return-home or parking policy.

The initial estimate differs from the reference state by `[0.04 m, −0.03 m, 0.01 m/s, 0.02 rad]`. Standard deviations of the initial rover prior are `[0.08 m, 0.08 m, 0.03 m/s, 0.04 rad]`. No landmark coordinate or landmark covariance is present at filter construction. The finite initial pose prior anchors the map frame; no posterior truth alignment is performed.

Speed readings are taken at 10 Hz with 0.025 m/s standard deviation. Identified range/bearing observations arrive at 5 Hz, within 4 m, with full angular field of view and opaque static rectangles. Noise scales remain 0.05 m and 0.015 rad. IDs are supplied; coordinates are withheld. Point landmarks are sensing features, not collision obstacles. A coincident feature cannot supply a defined bearing and is skipped by the synthetic sensor.

Seeds 7, 19, and 41 each spawn independent plant, speed-sensor, and landmark-sensor streams. Pre-generated arrays are reused for every estimator and scenario for that seed. Plant speed/yaw offsets have per-interval standard deviations 0.03 m/s and 0.04 rad/s. In the outage condition, landmarks disappear on `[28,40) s`; speed readings continue. An unmodeled +0.08 rad/s yaw bias acts on `[30,34) s`. The initial rover model, gains, actuator limits, forward-speed guard, and collision audit continue from previous milestones.

## Joint state and prediction

The filter state is dynamically ordered by discovery:

```text
mean = [x, y, v, theta, landmark_a_x, landmark_a_y, landmark_b_x, landmark_b_y, ...]
```

Opaque feature IDs map to local state slots. The filter does not receive a list of unobserved features or their coordinates. The logger later pads arrays into canonical numeric ID order for evidence and visualization; that bookkeeping is not the estimator's prior.

The nominal rover prediction and its state/process Jacobians are reused from Milestone 03's differentiated RK4 map. For rover covariance `P_rr`, rover-to-map covariance `P_rm`, and static map block `P_mm`:

```text
rover_next = RK4(rover, applied_command, dt)
P_rr_next  = F P_rr Fᵀ + G W Gᵀ
P_rm_next  = F P_rm
P_mm_next  = P_mm
```

Assumed independent held errors are acceleration, forward-speed offset, and yaw-rate offset, with standard deviations `[0.02 m/s²,0.08 m/s,0.10 rad/s]`. These are discrete held errors, not white-noise densities. The deterministic bias is not estimated. The map block stays static under prediction, but subsequent observations can change it through the cross-covariance.

## First observations and augmentation

For a newly identified feature with range `r` and bearing `b`, define `phi=theta+b` and inverse observation:

```text
new_point = [x + r cos(phi), y + r sin(phi)]
A = d(new_point)/d(rover)
  = [[1,0,0,−r sin(phi)], [0,1,0,r cos(phi)]]
B = d(new_point)/d(range,bearing)
  = [[cos(phi),−r sin(phi)], [sin(phi),r cos(phi)]]
```

With measurement covariance `R`, the new feature's uncertainty and correlations are:

```text
P_new,new   = A P_rr Aᵀ + B R Bᵀ
P_new,old   = A P_rover,old
```

The full joint block is appended, including correlations with every existing landmark. The original rover/map covariance remains unchanged by this first sighting. The same reading is **not** subsequently used as an independent correction. Measurements with nonpositive/undefined range or nonfinite values are explicitly rejected and logged. Duplicate feature IDs within a packet are rejected rather than silently counted twice.

This augmentation matters even when two landmarks appear in the same packet: their initial coordinates share uncertain rover pose, so independent feature covariances would discard relevant information. Later map features also inherit uncertainty accumulated during rover motion.

## Reobservations and revisits

The expected range/bearing to a previously mapped point uses the original nonlinear measurement equation. Its full Jacobian has rover columns `H_r`, landmark-position columns `−H_r[:, :2]`, and zero columns elsewhere. Innovations wrap bearing differences around ±π. Linear solves produce the gain, and Joseph form updates the complete covariance. Symmetry is averaged to control floating-point asymmetry; eigenvalues are not clipped.

The joint gain can move the rover and correlated unobserved landmarks. It cannot manufacture an independent absolute pose anchor from relative measurements. Tests include an initially absent feature added after motion, whose coordinates change when a different correlated feature is reobserved.

A **revisit** is operationally defined as an identified feature appearing after at least five seconds without an accepted observation. The IDs supply correspondence. This is a landmark-filter demonstration of revisiting; it does not implement visual place recognition, scan matching, a pose graph, or a loop-closure front end.

At each time, the sensor layer generates a packet, the estimator corrects, the controller receives the estimated rover state, the plant integrates a held clipped command, and the estimator predicts from that applied command. All feedback modes use estimates. Hidden truth and plant disturbances do not enter the SLAM update API.

## Comparisons

| Mode | Prior landmark coordinates | Reobservations | Controller input |
| :-- | :-- | :-- | :-- |
| `known_map` | Supplied, explicitly privileged baseline | Ordinary known-map EKF updates | Estimated rover state |
| `first_sighting` | None | Ignored; speed corrections continue | Estimated rover state |
| `slam` | None | Joint rover/map corrections | Estimated rover state |

The first-sighting mode is not an artificially frozen map: correlated speed corrections may still adjust its feature means. Its distinguishing limit is that it does not use later range/bearing observations of known IDs. There are 18 closed-loop executions, and each has its own resulting path, visibility, and measurement values. Shared noise does not imply identical observation packets across trajectories.

For each of the six SLAM runs, a separate **estimator-only matched replay** uses the same applied commands and observations but denies a feature's return after the five-second gap. Once denied, that feature remains denied. Continuous observations in its first visibility episode and observations of newly discovered IDs remain usable. The replay does not drive another simulated rover, and its errors are measured against the original SLAM run's actual states. It therefore isolates this estimator intervention without asserting a counterfactual physical trajectory or clearance outcome.

Before the first denied revisit, the ordinary and denied-revisit posterior histories match exactly. This equality and their later divergence are tested. Replay tests also remove the true map and obstacle world and poison actual-state/disturbance arrays; SLAM's reconstructed posteriors remain identical.

## Metrics, gauge, and failure interpretation

Pose RMS is the RMS position error against actual rover position. Tracking RMS compares actual motion with the reference. Final map RMS compares learned point coordinates with evaluation truth in the declared initial-prior frame, without a fitted transform. Pair-distance RMS compares inter-landmark distances and is insensitive to a global rigid pose change. The first-sighting map RMS records each feature at its own discovery time; it is a diagnostic collection, not one simultaneous historical map.

Nominal SLAM discovers six landmarks for every seed. Mean pose RMS is 0.1489 m; final map RMS is 0.1289 m; pair-distance RMS is 0.0299 m. The matched denied-revisit replay has 0.2199 m pose RMS and 0.1268 m pair-distance RMS. Raw coordinate error need not improve for every seed: nominal seed 7 has larger final raw map RMS after revisits even though its relative map distances improve substantially. The initial prior frame and the shared pose/map gauge explain why these metrics answer different questions.

The outage condition exposes limitations. SLAM seed 19 has 23 enclosing-disk overlap samples. Seed 41 exits the forward-speed domain early, after a map/pose correction and ensuing saturated feedback. Its full partial history and final posterior remain saved. Seed 7 completes with positive audited clearance. The supplied-map baseline also collides in the seed-19 outage case. A better final map cannot erase unsafe motion earlier in the run.

Across-run summary means include every recorded run. A failed run contributes its observed prefix RMS and last available map, not a completed-horizon quantity; status and observed horizon remain separately recorded. A clear run must complete and have a positive interval clearance bound. These bounds use actual states after execution and inherit the earlier numerical-endpoint/RK4 error limitations. No runtime collision intervention or safe stop is implemented.

NEES, feature NIS, covariance eigenvalues, map correlations, and nominal Gaussian coverage are diagnostics. Repeated linearization, a fixed biased prior, a fixed unmodeled disturbance, and three seeds do not establish estimator consistency or a probabilistic safety guarantee.

## Evidence and reproduction

`explorer slam-reproduce` runs the entire experiment into a new result directory. `slam-simulate` omits rendering. `slam-render` verifies/replays saved inputs and creates the gallery without a new physical simulation, sensor run, or plan. `slam-verify` checks hashes, tables, recalculated outcomes and clearance, all ordinary estimator replays, and all denied-revisit replays.

Numerical logs record N actual/posterior/controller states and N−1 applied/requested commands and plant disturbances. Missing observations are NaN plus masks. Event codes distinguish initialization, update, revisit, rejection, and omission. Every sample saves rover covariance and learned point means. Full joint means/covariances are saved at sensor opportunities and at the last sample, indexed by `joint_indices`. Unseen slots are NaN, never false zero-coordinate landmarks. This keeps dense-covariance evidence compact while retaining every posterior that can introduce or reobserve a landmark. Map ellipses in playback use the latest recorded joint posterior.

Manifests hash configuration, reference certificates, noise inputs, numerical histories, statuses, metrics, and tables. Render provenance hashes the numerical manifest, render sources, and outputs. Rendering tests forbid planner/physical-experiment entrypoints. The six figures and synchronized GIF retain actual paths, estimated motion, discovered maps, uncertainty, and failure outcomes.

## Next milestone

Decision-making and exploration should choose goals from available evidence and manage localization health before relying on the evolving map. Occupancy inference and obstacle discovery still require a new sensor-to-map representation. Return-home decisions and stopping/parking remain separate from merely reaching the start position on this prescribed moving loop.
