# Complete visual gallery

All **40 figures and seven animations** from the five completed milestones are collected here. Earlier visualizations stay in this cumulative gallery as the project advances. SVG versions and the retained animation review frames are linked alongside the figures.

[Current project status and reproduction commands](README.md)

| Milestone | Figures | Animations |
| :-- | --: | --: |
| [01 · Trajectory tracking](#milestone-01-trajectory-tracking) | 11 | 2 |
| [02 · Obstacle-aware navigation](#milestone-02-obstacle-aware-navigation) | 11 | 2 |
| [03 · Sensing and localization](#milestone-03-sensing-and-localization) | 6 | 1 |
| [04 · Landmark mapping and SLAM](#milestone-04-landmark-mapping-and-slam) | 6 | 1 |
| [05 · Decision-making and exploration](#milestone-05-decision-making-and-exploration) | 6 | 1 |

## Milestone 01: Trajectory tracking

[Full milestone walkthrough](MILESTONE_01.md)

![Synchronized open-loop and feedback rover execution with a shared yaw disturbance](media/hero.gif)

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

### Animation review frames

| Replay | Before | During | After |
| :-- | :-- | :-- | :-- |
| [Hero](media/hero.gif) | [Before](media/hero_before.png) | [During](media/hero_during.png) | [After](media/hero_after.png) |
| [Diagnostic replay](media/diagnostic_replay.gif) | [Before](media/diagnostic_replay_before.png) | [During](media/diagnostic_replay_during.png) | [After](media/diagnostic_replay_after.png) |


## Milestone 02: Obstacle-aware navigation

[Full milestone walkthrough](MILESTONE_02.md)

![The same rover follows an obstacle-aware reference with open-loop and feedback control](media/navigation/navigation_hero.gif)

The same oriented rover, navy background, cyan feedback, amber open loop, pale reference, and equal spatial aspect ratios continue from Milestone 01. All empirical images and replay frames use saved data. Maps show supplied geometry, not perceived or learned surroundings.

### Planning: where can this rover fit?

![Full rover envelope and conservative inflated configuration space](media/navigation/01_configuration_space.png)

**01 · Configuration space.** A 0.329 m disk encloses the original body and wheels at every heading. Conservative box inflation and inward boundary offsets give the point planner room for the rover and reserves. [SVG](media/navigation/01_configuration_space.svg).

![A* and Dijkstra search expansions and equal-cost solutions](media/navigation/02_astar_vs_dijkstra.png)

**02 · A* versus Dijkstra.** Both use the same nodes, collision-checked edges, and Euclidean costs. The heuristic changes the search effort while preserving the optimal graph cost here. [SVG](media/navigation/02_astar_vs_dijkstra.svg).

![Recorded search expansion replay using a shared node-expansion budget](media/navigation/search_replay.gif)

**Search replay.** One shared expansion-count clock makes the algorithms' progress comparable. This is algorithmic work, not wall-clock speed. The path appears when each recorded search reaches its goal.

### Bridge the path to the motion model

![Grid path, checked shortcuts, and rounded timed reference](media/navigation/03_path_to_trajectory.png)

**03 · Path to trajectory.** Grid edges and shortcut segments are checked continuously. Quadratic Bézier corners match line velocities at joins. Their full control-point boxes must stay free; unchecked smoothing is never assumed safe. [SVG](media/navigation/03_path_to_trajectory.svg).

![Positive reference speed and bounded acceleration and angular rate](media/navigation/04_reference_feasibility.png)

**04 · Reference feasibility.** Analytical segment extrema constrain timing between samples as well as at them. Position and velocity are continuous; acceleration may jump at segment boundaries. This is a geometric planner plus a validated kinematic bridge, not a kinodynamic search or an energy optimum.

### Execute, measure, retain failures

![Open-loop and feedback executions with an explicitly marked envelope violation](media/navigation/05_navigation_execution.png)

**05 · Execution.** A safe reference does not protect a controller that drifts away from it. The first envelope overlap is marked, and the full unsafe open-loop history remains visible. Collision auditing is post hoc; neither controller replans or reacts to obstacles. [SVG](media/navigation/05_navigation_execution.svg).

![Four synchronized recorded snapshots around the disturbance and near the goal](media/navigation/06_navigation_snapshots.png)

**06 · Time progression.** Four snapshots use identical bounds and the same recorded comparison. Actual trails stop at the snapshot time; ghosts show desired poses.

![Tracking error with a separately labelled feedback zoom and wrapped heading error](media/navigation/07_navigation_error.png)

**07 · Tracking error.** A common scale compares both controllers; a labelled centimetre-scale feedback panel exposes the smaller recovery. Shading marks the shared disturbance interval.

![Clearance and an interval travel bound with unsafe negative results retained](media/navigation/08_clearance_audit.png)

**08 · Clearance.** Tracking error and clearance answer different questions. The audit checks the enclosing disk against obstacles and boundaries and subtracts a bound on between-sample travel. Negative clearance is retained as a violation. [SVG](media/navigation/08_clearance_audit.svg).

![Applied controls, requested samples, and shared actuator limits](media/navigation/09_navigation_controls.png)

**09 · Control activity.** Held applied commands, requested samples, and actuator limits are shown separately. Neither controller saturated in these declared runs; both were subject to the same limits.

### Understand the limits of the planner

![A sealed world and narrow passage that produce retained planning failures](media/navigation/10_retained_planning_failures.png)

**10 · Retained failures.** The sealed world has no route on either graph. In the 0.30 m passage a point path exists, but the conservative rover planner rejects it. No motion is executed for either rejected rover plan. This is not a universal impossibility proof for every physical configuration. [SVG](media/navigation/10_retained_planning_failures.svg).

![Grid spacing versus optimal graph cost and expanded nodes](media/navigation/11_grid_resolution.png)

**11 · Grid resolution.** Finer grids give different graphs and more search work. Costs at 0.5, 0.25, and 0.125 m spacing are 22.657, 22.364, and 22.218 m, with 168, 660, and 2,513 expansions.

### Animation review frames

| Replay | Before | During | After |
| :-- | :-- | :-- | :-- |
| [Navigation hero](media/navigation/navigation_hero.gif) | [Before](media/navigation/navigation_hero_before.png) | [During](media/navigation/navigation_hero_during.png) | [After](media/navigation/navigation_hero_after.png) |
| [Search replay](media/navigation/search_replay.gif) | [Before](media/navigation/search_replay_before.png) | [During](media/navigation/search_replay_during.png) | [After](media/navigation/search_replay_after.png) |


## Milestone 03: Sensing and localization

[Full milestone walkthrough](MILESTONE_03.md)

![Exact-state, dead-reckoning, and EKF feedback during a landmark sensor outage](media/localization/localization_hero.gif)

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

### Animation review frames

| Replay | Before | During | After |
| :-- | :-- | :-- | :-- |
| [Localization hero](media/localization/localization_hero.gif) | [Before](media/localization/localization_hero_before.png) | [During](media/localization/localization_hero_during.png) | [After](media/localization/localization_hero_after.png) |


## Milestone 04: Landmark mapping and SLAM

[Full milestone walkthrough](MILESTONE_04.md)

![The rover builds a landmark map using first sightings and joint EKF SLAM](media/slam/slam_hero.gif)

All empirical figures and replay frames use saved evidence. Crosses show evaluation truth; they are not observations supplied to SLAM. Shaded obstacle geometry is supplied for routing.

![Final landmark maps from first sightings and joint SLAM](media/slam/01_mapping.png)

**01 · Joint mapping.** Both filters begin without landmark coordinates. Reobservations constrain the joint map and motion belief. Ellipses show approximate 95% map marginals. [SVG](media/slam/01_mapping.svg).

![Four stages of landmark discovery and map correction](media/slam/02_map_growth.png)

**02 · Map growth.** Landmark means appear after their first sensor observation. Equal-scale snapshots retain discovery order and the actual trail.

![Pose estimation and actual tracking errors with normal sensing and an outage](media/slam/03_errors.png)

**03 · Two errors.** Localization compares estimate with truth; tracking compares actual motion with the reference. Different closed-loop estimators produce different trajectories and visibility.

![Matched estimator replays with accepted and denied revisits](media/slam/04_revisits.png)

**04 · Revisit ablation.** These replays use identical recorded motion and observations. They isolate estimator behavior without claiming a counterfactual rover trajectory. [SVG](media/slam/04_revisits.svg).

![Correlations between rover and landmarks, with rover position covariance over time](media/slam/05_joint_uncertainty.png)

**05 · Correlated belief.** The dense joint covariance retains relations between rover and map variables. A finite initial rover prior anchors the coordinate frame; covariance is a model belief, not a clearance certificate.

![Actual outage trajectories and clearance audits retain collisions](media/slam/06_outage_clearance.png)

**06 · Retained failures.** Landmark corrections cannot undo earlier envelope overlaps. The full study also includes a partial run that exits the controller's speed domain. [SVG](media/slam/06_outage_clearance.svg).

### Animation review frames

| Replay | Before | During | After |
| :-- | :-- | :-- | :-- |
| [Slam hero](media/slam/slam_hero.gif) | [Before](media/slam/slam_hero_before.png) | [During](media/slam/slam_hero_during.png) | [After](media/slam/slam_hero_after.png) |


## Milestone 05: Decision-making and exploration

[Full milestone protocol](docs/exploration.md)

![An unsupervised mission continues through a sensor outage while a supervised mission waits and recovers](media/exploration/exploration_hero.gif)

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

### Animation review frames

| Replay | Before | During | After |
| :-- | :-- | :-- | :-- |
| [Exploration hero](media/exploration/exploration_hero.gif) | [Before](media/exploration/exploration_hero_before.png) | [During](media/exploration/exploration_hero_during.png) | [After](media/exploration/exploration_hero_after.png) |
