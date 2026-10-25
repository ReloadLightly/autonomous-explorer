# Milestone 02: obstacle-aware navigation

The physical problem is now: **how can the same rover reach a destination without its body or wheels intersecting a known obstacle?** Milestone 01 supplied the motion model and trajectory tracker. This milestone supplies a route and turns it into a timed reference. The [source audit](navigation-sources.md) connects the work to Chapter 4, Exercise 1. The [verification record](navigation-verification.md) distinguishes the tests, numerical evidence, and visual review.

## 1. Start with a finite robot in a known world

The provided workspace is 12 × 9 m. The start is (1,1) m and the destination is (11,8) m. Two rectangular obstacles form a winding passage: `[3,0,4.1,5.7]` and `[7.3,3.3,8.4,9]`, using `(xmin,ymin,xmax,ymax)` in metres. Bounds use `(xmin,xmax,ymin,ymax)`. The obstacle positions are supplied, not estimated from sensors. Figure [01](../media/navigation/01_configuration_space.png) shows the physical workspace and conservative configuration-space approximation.

The same renderer still draws a 0.48 × 0.30 m body, two drive wheels on a 0.38 m track, and a rear caster. The wheels make the outer width 0.45 m. We enclose the entire body/wheel bounding rectangle in an orientation-independent disk:

$$R=\sqrt{(0.48/2)^2+(0.45/2)^2}=0.32897568\text{ m}.$$

`rover_radius` computes this value from configuration. A compatibility test checks the dimensions against the preserved renderer and checks the enclosure after rotation. The disk is conservative: configurations rejected by it may still fit a more detailed oriented model.

For point planning, each obstacle box is expanded by $R+m$ in both coordinate directions, and workspace walls move inward by the same amount. This axis-aligned expansion contains the exact disk Minkowski sum, including its rounded corners, so it may overestimate collision space. Contacts count as collisions. There are two explicit reserves:

- **Search margin: 0.35 m.** Leaves extra room around the robot for the geometric route.
- **Reference margin: 0.10 m.** Rounded curves must be certified outside this smaller forbidden region. The difference provides room to round corners.

Neither reserve is a proven invariant tracking-error tube. They do not make an obstacle-unaware controller safe under arbitrary disturbance.

[`World`](../src/autonomous_explorer/planning.py) implements this geometry. For a segment $p(\lambda)=p_0+\lambda(p_1-p_0)$, intersect each box-coordinate slab with $\lambda\in[0,1]$. If a nonempty shared parameter interval remains, the segment touches or crosses the obstacle and is rejected. Parallel coordinates are handled separately. This catches thin obstacles between free endpoints; checking endpoints alone would miss them.

## 2. Build and search a graph

The 0.25 m grid uses integer node indices and eight candidate directions. Grid spacing is in metres per lattice step, not cells per metre. The Euclidean edge cost is h for axial motion and $h\sqrt2$ for diagonal motion. Every candidate edge must pass the full continuous segment check. Diagonals also require both adjacent axial nodes to be free, preventing grid-corner cutting. This extra rule can reject geometrically feasible edges; it is part of the declared graph for both algorithms.

Endpoints must lie on the lattice. Off-grid requests raise an error rather than changing the problem through implicit snapping. Free-space boundaries and obstacle contacts are rejected consistently. The graph is finite because the world is bounded.

[`grid_search`](../src/autonomous_explorer/planning.py) uses a heap ordered by $(f,g,\text{integer node})$, giving deterministic ties. The best cost to a node is g. A shorter candidate replaces its label and parent; stale heap entries are skipped. A* chooses

$$f(q)=g(q)+\|q-q_G\|_2,$$

while Dijkstra uses $f(q)=g(q)$. Euclidean distance is an admissible, consistent heuristic for these Euclidean edges by the triangle inequality. A node's first nonstale expansion can therefore be closed. On reaching the goal, parent links recover the route. Exhausting the frontier returns `unreachable_on_grid` rather than a fabricated path.

Both algorithms return **22.363961 m** on the primary graph. A* expands **660** nodes; Dijkstra expands **868**. Figure [02](../media/navigation/02_astar_vs_dijkstra.png) and the [search replay](../media/navigation/search_replay.gif) show the saved expansion order. The replay uses one expansion-count clock; it is not a runtime benchmark. Optimality applies to this graph and cost, not every continuous robot motion.

The resolution study changes only grid spacing: 0.5, 0.25, and 0.125 m. The respective optimal graph costs are 22.656854, 22.363961, and 22.217514 m, with 168, 660, and 2,513 A* expansions. Figure [11](../media/navigation/11_grid_resolution.png) shows the work/approximation tradeoff. It does not establish asymptotic convergence from three examples.

## 3. A geometric path is not a rover trajectory

Grid paths have corners and no time law. Executing instantaneous changes in velocity direction at positive speed would require unbounded angular rate. [`shortcut_path`](../src/autonomous_explorer/planning.py) first replaces chains of grid edges with the farthest visible later waypoint, checking each entire new segment against the **search** forbidden space. This reduces 83 nodes to six waypoints and 21.183957 m of line segments. It is a deterministic greedy shortcut, not a globally optimal any-angle planner.

Unchecked smoothing can cut inside an obstacle. We use a transparent construction instead of assuming a spline is safe. For an interior waypoint Q, define incoming/outgoing unit directions $e_-,e_+$ and a tangent distance r, initially 0.45 times the shorter adjacent segment length. Set

$$A=Q-r e_-,\quad B=Q+r e_+,$$
$$p(u)=(1-u)^2A+2u(1-u)Q+u^2B,\quad 0\le u\le1.$$

The quadratic Bézier curve lies inside the convex hull of its three control points, hence inside their axis-aligned bounding box. `World.hull_box_free` requires that entire box to be free in the **reference** forbidden space. If it is not, r is halved and checked again, up to 24 attempts. Failure is explicit. This sufficient certificate may reject an actually safe curve; it does not miss an obstacle crossing inside a certified box. Straight joins use full segment certificates. Zero-length duplicates and zero-speed reversing corners are rejected.

All four corners in this map use r = 0.309375 m after two halvings. The route consists of five lines and four rounded corners. Figure [03](../media/navigation/03_path_to_trajectory.png) distinguishes the graph path, shortcuts, and rounded reference. A regression test constructs a valid L-shaped route whose unchecked large rounding would collide, and confirms that the certificate shrinks it to a safe curve.

## 4. Supply timing and enforce kinematic bounds

For provisional cruise speed s = 0.4 m/s, a straight segment of length L gets duration L/s. A corner gets duration $T_c=2r/s$. Thus

$$\dot p(u)=s[(1-u)e_-+u e_+],$$

matching the adjacent line velocities at both ends. Position and Cartesian velocity are continuous: the reference is C1. Acceleration is constant within each quadratic corner and zero on lines, so it can jump at joins. The kinematic model permits command jumps; no jerk, steering-actuator, or wheel-dynamics limit is claimed.

[`RoundedReference.sample`](../src/autonomous_explorer/path_reference.py) evaluates the quadratic, its derivative divided by segment duration, and its second derivative divided by duration squared. It returns the **same `ReferenceSample` interface** used by the Milestone 01 controller. A line is represented as a quadratic whose middle control point is its midpoint, giving zero second derivative. Speed and heading are recovered from Cartesian velocity; the original flatness inversion recovers acceleration and yaw rate.

Bounds are checked **between samples**, not only on the logging grid. On a segment, velocity is affine: $z(u)=z_0+u\Delta z$. Its minimum norm occurs at

$$u_* = \operatorname{clip}\left(-\frac{z_0^T\Delta z}{\|\Delta z\|^2},0,1\right),$$

with a separate constant-velocity branch. Maximum speed is at an endpoint. The velocity/acceleration cross product is constant, so $|\omega|=|\dot x\ddot y-\dot y\ddot x|/\|z\|^2$ peaks at minimum speed. Tangential acceleration $(z^T\ddot p)/\|z\|$ is monotone within a constant-acceleration segment; its maximum absolute value is at an endpoint. Both sides of a join are included by checking every segment.

Uniformly stretching all segment durations by $\gamma\ge1$ leaves geometry unchanged, divides speed and yaw rate by $\gamma$, and divides acceleration by $\gamma^2$. Choose the largest factor required by the acceleration/yaw limits, then stretch slightly further to fit an integer number of 0.02 s control intervals. All derivatives and extrema are recomputed. The result is:

| Quantity | Continuous bound |
| :-- | --: |
| Duration | 54.66 s |
| Minimum speed | 0.3028940 m/s |
| Maximum speed | 0.3875587 m/s |
| Maximum absolute acceleration | 0.1889517 m/s² |
| Maximum absolute yaw rate | 0.9999528 rad/s |
| Time-stretch factor | 1.0321018 |

Figure [04](../media/navigation/04_reference_feasibility.png) plots the saved derivatives and commands. `reference_certificate.json` records the analytic bounds, hull checks, tangent distances, and time scale. No minimum-time or minimum-energy optimization is solved. This is a geometric planner followed by a validated kinematic reference construction; it is not the kinodynamic planning algorithm discussed later in Chapter 4.

## 5. Execute the original dynamics and controller

The rover state and input remain `(x,y,v,theta)` and `(a,omega)`, with metres, seconds, and radians. The original RK4, zero-order-hold disturbance convention, open-loop interface, feedback law, and forward-speed guard are reused. Start speed/heading come from the planned reference, so both controllers start with zero tracking error. Feedback receives exact simulator state; there is still no estimator.

The protocol was declared in configuration before interpreting its results:

| Case | Exogenous inputs | Seed |
| :-- | :-- | :-- |
| `nominal` | Zero speed/yaw offsets | none |
| `yaw_bias` | +0.12 rad/s yaw offset for 4 s, beginning at 35% of the reference duration rounded to the nearest control timestamp: [19.14,23.14) s | none |
| `seeded` | Independent Gaussian speed/yaw offsets with standard deviations 0.06 m/s and 0.06 rad/s, held per 0.02 s interval | 7 |

Each controller pair shares initial state, timed reference, grid, horizon, and **one identical pre-generated exogenous array**. The seeded draw uses NumPy `default_rng(7)`/PCG64. Offsets enter translation speed and heading rate as in Milestone 01; they are not acceleration noise. No redraw occurs inside RK4. The sampled Gaussian model is not a timestep-invariant SDE or a terrain model.

Both controllers use acceleration limits ±0.5 m/s² and yaw-rate limits ±1.2 rad/s. Requested and applied commands and per-channel saturation flags are logged separately. No declared run saturated. The 0.0001 m/s singular-speed guard is unchanged; every planned and actual speed remains above it in this experiment. The endpoint speed is still 0.3876 m/s—goal regulation and stopping are separate work.

## 6. Audit clearance separately from tracking

[`audit_clearance`](../src/autonomous_explorer/navigation_evidence.py) computes the minimum signed Euclidean disk clearance to every obstacle and workspace wall. Positive means clear, zero means contact, and negative means overlap for the conservative disk model. Negative disk clearance does not by itself prove that the exact drawn body intersects: the disk is larger. The additional count `center_in_obstacle_or_outside_world_samples` identifies stronger violations when the axle centre itself is inside an obstacle or outside the map.

Sampling alone can miss collisions. For an interval of duration h, the translational speed magnitude is bounded by

$$V_k=\max\left(|v_k+d_{v,k}|,\ |v_k+a_kh+d_{v,k}|\right).$$

This follows because stored v is affine and the disturbance is constant during the interval; turning rotates velocity without changing its norm. Any time is at most h/2 from a nearer interval endpoint. Since signed clearance is 1-Lipschitz, use

$$c_{\mathrm{lower},k}=\min(c_k,c_{k+1})-V_kh/2.$$

A positive value certifies clearance under the continuous motion bound **conditioned on the numerical endpoints**. We do not add a separately proven global RK4 error bound, so this is a numerical audit rather than a formal physical safety proof. A negative lower bound alone means “uncertified,” not proven collision. A test shows a fast segment crossing a thin obstacle with both endpoints clear but its interval audit negative.

The controller never reads this clearance or the map. Audits are post hoc; they do not stop, replan, or simulate contact physics. An unsafe run continues under the kinematic equations and remains visible. This deliberately exposes the difference between integration status (`completed`) and execution status (`envelope_violation`, `clearance_uncertified`, or `clear_within_numerical_audit`).

## 7. Read the evidence honestly

The [README outcome table](../README.md#measured-outcomes) is generated from the saved runs. Feedback RMS position errors are 0.000521, 0.005490, and 0.006999 m for nominal, yaw-bias, and seed-7 cases. Corresponding open-loop errors are 0.027423, 1.738191, and 0.761438 m. These are sample RMS statistics over all 2,734 state samples, including endpoints; final, maximum, speed, wrapped-heading, saturation, domain and separate effort metrics are also saved.

Open loop in the yaw-bias case first overlaps the disk envelope at 28.76 s, has 553 overlap samples, and has 404 centre-in-obstacle/outside-world samples. Its minimum clearance is −2.009107 m, including its excursion beyond the map boundary. This unsuccessful execution is retained. Feedback stays clear in this case with a smallest interval bound of 0.466095 m. Across all three feedback runs the smallest bound is 0.463677 m in the seeded case. The seed-7 open-loop run remains clear in this particular realization despite its tracking error; neither controller is labelled universally safe or unsafe from one animation.

The nominal and yaw-bias feedback endpoint errors are about 3.1×10⁻⁹ m after a long final straight segment. Rounded display values of 0.0000 m should not be read as exact tracking throughout the route. Acceleration and angular-rate effort remain separate, with different units. None is called physical energy.

The sealed-world and narrow-passage searches are separate retained failures. In the sealed map neither point nor rover graph reaches the goal. In the 0.30 m passage a point route exists but the conservative rover graph has no path. Neither rejected rover route is executed. Failure is relative to the grid/envelope, not proof of completeness over every physical robot motion.

## 8. Saved files and reproduction boundaries

`results/navigation/` contains:

- `config.json`: resolved experiment, including derived start/end states, horizon and disturbance interval.
- `astar.npz`, `dijkstra.npz`: path, expansion order, occupied-node mask, x/y lattice coordinates.
- `resolution_*.npz`, failure-case archives: the same search schema and explicit summary statuses.
- `plan.npz`: shortcuts, quadratic control points, durations/joins, timestamps, reference states/derivatives, and reference clearance.
- `reference_certificate.json`, `planning_summary.json`: continuous bounds, rounding reductions, costs, counts, failures and configuration references.
- `case__controller.npz`: unchanged Milestone 01 schema: N state/reference samples and N−1 requested/applied commands, disturbances and saturation flags.
- `case__controller__clearance.npz`: N sampled clearances and N−1 interval lower bounds. Matching JSON retains controller/scenario/status.
- `metrics.json`, `outcomes.md`, `manifest.json`: saved-data metrics, generated table, UTC/runtime/upstream/source metadata and file hashes.

Rendering checks numerical evidence before reading it. `media/navigation/provenance.json` binds figures and GIFs to the numerical manifest, input hashes, and renderer sources. Plotting never calls a planner, trajectory builder, or simulator. Tests replace those functions with failures while exercising static rendering. GIFs use nearest logged states or recorded expansion indices; they preserve actual history and fixed comparison bounds.

The original milestone's numerical evidence and media remain byte-for-byte intact. Its complete README is retained as [MILESTONE_01.md](../MILESTONE_01.md); the same robot and core control code are used here. New CLI commands are prefixed `plan-` so original reproduction commands still work.

## 9. Limits and the next learning step

The world is known and static, the state is exact, and the collision shape is conservative. There is no collision-avoidance controller, online replanning, robust tube guarantee, sensing delay, contact/slip physics, map uncertainty, estimator, or hardware test. C1 reference joins allow acceleration discontinuities. A single noise seed is not a robustness estimate. The rover finishes moving and cannot use the current flatness inversion to park from arbitrary standstill.

The next milestone can replace perfect state with explicit sensors and localization, then examine how estimation error affects tracking and clearance. Mapping/SLAM, exploration decisions, replanning, return-home behavior, and a deliberate stop/park transition remain later work.
