# Milestone 02 source audit

Inspected on 2026-10-25. The project started this milestone clean at `ff5004faf57ac3800d29abf0e0462f7054fd3a4e`; remote `main` matched. No applicable `AGENTS.md` was present. The existing model, control, evidence, theme, and Milestone 01 artifacts were inspected before adding planning.

The [book](https://porabook.com/), [exercise index](https://porabook.com/exercises/), and current [Chapter 4 web text](https://porabook.com/book/motion-planning/) were read. The anchors are configuration space (§4.1), grid-based planning (§4.2), and **Exercise 1: A* Motion Planning**, including its path-smoothing subsection. The chapter's later kinodynamic-planning discussion helps distinguish that larger problem from the geometric search plus validated kinematic bridge implemented here.

The upstream revision remains [`02ee1421786b73ae4810b086cd1f59882ab972bc`](https://github.com/StanfordASL/pora-exercises/commit/02ee1421786b73ae4810b086cd1f59882ab972bc), verified against the remote at the start of this milestone. Exact inspected files:

| Source | Inspection |
| :-- | :-- |
| [ch04/exercises/a_star.ipynb](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch04/exercises/a_star.ipynb) | Exercise 1.1 grid search; Exercise 1.2 cubic-spline smoothing; unfinished implementations |
| [ch04/exercises/a_star.py](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch04/exercises/a_star.py) | Eight-connected neighbors, Euclidean distance, labels/parents/open and closed sets, start-relative snapping, unfinished solver |
| [ch04/exercises/utils.py](https://github.com/StanfordASL/pora-exercises/blob/02ee1421786b73ae4810b086cd1f59882ab972bc/ch04/exercises/utils.py) | Rectangular workspace, separate boundary and point/line checks, scaled default margin, obstacle generation |

[`navigation-upstream-audit.json`](navigation-upstream-audit.json) records the source hashes. The pinned upstream repository still has no license grant identified in the original [license inspection](sources.md). No upstream code was incorporated. Our geometry, searches, reference construction, audits, tests, and rendering are original implementation; the book is linked, not vendored as a PDF.

## Choices made explicit

1. **Full robot versus point.** The upstream exercise is an introductory point-planning scaffold. Our point search uses inflated obstacles derived from the existing rover's body and wheels, plus stated reserves. The orientation-independent disk and axis-aligned offsets are conservative approximations.
2. **Edges versus endpoints.** Free endpoints are insufficient. We check entire edges continuously against axis-aligned boxes and additionally forbid diagonal grid-corner cutting. The exact same graph goes to A* and Dijkstra.
3. **Boundary/margin units.** The upstream helper separates `in_statespace` from `is_free` and scales a default point margin by workspace size. We combine bounds and obstacle checks, use metres explicitly, and reject contacts. Grid resolution is metres per step.
4. **Snapping.** Upstream snaps positions onto a lattice offset by the start. Our lattice has a declared world origin, uses integer indices, and rejects off-grid start/goal requests. The requested destination is never silently moved.
5. **Smoothing.** Upstream asks for a cubic spline through path nodes but does not supply collision or dynamic certification. We use lines and quadratic Bézier corners with a free-control-box certificate and C1 joins. This deliberate extension is not presented as the exact notebook spline solution. Acceleration discontinuities are disclosed.
6. **Path versus trajectory.** The new reference gets a positive-speed time law and continuous segmentwise speed/acceleration/yaw bounds before it reaches the original controller. Search is geometric; we do not claim to implement a kinodynamic search or continuous minimum-time optimizer.
7. **Planning versus execution.** The existing controller does not read obstacles. Clearance is audited after simulation, unsafe paths remain recorded, and the animation carries a persistent failure notice. A completed integration is not automatically called safe.
8. **Randomness.** The main map is deterministic, rather than repeatedly drawing clutter until a pleasing map succeeds. The seed-7 execution perturbation is declared in configuration. Sealed-world and narrow-passage planning failures are retained.

## Book-to-code map

| Study topic | Implementation | Evidence |
| :-- | :-- | :-- |
| Chapter 4 configuration space | `World.inflate`, `rover_radius`, signed clearance | Figure 01, envelope tests |
| Chapter 4 grid search / Exercise 1.1 | `grid_search`, exact segment tests, shortcuts | Figure 02, search GIF, resolution study, A*/Dijkstra cost tests |
| Exercise 1.2 path smoothing, extended with certification | `build_rounded_reference`, `RoundedReference.bounds` | Figures 03–04; collision-counterexample, C1 and extrema tests |
| Chapter 1 motion/integration and Chapter 3 tracking prerequisites | Original `State`, RK4 `step`, `Feedback`, `OpenLoop`, `paired_runs` | Figures 05–09; navigation GIF, paired seeded logs |
| Planning failure and discretization limits | Saved failure/resolution searches | Figures 10–11, explicit status records |

Book attribution: Daniele Gammelli, Joseph Lorenzetti, Katie Luo, Gioele Zardini, and Marco Pavone, *Principles of Robot Autonomy*, Cambridge University Press, 2026. The original [source audit](sources.md) includes the citation entry and tracking-scaffold interface resolutions.
