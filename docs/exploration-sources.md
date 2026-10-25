# Decision-making sources and implementation boundary

Primary references consulted on 2026-10-25:

- [Principles of Robot Autonomy, Chapter 16 · Finite State Machines](https://porabook.com/book/finite-state-machines/): discrete mission modes, observation-driven transitions, and the limits of manually designed state machines.
- [Chapter 17 · Sequential Decision Making and Dynamic Programming](https://porabook.com/book/decision-making/): background on action objectives. This milestone uses a greedy utility heuristic and does not implement dynamic programming or claim optimal decisions.

The mission policy, coverage proxy, continuous-to-grid connector, zero-speed waypoint law, threshold values, and experiments are original project choices. No book code, exercise solution, PDF, or external decision-making package is incorporated. Previous sources for geometry, sensors, filtering, and SLAM remain in their milestone source notes.

The implementation uses a supplied static routing map and candidate observation viewpoints. Unknown landmark coordinates remain inside synthetic sensing/evaluation and the learned SLAM map. It does not implement frontier discovery, unknown-space occupancy reconstruction, formal chance constraints, or a certified collision-avoidance controller. The exact assumptions and measured failures are documented in [exploration.md](exploration.md).
