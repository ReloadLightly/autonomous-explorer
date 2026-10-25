# Localization sources and implementation boundary

Primary mathematical references consulted on 2026-10-25:

- [Principles of Robot Autonomy, Chapter 5 · Introduction to Robot Sensors](https://porabook.com/book/robot-sensors/): sensor modeling context.
- [Chapter 12 · Approximate Filters for State Estimation](https://porabook.com/book/approximate-filters/): Gaussian filtering, linearization, prediction/correction, and covariance updates.
- [Chapter 13 · Robot Localization](https://porabook.com/book/robot-localization/): localization using a motion model and observations of known surroundings.
- [MathWorks · Extended and Unscented Kalman Filter Algorithms for Online State Estimation](https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html): discrete nonlinear transition/measurement Jacobians and multirate corrections.
- [UC Berkeley EECS C106B, Spring 2023 · Project 3: State Estimation](https://ucb-ee106.github.io/106b-sp23site/assets/proj/proj3.pdf), developed by Fangyu Wu: educational comparison of commanded dead reckoning and sensor-corrected estimation. Its robot state and discretization differ from this project.

The implementation is original NumPy code. No upstream exercise solution, book PDF, MATLAB code, or Berkeley scaffold is incorporated. This milestone does not claim to reproduce a particular upstream notebook. Earlier upstream audits remain recorded in [sources.md](sources.md) and [navigation-sources.md](navigation-sources.md).

Project-specific choices include the extended four-state rover, exact differentiation through its RK4 stages, three held process-error channels, speed and identified range/bearing sensors, opaque rectangular geometry, three fixed seeds, a fixed yaw bias, and a forced landmark outage. Their equations, units, timing, and limitations are specified in [localization.md](localization.md). Noise levels and outcomes are experiment choices, not numerical results attributed to these sources.
