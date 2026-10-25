# SLAM sources and implementation boundary

Primary reference consulted on 2026-10-25:

- [Principles of Robot Autonomy, Chapter 14 · Simultaneous Localization and Mapping](https://porabook.com/book/slam/), especially its EKF-SLAM treatment: joint rover/landmark estimation, dense covariance, reobservations, and limits from linearization and correspondence assumptions.

This milestone implements an original small NumPy EKF-SLAM back end around the project's existing extended-unicycle rover and synthetic feature observations. No upstream code, notebook solution, book PDF, or third-party SLAM library is incorporated. Earlier sensor/filter references remain in [localization-sources.md](localization-sources.md).

Project-specific choices include the four-state rover, differentiated RK4 prediction, inverse-observation covariance augmentation, perfect supplied feature IDs, the five-second revisit rule, finite initial pose prior, prescribed loop, matched denied-revisit replays, and retained outage failures. These choices and numerical outcomes are specified in [slam.md](slam.md); they are not results attributed to the source chapter.

The implementation estimates current rover state and static point coordinates. It does not implement a camera/lidar front end, ambiguous data association, graph smoothing, occupancy reconstruction, or autonomous exploration. Supplied obstacle geometry remains separate from the initially unknown landmark map.
