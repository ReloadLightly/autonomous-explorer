# Project continuation and delivery

This document records the published project state. It is not an independent rerun of the scientific results. Read the linked evidence and current code before continuing.

## Published milestone status

Implementation snapshot: milestone 05 at 2b4fba64f88c180c2f4c44ac1a502250a9fcf0da on 2026-10-25. Subsequent documentation updates preserve these results and improve gallery access.

| Milestone | Published status | Evidence |
| --- | --- | --- |
| 01: Trajectory tracking | Reported complete | [MILESTONE_01.md](MILESTONE_01.md), [results](results/milestone/) |
| 02: Obstacle-aware navigation | Reported complete | [MILESTONE_02.md](MILESTONE_02.md), [results](results/navigation/) |
| 03: Sensing and localization | Reported complete | [MILESTONE_03.md](MILESTONE_03.md), [results](results/localization/) |
| 04: Landmark mapping and SLAM | Reported complete | [MILESTONE_04.md](MILESTONE_04.md), [results](results/slam/) |
| 05: Decision-making and viewpoint exploration | Reported complete | [Protocol](docs/exploration.md), [verification](docs/exploration-verification.md), [results](results/exploration/) |
| 06: Unknown-space occupancy mapping and exploration | Next in current README | Scope below |
| Later: Precise goal regulation | Planned in current README | Stationary parking and final-pose regulation |
| Later: Learning-based decision-making | Provisional | Select a specific Chapter 18 extension after the classical exploration system is established |

## Current implementation boundary

The current mission selects among supplied candidate observation viewpoints and uses a supplied static obstacle map. Landmark positions are learned by SLAM. The project does not yet demonstrate unknown-space occupancy reconstruction or frontier discovery. Its mission and safety claims must retain the qualifications in the current README.

## Cumulative visual gallery

The [complete visual gallery](GALLERY.md) now collects all 40 figures and seven animations, including SVG versions and 21 retained animation review frames. The README links to it prominently and displays all seven animations across the current milestone and visual history. Append future milestones while preserving earlier gallery sections and previews.

The gallery navigation update was checked on 2026-10-25: all links and section anchors resolve, all 89 referenced media assets are tracked and reachable, and all 52 media files from the earlier navigation-era commit remain unchanged. The five existing curated-gallery/provenance checks passed. This documentation check did not rerun simulation campaigns. The next implementation step remains milestone 06 below.

## Next implementation step

Continue with the README's unknown-space occupancy mapping and exploration milestone rather than restarting an earlier stage.

Before implementing, inspect the current sensing, landmark estimation, planning, mission, evidence, and rendering interfaces. Consult the book's relevant sensor, mapping, planning, and decision-making sections and primary sources for any added occupancy/frontier method. Write the concrete assumptions and comparison protocol before running experiments.

The milestone should connect range measurements to an evolving free/occupied/unknown representation, select exploration goals from that representation, and retain return-home behavior. Keep hidden ground-truth geometry outside the robot's decision interfaces and use it only for simulation and evaluation. Specify how pose uncertainty affects mapping and planning.

Completion requires a runnable integrated mission, saved sensor/decision evidence, replayable comparisons, appropriate checks of geometry and information access, actual-world versus robot-map visualizations, an updated walkthrough, and a verified GitHub push. Retain unsuccessful missions and state the limits of any collision or completion claims.

Later design choices remain provisional until this milestone's evidence exists; do not silently add unrelated training objectives.

## Delivery and operational status

- [x] Root AGENTS.md records the standing expectations for verified progress pushes, accurate GitHub About metadata, visual evidence, and continuity.
- [x] This continuation file reflects the already published milestones.
- [ ] GitHub About description and topics verified. They were empty when inspected on 2026-10-25. The connected file API does not expose repository metadata updates; complete this via an authenticated metadata-capable connection and update this status afterward.

For every completed task, update the relevant milestone, evidence links, blockers, and next step, then follow AGENTS.md's publication and verification instructions.
