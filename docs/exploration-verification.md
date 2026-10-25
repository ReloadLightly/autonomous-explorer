# Milestone 05 verification record

Verified on 2026-10-25 using Python 3.13.5 and NumPy 2.5.4. The [manifest](../results/exploration/manifest.json) records runtime, configuration, source hashes, and numerical input/output hashes.

## Completed checks

- **113 tests passed**, including all 94 earlier tests and 19 new exploration checks. Coverage includes continuous-to-grid connectors and unreachable worlds; utility changes from coverage and estimated map uncertainty; braking/waiting/recovery/timeout transitions; recovery hysteresis; zero-speed turning/control; return decisions; pose-correction replanning; no reachable plans; truth/schedule-free mission replay; logged route checks; paired noise; actual-home auditing; result preservation; and gallery provenance.
- `ruff check src tests`, `ruff format --check src tests`, and `git diff --check` passed. `uv lock --check --offline` passed; no runtime dependency was added.
- All **18 mission/SLAM/control replays** verified exactly from recorded observations and applied commands. Verification also recalculated summary metrics, actual clearance, candidate utilities, decisions, transitions, and routes.
- All four earlier milestone result sets passed their existing verification commands. Earlier numerical evidence and galleries were preserved, with the Milestone 04 walkthrough archived separately.
- An independent full reproduction into `build/exploration-reproduced/` matched **477 arrays in 21 NPZ files**, all run metadata (including every candidate utility, plan and transition), terminal statuses, metrics, resolved configuration, and summary. Manifest creation times naturally differ.
- All **ten PNG/GIF outputs** matched the independent reproduction byte for byte. The GIF has **144 frames**, at **1150×680 pixels**, lasting **12.8 s** including its final hold. SVG byte identity is not claimed because of ordinary generated metadata.
- All **5,737 recorded joint covariance snapshots** were symmetric and positive definite on their discovered-state subspaces. The smallest eigenvalue was approximately **5.20981×10⁻⁶**. Unknown slots were excluded using the explicit NaN mask.
- Current numerical-source hashes match the release manifest. Every run's state/controller history and N−1 applied-control length were checked.

Commands run in the existing virtual environment included:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explorer-mpl .venv/bin/pytest -q
.venv/bin/ruff check src tests
.venv/bin/ruff format --check src tests
git diff --check
UV_CACHE_DIR=/tmp/explorer-uv-cache uv lock --check --offline
.venv/bin/explorer verify --results results/milestone
.venv/bin/explorer plan-verify --results results/navigation
OPENBLAS_NUM_THREADS=1 .venv/bin/explorer localize-verify --results results/localization
OPENBLAS_NUM_THREADS=1 .venv/bin/explorer slam-verify --results results/slam
OPENBLAS_NUM_THREADS=1 .venv/bin/explorer explore-verify --results results/exploration
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explorer-mpl .venv/bin/explorer explore-reproduce --results build/exploration-reproduced/results --media build/exploration-reproduced/media
```

Cache settings accommodate this workspace's read-only home configuration directories. The README provides equivalent `uv run --frozen` commands for a fresh checkout.

## Observed outcomes

Supervised missions under nominal sensing and temporary loss completed for all three seeds, met actual home-arrival tolerances, and maintained positive interval clearance audits. Actual final home distances ranged from **0.114 to 0.167 m**; actual absolute stored speeds ranged from **0.024 to 0.042 m/s**.

During persistent loss, supervised missions brake, wait, and abort at **29.85 s**, preserving positive clearance in all three cases. Actual absolute final stored speeds were below **0.004 m/s**. They remain away from home and do not claim arrival.

The unsupervised persistent-loss baseline reports completion in all three runs, but ends **2.097–3.979 m** from actual home with envelope violations. The independent home audit rejects all three. Under temporary loss, all three baseline missions abort with `no_reachable_goal` following pose corrections; seed 41 also violates the collision envelope. Those failures remain in the table, logs, and images.

Modelled coverage is **99.1%** for nominal and recovered supervised missions and **44.5%** for persistent-loss aborts. This is projected visibility over a supplied map from estimated poses, not a measurement of reconstructed occupancy or unknown-space exploration success.

## Visual and causal review

The six figures, three SVG versions, one GIF, and three retained representative GIF frames were reviewed for path distinctions, explicit supplied geometry, state histories, utility scores, actual clearance, observation age, and the separation of reported completion from actual home arrival. Terminal animation panels hold their last recorded sample with an explicit “log ended” label.

[Gallery provenance](../media/exploration/provenance.json) hashes the exact numerical manifest, inputs, rendering sources, and output files. Automated tests check provenance and the README summary. Rendering tests forbid the physical mission simulator; replay of the estimator and policy is allowed as an evidence audit.

Causal replay tests remove true landmark coordinates and outage schedules and poison true state/disturbance arrays. The same SLAM posteriors, coverage, goals, plans, state transitions, and requested commands are reproduced. The supplied obstacle map is retained because it is an explicit planning input.

## Scope and limits

The experiment uses known obstacle geometry, supplied candidate observation viewpoints, identified static landmarks, synchronous synthetic observations, a local initial-pose prior, and a kinematic rover. Greedy utility selection and localization-health thresholds are heuristics. Clearance is an audit conditioned on numerical endpoints, without a formal global RK4 error enclosure.

Low-speed arrival and braking behavior do not certify exact stationary parking. The model's translational disturbance remains active at zero stored speed. No hardware, arbitrary-disturbance safety, learned obstacle occupancy, frontier discovery, or optimal exploration policy is claimed. Unknown-space occupancy mapping and exploration is the next milestone; final-pose parking remains separate.
