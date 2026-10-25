# Milestone 04 verification record

Verified locally on 2026-10-25 with Python 3.13.5 and NumPy 2.5.4. [The release manifest](../results/slam/manifest.json) records the runtime, source hashes, input files, statuses, and replay entries.

## Completed checks

- **94 tests passed**, including all 77 earlier tests and 17 new SLAM checks. New coverage includes independent finite differences for inverse and full joint observation Jacobians; single-use initialization; pose-to-map and map-to-map covariance augmentation; motion propagation of cross-covariance; correction of a correlated unobserved feature; branch-cut wrapping; denied revisits; invalid measurements/duplicate IDs; empty-map construction; replay with truth removed; estimated-state controller commands; forced invalid-speed partial logs; outage observations; saved-data rendering; evidence preservation; and gallery provenance.
- `ruff check src tests`, `ruff format --check src tests`, and `git diff --check` passed. `uv lock --check --offline` passed; no dependency was added.
- Tracking, navigation, and localization evidence passed all three existing verification commands. Their numerical results and media were preserved; the Milestone 03 walkthrough is archived separately.
- SLAM verification passed for **18 closed-loop executions and six estimator-only denied-revisit replays**. Verification checks hashes, recomputed metrics/tables/clearance, exact estimator replay, full logged joint snapshots, and controller-state selection.
- An independent full reproduction into `build/slam-reproduced/` matched **all 500 arrays in 28 NPZ files**, both metric files, configuration, tables, planning summary, and reference certificate. Creation timestamps in manifests naturally differ.
- All **ten PNG/GIF outputs** matched the independent reproduction byte for byte. The GIF contains 120 frames at 1150×680 pixels and lasts 10.8 s including the final hold. SVGs retain Matplotlib's ordinary generated metadata rather than a byte-identity claim.
- All **5,896 recorded joint covariance snapshots** were checked on their discovered-state subspaces. They were symmetric and positive definite; the smallest eigenvalue was approximately **1.59494×10⁻⁸**. Unknown landmark slots were excluded using the explicit NaN diagonal mask.
- Current numerical and render-source hashes match their release manifests. State/controller history lengths and N−1 control lengths were checked for every run.

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
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explorer-mpl .venv/bin/explorer slam-reproduce --results build/slam-reproduced/results --media build/slam-reproduced/media
```

Temporary cache settings accommodate this workspace's read-only home configuration directories. The README gives equivalent `uv run --frozen` commands for a fresh checkout.

## Observed outcomes

All nine normal-sensing integrations completed and remained clear. SLAM discovered all six landmarks for every seed. Its mean pose RMS was 0.1489 m, final raw map RMS was 0.1289 m, and map pair-distance RMS was 0.0299 m. The privileged supplied-map filter's pose RMS was 0.0236 m.

On the same recorded nominal SLAM trajectories, denying feature revisits increased mean pose RMS to 0.2199 m and pair-distance RMS to 0.1268 m. Absolute coordinate error did not improve for every seed: nominal seed 7's final raw map RMS was 0.2237 m with ordinary SLAM versus 0.2068 m with revisits denied, while its map pair-distance error improved. Both outcomes are retained. Relative map structure and frame-anchored coordinate error are distinct measurements.

The outage runs were not universally clear:

- Supplied-map EKF: two clear completions; seed 19 has an envelope violation.
- First-sighting-only mapping: three completed executions with envelope violations.
- SLAM: seed 7 completes clear; seed 19 completes with **23 envelope-overlap samples**; seed 41 exits the positive-speed domain at **53.45 s**, retaining **1,069 of 1,320** planned control intervals and its final partial posterior.

Thus 17 of the 18 physical integrations complete. Summary means retain the recorded prefix RMS and last available map for the early exit, with status and horizon recorded separately. Neither map recovery nor numerical completion is labelled as a runtime safety guarantee.

## Visual review

The six figures, three SVG versions, one GIF, and three representative GIF frames use saved evidence. The final map comparison, discovery sequence, covariance/correlation panels, matched revisit comparison, unsafe outage histories, and animation snapshots were inspected. Actual paths remain separate from estimated paths; truth crosses are labelled as evaluation overlays. Map ellipses use the latest recorded joint posterior and share the panel's spatial window.

[Gallery provenance](../media/slam/provenance.json) hashes the exact numerical manifest, numerical inputs, rendering sources, and outputs. Automated tests check provenance and the README's generated table. Renderer tests forbid planner and physical-experiment entrypoints; replaying filter equations is allowed as input verification.

## Scope and remaining work

The landmark positions are initially unknown to SLAM; their identities and the routing obstacle map are supplied. The finite initial rover prior anchors the coordinate frame, and the estimator never fits a truth alignment. This is current-state EKF SLAM, with dense covariance and repeated-linearization limits, rather than historical-trajectory smoothing or a perception front end.

There is no obstacle occupancy learning, exploration goal selection, runtime collision monitor, safe stop, or hardware validation. Clearance remains an audit conditioned on numerical integration endpoints, with no formal global RK4 error enclosure. The prescribed loop ends moving rather than parked. Decision-making and exploration is next, with localization-health handling motivated by the retained failure cases.
