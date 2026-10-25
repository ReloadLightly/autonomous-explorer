# Milestone 03 verification record

Verified locally on 2026-10-25 with Python 3.13.5 and NumPy 2.5.4. Exact runtime details and numerical-source hashes are recorded in [the manifest](../results/localization/manifest.json).

## Completed checks

- **77 tests passed**, including the 62 existing tests for Milestones 01 and 02. New tests cover independent finite-difference checks of RK4 state/process sensitivities, landmark Jacobians, ±π innovations, range/occlusion/schedule rules, outages, positive covariance, independent random streams, controller input provenance, replay with deliberately corrupted truth, failed estimated-speed startup, directory preservation, and saved-data rendering.
- `ruff check src tests`, `ruff format --check src tests`, and `git diff --check` passed.
- `uv lock --check --offline` passed using the existing lockfile. No runtime dependency was added.
- The original tracking and navigation evidence both passed their existing verification commands.
- Localization verification passed for all **27 runs**, including checksums, recalculated metrics, regenerated summary/per-run tables, replayed posterior/covariance histories, exact controller-state selection, and recalculated clearance.
- An independent full reproduction into `build/localization-reproduced/` matched **all 527 arrays in 31 NPZ files**, all metrics, both tables, the resolved configuration, and the reference certificate. Equality includes the saved observations and noise inputs. Manifest creation timestamps naturally differ.
- All ten PNG/GIF outputs also matched the independent reproduction byte for byte. The GIF has 120 frames at 1350×650 pixels and lasts 10.8 s including its final hold.
- Current experiment-source hashes match the recorded release hashes. State/estimate/covariance lengths and N−1 control lengths were also checked for every release log.

Commands actually run in the existing virtual environment included:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explorer-mpl .venv/bin/pytest -q
.venv/bin/ruff check src tests
.venv/bin/ruff format --check src tests
git diff --check
UV_CACHE_DIR=/tmp/explorer-uv-cache uv lock --check --offline
.venv/bin/explorer verify --results results/milestone
.venv/bin/explorer plan-verify --results results/navigation
OPENBLAS_NUM_THREADS=1 .venv/bin/explorer localize-verify --results results/localization
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explorer-mpl .venv/bin/explorer localize-reproduce --results build/localization-reproduced/results --media build/localization-reproduced/media
```

The README provides equivalent `uv run --frozen` commands for a fresh checkout. Temporary cache settings above accommodate this workspace's read-only home configuration directories; they do not alter the experiment.

## Numerical outcomes

All 27 integrations completed. This is distinct from safe execution:

- Exact-state feedback had positive interval clearance bounds in all nine runs. Its shadow EKF still drifted during the outage.
- EKF feedback with ordinary landmark availability stayed clear in all six nominal/yaw-bias runs. Mean localization RMS was 0.0432 m nominal and 0.0433 m with yaw bias; mean tracking RMS was 0.0360 m and 0.0373 m. Its worst interval bound across these runs was 0.4445 m.
- EKF feedback during the landmark outage violated the conservative envelope for all three seeds. Mean localization RMS rose to 0.7065 m and tracking RMS to 0.7879 m. Saturation and poor uncertainty coverage remain in the logs.
- Command-only dead reckoning had an envelope violation for one nominal seed and all yaw-bias/outage seeds. Its outage and yaw-bias executions match because it ignores the observations in both cases.

All negative clearance and overlap counts are retained in [metrics.json](../results/localization/metrics.json). No failed run was dropped, relabelled as successful tracking, or used to claim a runtime safety guarantee.

## Visual review and provenance

The six PNG figures, three SVG versions, one GIF, and three retained animation review frames were generated from saved evidence. Sensor geometry, actual/estimated paths, clearance violations, covariance underestimation, seed outcomes, and the before/during/after animation frames were inspected. Overlapping animation labels found in the first rendering were moved below the spatial axes before generating the release gallery.

[Gallery provenance](../media/localization/provenance.json) records hashes for the exact input manifest, numerical inputs, render sources, and rendered outputs. Automated tests check that the README summary matches the saved summary and that the gallery's hashes match its files. Rendering is tested with experiment/planner entrypoints forbidden; estimator replay is allowed as an input audit.

## Limits

This evidence covers a supplied static world, known landmark identities/positions, ideal synchronous observation timing, a local initial-pose prior, and a kinematic rover. It does not validate SLAM, perception/data association, a stopping policy, online replanning, or hardware. The bias intentionally mismatches the EKF's zero-mean assumption, and the uncertainty diagnostics are descriptive rather than a statistical consistency certification. Clearance bounds are conditioned on numerical integration endpoints and omit a formal global RK4 error bound. The final reference speed remains positive.

Mapping and SLAM is the next planned milestone. The retained outage failures also provide a concrete test case for later localization-health handling and replanning.
