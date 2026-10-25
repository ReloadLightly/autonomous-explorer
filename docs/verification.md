# Milestone verification record

Executed locally on 2026-10-25 with Python 3.13.5, NumPy 2.5.4, Matplotlib 3.11.2, Pillow 12.3.0, and pytest 9.1.1. `uv.lock` records the dependency resolution. The pinned upstream sources were inspected before implementation; no upstream implementation was incorporated.

## Numerical and software checks

The focused test suite covers:

- Named state ordering, velocity projection, and acceleration/yaw derivatives with units.
- Straight travel with constant acceleration at multiple headings, and the analytical zero-yaw limit.
- Constant-turn motion against its analytical solution, with RK4 error reduction of approximately 16× when halving the timestep.
- Cubic endpoint positions and Cartesian velocities, interior derivatives, and the continuous minimum-speed check.
- Flatness inversion recovering requested Cartesian acceleration away from zero speed.
- Zero-error feedback equalling feedforward.
- The open-loop interface having no state input, copying its precomputed commands, and producing identical commands from different actual initial states.
- Identical disturbance arrays, grids, and initial conditions for paired runs, including the half-open bias interval.
- Exactly repeated seed-7 state and command arrays.
- Speed/yaw disturbance injection, with fixed disturbance samples across RK4 stages.
- Optional input clipping and requested/applied/saturation logging.
- Wrapped heading errors across the ±π boundary.
- Explicit zero/negative/near-threshold speed handling and retained histories after crossing the speed domain, including at the final timestamp.
- Invalid grid rejection, evidence-directory preservation, and tamper detection.
- Metrics recomputation from logs, gallery/input checksums, and the README table matching the generated saved-data table.
- Rendering with simulator functions replaced by failures, ensuring plotting cannot silently rerun the experiment.

Commands:

```bash
uv run --frozen pytest -q
uv run --frozen ruff check src tests
uv run --frozen ruff format --check src tests
uv run --frozen explorer verify --results results/milestone
```

The final local suite passed **35 tests**, with no skips, including a GIF palette/timing regression check. A [GitHub Actions workflow template](checks-workflow.yml) runs the same checks using the committed lockfile when installed as `.github/workflows/checks.yml`. It is retained as documentation rather than activated: GitHub rejected the initial push because the available OAuth token lacks `workflow` scope. Local verification is complete; no remote CI success is claimed.

## Clean reproduction and replay

The documented full command was executed with the output directory absent:

```bash
uv run --frozen explorer reproduce --results build/reproduced/results --media build/reproduced/media
```

All six main runs completed. Comparing to the independent first run showed **84 saved arrays exactly equal** across the scenario logs, singularity probe, primitives, and integration study; all metric values matched exactly. The final numerical source fingerprints match the source files delivered in this repository. The earlier development artifacts were kept outside the curated evidence directories.

The separate saved-data route was also executed:

```bash
uv run --frozen explorer render --results results/milestone --media media
```

This produced the curated gallery after visual adjustments, with `media/provenance.json` binding it to `results/milestone`. The numerical hashes stayed unchanged during rendering. All local documentation/media links were checked. No PDF, environment, cache, or temporary output is committed.

## Visual review

All eleven static figures were opened and inspected. The two GIFs were decoded and representative frames inspected, in addition to the six full-size PNG review frames. Review covered physical orientation, actual history trails, equal spatial aspect ratios, fixed paired limits, units, disturbance timing, error/command alignment, legend contrast, and text boundaries.

Adjustments made after inspection: moved geometry labels away from the rover, strengthened reference ghosts, used explicit timestep tick labels on the accuracy plot, moved the state-tracking legend away from the speed peak, extended step-command traces through the final interval, and derived disturbance captions from configuration. Decoded GIF review exposed an initial-palette issue that desaturated amber when the overlapping initial bodies hid that colour; reserving the theme accents fixed it. GIF timing distributes centisecond rounding across frames rather than accelerating every frame. The GIFs retain synchronized indices and the original saved simulation timestamps.

Each animation has 120 encoded frames and 10.8 seconds total playback, including the 0.8 s final hold. The hero is about 0.32 MB and the diagnostic replay about 0.42 MB; the entire curated media directory is about 2.5 MB. PNGs and selected SVGs can be regenerated at higher resolution from the numerical evidence. MP4 export was omitted because no ffmpeg encoder was available in the environment.

## Failures and practical limits

The initial analytical benchmark accidentally preserved integer timestamp dtype when filling constant speed. The straight-limit test exposed it; conversion to floating timestamps fixed the benchmark, and all saved evidence was regenerated afterward. This was a benchmark implementation bug, not a discarded tracking result.

The deliberate stopped-start feedback probe fails at t = 0 with one invalid-domain event and no fabricated final result. All six declared performance runs completed without saturation or domain exits. The observed feedback improvements apply to this exact-state, unbounded-actuation model and the three declared scenarios; one stochastic seed does not establish general robustness. The reference ends moving at 0.5 m/s. Further limits are listed in the [experiment report](experiment.md).

The initial local commit containing an active workflow was retained on the local `local/with-ci-template` branch. The delivery commit moves that optional workflow into this documentation directory; no remote history was overwritten or force-pushed. All source, numerical evidence, media, and local checks are included in the delivery.
