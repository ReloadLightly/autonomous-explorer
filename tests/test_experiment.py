"""Paired exogenous inputs, saved evidence, failure reporting, and rendering provenance."""

from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.control import Feedback, OpenLoop
from autonomous_explorer.evidence import (
    load_log,
    metrics,
    read_json,
    run_experiments,
    sha256,
    verify_evidence,
)
from autonomous_explorer.model import State, step
from autonomous_explorer.reference import CubicReference
from autonomous_explorer.simulation import disturbances, paired_runs, simulate, time_grid

CONFIG = Path(__file__).parents[1] / "configs/milestone.json"


def setup_case():
    config = read_json(CONFIG)
    reference = CubicReference(
        State.from_array(config["initial_state"]),
        State.from_array(config["final_state"]),
        config["duration_s"],
    )
    return config, reference, time_grid(config["duration_s"], config["dt_s"])


def test_disturbance_injection_is_speed_and_yaw_not_acceleration():
    actual = step([0, 0, 0.5, 0], [0, 0], 2, [0.2, 0.3])
    np.testing.assert_allclose(actual[2:], [0.5, 0.6], atol=1e-14)
    expected = [0.7 / 0.3 * np.sin(0.6), 0.7 / 0.3 * (1 - np.cos(0.6))]
    # Large single step used deliberately: RK4 has small but nonzero quadrature error.
    np.testing.assert_allclose(actual[:2], expected, atol=1e-4)


def test_same_sample_held_across_all_integrator_stages():
    initial = State(0, 0, 0.5, 0)
    times = np.array([0, 0.05, 0.10])
    exogenous = np.array([[0.2, 0.4], [-0.1, 0.3]])
    ref = CubicReference(initial, State(1, 0, 0.5, 0), 2)
    run = simulate(initial, ref, times, exogenous, OpenLoop(np.zeros((2, 2))))
    expected = step(step(initial.array(), [0, 0], 0.05, exogenous[0]), [0, 0], 0.05, exogenous[1])
    np.testing.assert_array_equal(run.states[-1], expected)


def test_yaw_bias_interval_is_half_open():
    config, _, times = setup_case()
    noise = disturbances(config["scenarios"][1], times)
    np.testing.assert_allclose(noise[:, 0], 0)
    assert np.count_nonzero(noise[:, 1]) == 80
    assert noise[160, 1] == 0.18
    assert noise[239, 1] == 0.18
    assert noise[240, 1] == 0


def test_paired_comparisons_and_seeded_reproducibility():
    config, reference, times = setup_case()
    for scenario in config["scenarios"]:
        runs = paired_runs(config, reference, times, scenario)
        left, right = runs["open_loop"], runs["feedback"]
        np.testing.assert_array_equal(left.disturbance, right.disturbance)
        np.testing.assert_array_equal(left.times, right.times)
        np.testing.assert_array_equal(left.states[0], right.states[0])
        assert left.status["status"] == right.status["status"] == "completed"
    repeated = paired_runs(config, reference, times, config["scenarios"][-1])
    for name in repeated:
        np.testing.assert_array_equal(repeated[name].states, runs[name].states)
        np.testing.assert_array_equal(repeated[name].requested, runs[name].requested)


def test_realized_open_loop_commands_independent_of_initial_error():
    _, reference, times = setup_case()
    commands = np.tile([0.03, 0.1], (len(times) - 1, 1))
    first = simulate(
        State(0, 0, 0.5, 0), reference, times, np.zeros_like(commands), OpenLoop(commands)
    )
    second = simulate(
        State(2, -1, 0.8, 1), reference, times, np.zeros_like(commands), OpenLoop(commands)
    )
    np.testing.assert_array_equal(first.requested, second.requested)
    assert not np.allclose(first.states, second.states)


def test_identical_limits_log_requested_applied_and_saturation():
    _, reference, _ = setup_case()
    run = simulate(
        State(0, 0, 0.5, 0),
        reference,
        [0, 0.05],
        np.zeros((1, 2)),
        OpenLoop([[3, -4]]),
        limits=[1, 2],
    )
    np.testing.assert_array_equal(run.requested, [[3, -4]])
    np.testing.assert_array_equal(run.applied, [[1, -2]])
    np.testing.assert_array_equal(run.saturated, [[True, True]])


def test_zero_speed_failure_retains_initial_state_and_no_fake_final_metric():
    _, reference, times = setup_case()
    run = simulate(State(0, 0, 0, 0), reference, times, np.zeros((len(times) - 1, 2)), Feedback())
    assert run.status["status"] == "invalid_domain"
    assert run.status["failure_time_s"] == 0
    assert run.status["invalid_domain_events"] == 1
    assert run.states.shape == (1, 4)
    assert run.applied.shape == (0, 2)
    log = {
        "states": run.states,
        "reference_states": reference.arrays(run.times)[0],
        "times": run.times,
        "applied": run.applied,
        "saturated": run.saturated,
    }
    outcome = metrics(log, run.status, 1e-4)
    assert outcome["final_position_m"] is None


@pytest.mark.parametrize("times", [[0, 0.05], [0, 0.05, 0.1]])
def test_crossing_speed_domain_retains_offending_state(times):
    _, reference, _ = setup_case()

    class BrakingFeedback(Feedback):
        def command(self, actual, desired):
            if actual.v <= self.min_speed:
                return super().command(actual, desired)
            return np.array([-20.0, 0.0])

    run = simulate(
        State(0, 0, 0.5, 0),
        reference,
        np.asarray(times),
        np.zeros((len(times) - 1, 2)),
        BrakingFeedback(),
    )
    assert run.status["status"] == "invalid_domain"
    assert run.status["failure_time_s"] == 0.05
    assert run.states.shape == (2, 4)
    assert run.states[-1, 2] < 0
    assert len(run.requested) == 1


@pytest.mark.parametrize("duration,dt", [(25, 0), (-1, 0.05), (25, float("nan")), (1, 0.3)])
def test_invalid_grids_are_rejected(duration, dt):
    with pytest.raises(ValueError):
        time_grid(duration, dt)


def test_saved_evidence_replays_exactly_and_detects_tampering(tmp_path):
    root = tmp_path / "logs"
    run_experiments(CONFIG, root)
    manifest = verify_evidence(root)
    config, reference, times = setup_case()
    repeated = paired_runs(config, reference, times, config["scenarios"][-1])
    for name, run in repeated.items():
        saved = load_log(root / f"seeded__{name}.npz")
        np.testing.assert_array_equal(saved["states"], run.states)
        np.testing.assert_array_equal(saved["requested"], run.requested)
        assert len(saved["disturbance"]) == len(saved["times"]) - 1
    assert len(manifest["runs"]) == 6
    with pytest.raises(ValueError, match="not empty"):
        run_experiments(CONFIG, root)
    (root / "metrics.json").write_text("{}")
    with pytest.raises(ValueError, match="checksum"):
        verify_evidence(root)


def test_curated_media_and_metrics_share_saved_results():
    repo = Path(__file__).parents[1]
    root, media = repo / "results/milestone", repo / "media"
    if not (media / "provenance.json").exists():
        pytest.skip("Run explorer reproduce to create the curated gallery")
    provenance = read_json(media / "provenance.json")
    manifest = verify_evidence(root)
    assert provenance["input_files_sha256"] == manifest["files_sha256"]
    assert provenance["results_manifest_sha256"] == sha256(root / "manifest.json")
    assert provenance["metrics_sha256"] == sha256(root / "metrics.json")
    for name, digest in provenance["outputs_sha256"].items():
        assert sha256(media / name) == digest
    assert len(list(media.glob("[0-9][0-9]_*.png"))) >= 10
    assert len(list(media.glob("*.gif"))) == 2
    archive = repo / "MILESTONE_01.md"
    readme = (archive if archive.exists() else repo / "README.md").read_text()
    table = readme.split("<!-- outcomes:start -->\n")[1].split("<!-- outcomes:end -->")[0]
    assert table == (root / "outcomes.md").read_text()


def test_renderer_does_not_simulate(tmp_path, monkeypatch):
    import autonomous_explorer.render as renderer
    import autonomous_explorer.simulation as sim

    root = tmp_path / "logs"
    run_experiments(CONFIG, root)

    def forbidden(*args, **kwargs):
        raise AssertionError("Rendering attempted to rerun the simulation")

    monkeypatch.setattr(sim, "simulate", forbidden)
    monkeypatch.setattr(sim, "paired_runs", forbidden)
    # A small smoke render exercises real plotting, but GIF encoding is tested via curated media.
    monkeypatch.setattr(renderer, "animations", lambda *args: None)
    renderer.render_gallery(root, tmp_path / "media")
    assert (tmp_path / "media/07_position_error.png").is_file()


def test_gif_retains_accents_absent_in_first_frame_and_exact_timing(tmp_path):
    from matplotlib.patches import Rectangle
    from PIL import Image, ImageColor

    from autonomous_explorer.render import animation_frames
    from autonomous_explorer.theme import AMBER, apply_theme, plt

    apply_theme()
    fig, ax = plt.subplots(figsize=(2, 1))
    ax.set(xlim=(0, 1), ylim=(0, 1))
    ax.axis("off")
    patch = Rectangle((0.2, 0.2), 0.6, 0.6, facecolor=AMBER, edgecolor=AMBER)
    ax.add_patch(patch)

    def update(k):
        patch.set_visible(k > 0)

    update.times = np.arange(3)
    output = tmp_path / "palette.gif"
    animation_frames(fig, update, [0, 1, 2], output, fps=12)
    with Image.open(output) as gif:
        duration = 0
        for k in range(gif.n_frames):
            gif.seek(k)
            duration += gif.info["duration"]
        pixels = np.asarray(gif.convert("RGB"))
        assert np.count_nonzero(np.all(pixels == ImageColor.getrgb(AMBER), axis=2)) > 10
        assert duration == 1050  # 3/12 s plus the 0.8 s final hold, in centiseconds.
