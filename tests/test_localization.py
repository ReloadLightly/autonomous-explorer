"""Sensor geometry, independent Jacobian checks, causal replay and closed-loop evidence."""

from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.control import Feedback
from autonomous_explorer.evidence import load_log, read_json, sha256, write_json
from autonomous_explorer.localization import ExtendedKalmanFilter, transition_jacobians
from autonomous_explorer.localization_evidence import (
    draw_inputs,
    replay_estimates,
    run_localization,
    simulate_localized,
    validate_config,
    verify_localization,
)
from autonomous_explorer.model import State, step, wrap_angle
from autonomous_explorer.path_reference import RoundedReference
from autonomous_explorer.planning import World
from autonomous_explorer.reference import CubicReference
from autonomous_explorer.sensors import Observation, landmark_model, sample_observation

ROOT = Path(__file__).parents[1]


def configuration():
    config = read_json(ROOT / "configs/localization.json")
    config["navigation"] = read_json(ROOT / "configs/navigation.json")
    return config


@pytest.mark.parametrize("yaw_rate", [0.0, 1e-8, 0.8, -1.4])
def test_rk4_sensitivities_match_independent_finite_differences(yaw_rate):
    state, control, dt = np.array([1.2, -0.4, 0.5, 3.13]), np.array([0.2, yaw_rate]), 0.12
    value, f, g = transition_jacobians(state, control, dt)
    np.testing.assert_allclose(value, step(state, control, dt), atol=1e-15)
    epsilon = 1e-6
    numeric_f = np.column_stack(
        [
            (step(state + delta, control, dt) - step(state - delta, control, dt)) / (2 * epsilon)
            for delta in np.eye(4) * epsilon
        ]
    )

    def disturbed(noise):
        return step(state, control + [noise[0], 0], dt, noise[1:])

    numeric_g = np.column_stack(
        [(disturbed(delta) - disturbed(-delta)) / (2 * epsilon) for delta in np.eye(3) * epsilon]
    )
    np.testing.assert_allclose(f, numeric_f, atol=2e-10, rtol=1e-7)
    np.testing.assert_allclose(g, numeric_g, atol=2e-10, rtol=1e-7)


def test_landmark_jacobian_and_angle_branch():
    state, beacon, epsilon = np.array([1.1, 0.3, 0.4, 3.13]), [3.5, -0.7], 1e-6
    _, h = landmark_model(state, beacon)
    numeric = []
    for delta in np.eye(4) * epsilon:
        diff = landmark_model(state + delta, beacon)[0] - landmark_model(state - delta, beacon)[0]
        diff[1] = wrap_angle(diff[1])
        numeric.append(diff / (2 * epsilon))
    np.testing.assert_allclose(h, np.array(numeric).T, atol=3e-10)
    with pytest.raises(ValueError, match="singular"):
        landmark_model(state, state[:2])


def test_sensor_schedule_range_occlusion_and_outage():
    sensors = configuration()["sensors"]
    sensors.update(landmarks_xy_m=[[2, 1], [5, 1], [1, 5], [1, 9]], max_range_m=5)
    world = World((0, 10, 0, 10), ((3, 0.1, 4, 2),))
    state, speed_noise, beacon_noise = np.array([1, 1, 0.4, 0]), np.zeros(21), np.zeros((21, 4, 2))
    packet = sample_observation(state, 0, 0, sensors, world, speed_noise, beacon_noise, None)
    np.testing.assert_array_equal(packet.landmark_ids, [0, 2])
    np.testing.assert_allclose(packet.range_bearing[:, 0], [1, 4])
    assert packet.speed == 0.4
    missing = sample_observation(state, 1, 0.02, sensors, world, speed_noise, beacon_noise, None)
    assert missing.speed is None and len(missing.landmark_ids) == 0
    outage = sample_observation(
        state, 10, 0.2, sensors, world, speed_noise, beacon_noise, [0.2, 0.4]
    )
    assert outage.speed == 0.4 and len(outage.landmark_ids) == 0
    recovered = sample_observation(
        state, 20, 0.4, sensors, world, speed_noise, beacon_noise, [0.2, 0.4]
    )
    np.testing.assert_array_equal(recovered.landmark_ids, [0, 2])


def test_wrapped_bearing_innovation_and_positive_covariance():
    sensors = configuration()["sensors"]
    sensors["landmarks_xy_m"] = [[2, 0]]
    estimator = ExtendedKalmanFilter(
        [0, 0, 0.4, np.pi - 0.01], np.eye(4) * 0.01, [0.02, 0.08, 0.10], sensors
    )
    old = estimator.state.copy()
    _, nis = estimator.correct(Observation(None, np.array([0]), np.array([[2, np.pi - 0.01]])))
    assert abs(estimator.state[3] - old[3]) < 0.03
    assert nis[0] < 0.1
    for _ in range(200):
        estimator.predict([0, 0.1], 0.02)
        expected, _ = landmark_model(estimator.state, [2, 0])
        estimator.correct(Observation(0.4, np.array([0]), expected[None, :]))
        np.testing.assert_allclose(estimator.covariance, estimator.covariance.T, atol=1e-15)
        assert np.linalg.eigvalsh(estimator.covariance).min() > 0


def test_noise_streams_are_repeatable_and_plant_is_independent_of_sensor_configuration():
    config, times = configuration(), np.arange(21) * 0.02
    first, second = draw_inputs(config, times, 7), draw_inputs(config, times, 7)
    for key in first:
        np.testing.assert_array_equal(first[key], second[key])
    config["sensors"]["landmarks_xy_m"].append([2, 2])
    np.testing.assert_array_equal(
        first["disturbance"], draw_inputs(config, times, 7)["disturbance"]
    )


@pytest.fixture(scope="module")
def evidence(tmp_path_factory):
    root = tmp_path_factory.mktemp("localization")
    config = configuration()
    config["seeds"] = [7]
    write_json(root / "config.json", config)
    run_localization(root / "config.json", root / "results")
    return root / "results"


def test_closed_loop_controller_uses_estimates_and_replays_without_truth(evidence):
    config = read_json(evidence / "config.json")
    log = load_log(evidence / "yaw_bias__7__ekf.npz")
    np.testing.assert_array_equal(log["controller_states"], log["estimates"])
    assert np.max(np.linalg.norm(log["estimates"][:, :2] - log["states"][:, :2], axis=1)) > 0.01
    corrupted_truth = dict(log, states=np.full_like(log["states"], 123456.0))
    replay, covariance = replay_estimates(corrupted_truth, config, "ekf")
    np.testing.assert_array_equal(replay, log["estimates"])
    np.testing.assert_array_equal(covariance, log["covariances"])
    plan = load_log(evidence / "plan.npz")
    reference = RoundedReference(plan["bezier_controls"], plan["segment_durations"])
    nav = config["navigation"]
    controller = Feedback(nav["kp_per_s2"], nav["kd_per_s"], nav["min_forward_speed_mps"])
    for k in range(0, len(log["requested"]), 37):
        expected = controller.command(
            State.from_array(replay[k]), reference.sample(log["times"][k])
        )
        np.testing.assert_array_equal(log["requested"][k], expected)
    oracle = load_log(evidence / "yaw_bias__7__oracle.npz")
    np.testing.assert_array_equal(oracle["controller_states"], oracle["states"])
    np.testing.assert_array_equal(log["disturbance"], oracle["disturbance"])


def test_sensor_outage_is_retained_and_covariance_is_not_a_safety_certificate(evidence):
    log = load_log(evidence / "landmark_outage__7__ekf.npz")
    active = (log["times"] >= 18) & (log["times"] < 30)
    assert not np.any(log["landmark_visible"][active])
    assert np.any(np.isfinite(log["speed_observations"][active]))
    outcomes = read_json(evidence / "metrics.json")
    assert (
        outcomes["landmark_outage__7__ekf"]["rms_localization_m"]
        > outcomes["yaw_bias__7__ekf"]["rms_localization_m"]
    )
    assert outcomes["yaw_bias__7__dead_reckoning"]["disk_overlap_samples"] > 0
    assert outcomes["yaw_bias__7__dead_reckoning"]["execution_status"] == "envelope_violation"
    assert (
        outcomes["nominal__7__ekf"]["rms_localization_m"]
        < outcomes["nominal__7__dead_reckoning"]["rms_localization_m"]
    )


def test_evidence_verification_and_preservation(evidence):
    assert len(verify_localization(evidence)["runs"]) == 9
    with pytest.raises(ValueError, match="not empty"):
        run_localization(ROOT / "configs/localization.json", evidence)


def test_invalid_sensor_configuration_is_rejected():
    config = configuration()
    config["sensors"]["landmark_every_steps"] = 0
    with pytest.raises(ValueError, match="positive integer"):
        validate_config(config)
    config = configuration()
    config["sensors"]["landmarks_xy_m"] = [[3.5, 2]]
    with pytest.raises(ValueError, match="free-space"):
        validate_config(config)


def test_invalid_estimated_speed_is_logged_without_executing_a_command():
    config = configuration()
    config["initial_estimate_offset"][2] = -1.0
    reference = CubicReference(State(1, 1, 0.4, 0), State(1.8, 1, 0.4, 0), 2.0)
    times = np.linspace(0, 2, 101)
    log, status = simulate_localized(
        config,
        reference,
        times,
        draw_inputs(config, times, 7),
        config["scenarios"][0],
        "dead_reckoning",
    )
    assert status["status"] == "invalid_domain"
    assert status["failure_time_s"] == 0
    assert status["completed_intervals"] == 0
    assert log["states"].shape == (1, 4)
    assert log["applied"].shape == (0, 2)
    assert log["covariances"].shape == (1, 4, 4)


def test_renderer_uses_only_saved_data(evidence, tmp_path, monkeypatch):
    import autonomous_explorer.localization_evidence as experiment
    import autonomous_explorer.localization_render as rendering
    import autonomous_explorer.planning as planning

    def forbidden(*args, **kwargs):
        raise AssertionError("Rendering may not rerun the experiment")

    monkeypatch.setattr(experiment, "simulate_localized", forbidden)
    monkeypatch.setattr(planning, "grid_search", forbidden)
    monkeypatch.setattr(rendering, "localization_animation", lambda *args: None)
    rendering.render_localization(evidence, tmp_path)
    assert (tmp_path / "04_clearance.png").exists()


def test_committed_localization_gallery_provenance():
    root, media = ROOT / "results/localization", ROOT / "media/localization"
    if not (media / "provenance.json").exists():
        pytest.skip("Run explorer localize-reproduce to produce the curated gallery")
    manifest, provenance = read_json(root / "manifest.json"), read_json(media / "provenance.json")
    assert provenance["results_manifest_sha256"] == sha256(root / "manifest.json")
    assert provenance["input_files_sha256"] == manifest["files_sha256"]
    for name, digest in provenance["outputs_sha256"].items():
        assert sha256(media / name) == digest
    assert len(list(media.glob("[0-9][0-9]_*.png"))) == 6
    readme = (ROOT / "MILESTONE_03.md").read_text()
    table = readme.split("<!-- localization-summary:start -->\n")[1].split(
        "<!-- localization-summary:end -->"
    )[0]
    assert table == (root / "summary.md").read_text()
