"""Joint-state math, unknown-map boundaries, first-sighting augmentation, and revisit evidence."""

import copy
from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.control import Feedback
from autonomous_explorer.evidence import load_log, read_json, sha256, write_json
from autonomous_explorer.localization import transition_jacobians
from autonomous_explorer.localization_evidence import draw_inputs
from autonomous_explorer.model import State, wrap_angle
from autonomous_explorer.path_reference import RoundedReference
from autonomous_explorer.sensors import Observation, landmark_model
from autonomous_explorer.slam import EKFSLAM, inverse_observation, joint_observation
from autonomous_explorer.slam_evidence import (
    new_estimator,
    replay_slam,
    run_slam,
    simulate_slam,
    simulator_sensors,
    validate_slam_config,
    verify_slam,
)

ROOT = Path(__file__).parents[1]


def estimator():
    return EKFSLAM(
        [0, 0, 0.4, 0.2],
        np.diag([0.04, 0.06, 0.01, 0.03]),
        [0.02, 0.08, 0.1],
        [0.025, 0.05, 0.015],
        revisit_gap_s=2,
    )


def test_inverse_observation_jacobians():
    rover, reading, epsilon = np.array([1.2, -0.4, 0.5, 3.13]), np.array([2.5, -3.1]), 1e-6
    point, pose, measurement = inverse_observation(rover, reading)
    numeric_pose = np.column_stack(
        [
            (
                inverse_observation(rover + delta, reading)[0]
                - inverse_observation(rover - delta, reading)[0]
            )
            / (2 * epsilon)
            for delta in np.eye(4) * epsilon
        ]
    )
    numeric_reading = np.column_stack(
        [
            (
                inverse_observation(rover, reading + delta)[0]
                - inverse_observation(rover, reading - delta)[0]
            )
            / (2 * epsilon)
            for delta in np.eye(2) * epsilon
        ]
    )
    np.testing.assert_allclose(pose, numeric_pose, atol=5e-10)
    np.testing.assert_allclose(measurement, numeric_reading, atol=5e-10)
    recovered, _ = landmark_model(rover, point)
    np.testing.assert_allclose(recovered, reading, atol=1e-14)


def test_joint_measurement_jacobian_includes_landmark_columns():
    joint, epsilon = np.array([1.2, -0.4, 0.5, 3.13, 2.5, 1.5, -1, 2]), 1e-6
    _, h = joint_observation(joint, 0)
    numeric = []
    for delta in np.eye(len(joint)) * epsilon:
        difference = joint_observation(joint + delta, 0)[0] - joint_observation(joint - delta, 0)[0]
        difference[1] = wrap_angle(difference[1])
        numeric.append(difference / (2 * epsilon))
    np.testing.assert_allclose(h, np.array(numeric).T, atol=4e-10)
    np.testing.assert_array_equal(h[:, 4:6], -h[:, :2])
    np.testing.assert_array_equal(h[:, 6:], 0)


def test_augmentation_keeps_cross_correlations_and_does_not_double_count_reading():
    e = estimator()
    original = e.covariance.copy()
    z = np.array([2, 0.3])
    point, a, b = inverse_observation(e.state, z)
    events, nis = e.correct(Observation(None, np.array([10]), z[None]), 0)
    assert events["new"] == [10] and nis == {}
    np.testing.assert_array_equal(e.covariance[:4, :4], original)
    np.testing.assert_allclose(e.state[4:], point)
    np.testing.assert_allclose(e.covariance[4:, :4], a @ original)
    np.testing.assert_allclose(
        e.covariance[4:, 4:], a @ original @ a.T + b @ e.measurement_covariance @ b.T
    )
    before = e.covariance.copy()
    _, second_a, _ = inverse_observation(e.state[:4], [3, -0.4])
    e.correct(Observation(None, np.array([20]), np.array([[3, -0.4]])), 0)
    np.testing.assert_allclose(e.covariance[6:, :6], second_a @ before[:4, :])
    assert np.linalg.norm(e.covariance[6:, 4:6]) > 0.01
    assert np.linalg.eigvalsh(e.covariance).min() > 0


def test_prediction_propagates_pose_map_cross_covariance():
    e = estimator()
    e.correct(Observation(None, np.array([0]), np.array([[2, 0.3]])), 0)
    before, state = e.covariance.copy(), e.state.copy()
    rover, f, g = transition_jacobians(state[:4], [0.1, 0.4], 0.1)
    e.predict([0.1, 0.4], 0.1)
    np.testing.assert_allclose(e.state[:4], rover)
    np.testing.assert_array_equal(e.state[4:], state[4:])
    np.testing.assert_allclose(e.covariance[:4, 4:], f @ before[:4, 4:])
    np.testing.assert_array_equal(e.covariance[4:, 4:], before[4:, 4:])
    np.testing.assert_allclose(e.covariance[:4, :4], f @ before[:4, :4] @ f.T + g @ e.process @ g.T)


def test_revisit_updates_unobserved_correlated_landmarks_and_covariance():
    e = estimator()
    e.correct(Observation(None, np.array([0]), np.array([[2, 0.3]])), 0)
    for _ in range(15):
        e.predict([0, 0.05], 0.1)
    e.correct(Observation(None, np.array([1]), np.array([[3, -0.4]])), 1.5)
    for _ in range(15):
        e.predict([0, 0.05], 0.1)
    before = e.state.copy()
    prior = e.covariance.copy()
    expected, _ = joint_observation(e.state, 0)
    events, _ = e.correct(Observation(None, np.array([0]), (expected + [0.15, 0.02])[None]), 3)
    assert events["revisited"] == [0]
    assert np.linalg.norm(e.state[:2] - before[:2]) > 1e-4
    assert np.linalg.norm(e.state[6:8] - before[6:8]) > 1e-5
    assert np.trace(e.covariance) < np.trace(prior)
    np.testing.assert_allclose(e.covariance, e.covariance.T, atol=1e-15)
    assert np.linalg.eigvalsh(e.covariance).min() > 0


def test_first_sighting_and_denied_revisit_never_reuse_old_readings():
    e = estimator()
    packet = Observation(None, np.array([0]), np.array([[2, 0.3]]))
    e.correct(packet, 0)
    state, p = e.state.copy(), e.covariance.copy()
    events, _ = e.correct(packet, 0.2, use_reobservations=False)
    assert events["skipped"] == [0]
    events, _ = e.correct(packet, 3, allow_revisits=False)
    assert events["skipped"] == [0]
    events, _ = e.correct(packet, 3.2, allow_revisits=False)
    assert events["skipped"] == [0]
    np.testing.assert_array_equal(e.state, state)
    np.testing.assert_array_equal(e.covariance, p)


def test_invalid_ranges_and_duplicate_ids_are_explicit():
    e = estimator()
    events, _ = e.correct(Observation(None, np.array([0, 1]), np.array([[-1, 0], [np.nan, 0]])), 0)
    assert events["rejected"] == [0, 1]
    assert e.state.shape == (4,) and e.landmark_ids == []
    with pytest.raises(ValueError, match="duplicate"):
        e.correct(Observation(None, np.array([0, 0]), np.array([[2, 0], [3, 0]])), 0)


def test_slam_bearing_correction_wraps_the_branch_cut():
    e = estimator()
    e.state[3] = np.pi - 0.01
    e.correct(Observation(None, np.array([0]), np.array([[2, -np.pi + 0.01]])), 0)
    before = e.state.copy()
    _, nis = e.correct(Observation(None, np.array([0]), np.array([[2, np.pi - 0.01]])), 0.2)
    assert nis[0] < 2
    assert np.linalg.norm(e.state - before) < 0.1


def test_invalid_estimated_speed_retains_a_partial_slam_log():
    config = read_json(ROOT / "configs/slam.json")
    config["initial_estimate_offset"][2] = -10
    reference = RoundedReference([[[1.5, 1], [1.525, 1], [1.55, 1]]], [0.1])
    times = np.linspace(0, 0.1, 3)
    inputs = draw_inputs({**config, "sensors": simulator_sensors(config)}, times, 7)
    log, status = simulate_slam(config, reference, times, inputs, config["scenarios"][0], "slam")
    assert status["status"] == "invalid_domain"
    assert status["completed_intervals"] == 0
    assert status["failure_time_s"] == 0
    assert log["states"].shape == (1, 4)
    assert log["applied"].shape == (0, 2)
    np.testing.assert_array_equal(log["joint_indices"], [0])


@pytest.fixture(scope="module")
def evidence(tmp_path_factory):
    root = tmp_path_factory.mktemp("slam")
    config = read_json(ROOT / "configs/slam.json")
    config["seeds"] = [7]
    write_json(root / "config.json", config)
    run_slam(root / "config.json", root / "results")
    return root / "results"


def test_slam_starts_empty_and_replays_without_truth(evidence):
    config = read_json(evidence / "config.json")
    initial = load_log(evidence / "plan.npz")["reference_states"][0]
    e = new_estimator(config, initial, "slam")
    assert e.state.shape == (4,) and e.landmark_ids == []
    log = load_log(evidence / "nominal__7__slam.npz")
    poisoned = copy.deepcopy(config)
    poisoned.pop("truth_landmarks_xy_m")
    poisoned["navigation"].pop("world")
    poisoned_log = {
        **log,
        "states": np.full_like(log["states"], 1e9),
        "disturbance": np.full_like(log["disturbance"], 1e9),
    }
    for key, value in replay_slam(poisoned_log, poisoned, "slam").items():
        np.testing.assert_array_equal(value, log[key])
    assert np.any(np.isnan(log["landmark_estimates"][0]))
    assert np.all(np.isfinite(log["landmark_estimates"][-1]))
    assert np.any(log["landmark_events"] == 3)


def test_slam_feedback_uses_only_estimated_state_and_shared_plant_noise(evidence):
    config, plan = read_json(evidence / "config.json"), load_log(evidence / "plan.npz")
    ref = RoundedReference(plan["bezier_controls"], plan["segment_durations"])
    nav = config["navigation"]
    controller = Feedback(nav["kp_per_s2"], nav["kd_per_s"], nav["min_forward_speed_mps"])
    slam = load_log(evidence / "nominal__7__slam.npz")
    known = load_log(evidence / "nominal__7__known_map.npz")
    np.testing.assert_array_equal(slam["controller_states"], slam["estimates"])
    np.testing.assert_array_equal(slam["disturbance"], known["disturbance"])
    for k in range(0, len(slam["requested"]), 41):
        expected = controller.command(
            State.from_array(slam["estimates"][k]), ref.sample(float(slam["times"][k]))
        )
        np.testing.assert_array_equal(slam["requested"][k], expected)
    for covariance in slam["joint_covariances"]:
        mask = np.isfinite(np.diag(covariance))
        p = covariance[np.ix_(mask, mask)]
        np.testing.assert_allclose(p, p.T, atol=1e-14)
        assert np.linalg.eigvalsh(p).min() > 0


def test_counterfactual_matches_before_first_denied_revisit(evidence):
    source = load_log(evidence / "nominal__7__slam.npz")
    counter = load_log(evidence / "nominal__7__no_revisit.npz")
    first = int(np.flatnonzero(np.any(source["landmark_events"] == 3, axis=1))[0])
    np.testing.assert_array_equal(source["estimates"][:first], counter["estimates"][:first])
    assert not np.array_equal(source["estimates"][first:], counter["estimates"][first:])
    assert not np.any(counter["landmark_events"] == 3)


def test_outage_retains_absence_while_speed_sensor_continues(evidence):
    log = load_log(evidence / "landmark_outage__7__slam.npz")
    active = (log["times"] >= 28) & (log["times"] < 40)
    assert not np.any(log["landmark_visible"][active])
    assert np.any(np.isfinite(log["speed_observations"][active]))


def test_slam_evidence_verifies_and_preserves_existing_results(evidence):
    manifest = verify_slam(evidence)
    assert len(manifest["runs"]) == 6 and len(manifest["replays"]) == 2
    with pytest.raises(ValueError, match="not empty"):
        run_slam(ROOT / "configs/slam.json", evidence)


def test_invalid_slam_configuration_is_rejected():
    config = read_json(ROOT / "configs/slam.json")
    config["sensors"]["landmarks_xy_m"] = config["truth_landmarks_xy_m"]
    with pytest.raises(ValueError, match="surveyed"):
        validate_slam_config(config)


def test_renderer_does_not_run_the_plant_or_planner(evidence, tmp_path, monkeypatch):
    import autonomous_explorer.slam_evidence as experiment
    import autonomous_explorer.slam_render as rendering

    def forbidden(*args, **kwargs):
        raise AssertionError("Rendering must use saved empirical evidence")

    monkeypatch.setattr(experiment, "simulate_slam", forbidden)
    monkeypatch.setattr(experiment, "plan_route", forbidden)
    monkeypatch.setattr(rendering, "slam_animation", lambda *args: None)
    rendering.render_slam(evidence, tmp_path)
    assert (tmp_path / "04_revisits.png").exists()


def test_curated_slam_provenance_and_summary():
    root, media = ROOT / "results/slam", ROOT / "media/slam"
    if not (media / "provenance.json").exists():
        pytest.skip("Generate the SLAM gallery with explorer slam-reproduce")
    provenance, manifest = read_json(media / "provenance.json"), read_json(root / "manifest.json")
    assert provenance["results_manifest_sha256"] == sha256(root / "manifest.json")
    assert provenance["input_files_sha256"] == manifest["files_sha256"]
    for name, digest in provenance["outputs_sha256"].items():
        assert sha256(media / name) == digest
    readme = (ROOT / "MILESTONE_04.md").read_text()
    table = readme.split("<!-- slam-summary:start -->\n")[1].split("<!-- slam-summary:end -->")[0]
    assert table == (root / "summary.md").read_text()
