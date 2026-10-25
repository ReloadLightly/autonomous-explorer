"""Belief-only goal selection, checked replanning, supervisor transitions and mission replay."""

import copy
from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.evidence import load_log, read_json, sha256, write_json
from autonomous_explorer.exploration_evidence import (
    new_mission,
    replay_exploration,
    resolve_config,
    run_exploration,
    verify_exploration,
)
from autonomous_explorer.mission import STATES, Belief, Mission, connect_plan
from autonomous_explorer.planning import World, rover_radius

ROOT = Path(__file__).parents[1]


def config():
    return resolve_config(ROOT / "configs/exploration.json")


def belief(position=(1.5, 1), speed=0.0, time=0.0):
    return Belief(
        np.array([*position, speed, 0.0]),
        np.diag([0.01, 0.01, 0.001, 0.001]),
        np.empty((0, 2)),
        np.empty((0, 2, 2)),
        time,
    )


def test_continuous_start_connector_and_all_path_edges_are_checked():
    world = World((0, 10, 0, 10), ((4, 2, 6, 8),))
    start, goal = [1.13, 1.07], [8, 8]
    path = connect_plan(world, start, goal, 0.5)
    assert path is not None and len(path) > 2
    np.testing.assert_array_equal(path[0], start)
    np.testing.assert_array_equal(path[-1], goal)
    assert all(world.segment_free(a, b) for a, b in zip(path[:-1], path[1:], strict=True))
    assert connect_plan(world, [5, 5], goal, 0.5) is None
    sealed = World((0, 10, 0, 10), ((4, 0, 6, 10),))
    assert connect_plan(sealed, start, goal, 0.5) is None


def test_goal_selection_changes_when_observation_coverage_changes():
    mission = new_mission(config(), "guarded")
    first = mission.select_goal(belief(), 0)[0]
    mission.covered |= mission.footprints[first]
    second = mission.select_goal(belief(), 0.2)[0]
    assert second != first
    decision = mission.decisions[-1]
    best = max(decision["candidates"], key=lambda x: (x["utility"], -x["id"]))
    assert decision["selected"] == best["id"]


def test_estimated_map_uncertainty_changes_goal_utility():
    c = config()
    c["navigation"]["world"] = {"bounds_m": [0, 10, 0, 10], "obstacles_xyxy_m": []}
    c["policy"]["viewpoints_xy_m"] = [[2, 1], [6, 1]]
    c["policy"]["map_uncertainty_weight"] = 100
    mission = Mission(c["navigation"], c["policy"], c["sensors"], [4, 1])
    mission.covered[:] = True
    prior = belief((4, 1))
    assert mission.select_goal(prior, 0)[0] == 0
    prior.landmarks = np.array([[7, 1]])
    prior.landmark_covariances = np.array([np.eye(2)])
    assert mission.select_goal(prior, 0.2)[0] == 1


def test_stale_localization_brakes_waits_and_replans_after_fresh_evidence():
    c, b = config(), belief(speed=0.4)
    mission = new_mission(c, "guarded")
    mission.install_plan(np.array([[1.5, 1], [6, 1]]), 6, 0)
    command = mission.command(b, 2.1, False)
    assert mission.state == "braking" and command[0] < 0 and command[1] == 0
    b.rover[2] = 0
    mission.command(b, 2.2, False)
    assert mission.state == "waiting"
    for time in [3.0, 3.2, 3.4]:
        b.last_correction_s = time
        mission.command(b, time, True)
    assert mission.state == "exploring"
    mission.command(b, 3.45, False)
    assert mission.plan_count == 2
    np.testing.assert_array_equal(mission.path[0], b.rover[:2])


def test_persistent_localization_loss_aborts_after_waiting():
    mission, b = new_mission(config(), "guarded"), belief()
    mission.command(b, 2.1, False)
    assert mission.state == "waiting"
    mission.command(b, 16.2, False)
    assert mission.state == "aborted" and mission.abort_reason == "localization_timeout"
    assert [e["to"] for e in mission.events] == ["braking", "waiting", "aborted"]


def test_recovery_requires_multiple_fresh_observations_and_resets_on_bad_health():
    mission, b = new_mission(config(), "guarded"), belief()
    mission.command(b, 2.1, False)
    b.last_correction_s = 2.2
    mission.command(b, 2.2, True)
    mission.command(b, 2.25, False)
    assert mission.good_observations == 1
    mission.command(b, 4.3, False)
    assert mission.good_observations == 0
    for time in [4.4, 4.6]:
        b.last_correction_s = time
        mission.command(b, time, True)
    assert mission.state == "waiting"


def test_unguarded_mode_does_not_stop_for_landmark_age():
    mission, b = new_mission(config(), "unguarded"), belief(speed=0.3)
    mission.install_plan(np.array([[1.5, 1], [6, 1]]), 6, 0)
    command = mission.command(b, 10, False)
    assert mission.state == "exploring" and command[0] > 0


def test_waypoint_controller_is_defined_at_zero_speed_and_turns_before_accelerating():
    mission, b = new_mission(config(), "guarded"), belief()
    mission.install_plan(np.array([[1.5, 1], [6, 1]]), 6, 0)
    b.rover[3] = np.pi
    turning = mission.command(b, 0, False)
    assert np.isfinite(turning).all() and turning[0] == 0 and abs(turning[1]) > 1
    b.rover[3] = 0
    forward = mission.command(b, 0.05, False)
    assert np.isfinite(forward).all() and forward[0] > 0


def test_budget_triggers_return_and_completion_requires_low_estimated_speed():
    mission, b = new_mission(config(), "guarded"), belief((6, 1))
    mission.goals_completed = mission.policy["max_exploration_goals"]
    mission.command(b, 0, False)
    assert mission.state == "returning" and mission.goal_id == -1
    b.rover[:2], b.rover[2] = mission.home, 0.2
    mission.command(b, 0.1, False)
    assert mission.state == "returning"
    b.rover[2] = 0
    mission.command(b, 0.2, False)
    assert mission.state == "complete"


def test_pose_correction_discards_old_path_before_replanning():
    mission, b = new_mission(config(), "guarded"), belief(speed=0.3)
    mission.install_plan(np.array([[1.5, 1], [6, 1]]), 6, 0)
    mission.command(b, 0.2, True, pose_jump_m=0.8)
    assert mission.state == "braking" and len(mission.path) == 0
    assert mission.events[-1]["reason"] == "pose_correction"


def test_unreachable_planning_aborts_without_a_motion_command():
    mission, b = new_mission(config(), "guarded"), belief((4.5, 4))
    command = mission.command(b, 0, False)
    assert mission.state == "aborted" and mission.abort_reason == "no_reachable_goal"
    np.testing.assert_array_equal(command, [0, 0])


@pytest.fixture(scope="module")
def evidence(tmp_path_factory):
    root = tmp_path_factory.mktemp("exploration")
    c = config()
    c["seeds"] = [7]
    write_json(root / "config.json", c)
    run_exploration(root / "config.json", root / "results")
    return root / "results"


def test_mission_replays_without_true_states_landmark_coordinates_or_schedule(evidence):
    c = read_json(evidence / "config.json")
    log = load_log(evidence / "temporary_outage__7__guarded.npz")
    poisoned = copy.deepcopy(c)
    poisoned.pop("truth_landmarks_xy_m")
    poisoned.pop("scenarios")
    poisoned.pop("yaw_bias_interval_s")
    changed = {
        **log,
        "states": np.full_like(log["states"], 1e8),
        "disturbance": np.full_like(log["disturbance"], 1e8),
    }
    replay, events, decisions = replay_exploration(changed, poisoned, "guarded")
    for key, value in replay.items():
        np.testing.assert_array_equal(value, log[key])
    metadata = read_json(evidence / "temporary_outage__7__guarded.json")
    assert events == metadata["events"] and decisions == metadata["decisions"]


def test_all_recorded_plans_start_at_the_estimate_and_have_free_segments(evidence):
    c = read_json(evidence / "config.json")
    world = World.from_config(c["navigation"]["world"]).inflate(
        rover_radius(c["navigation"]) + c["policy"]["planning_margin_m"]
    )
    log = load_log(evidence / "temporary_outage__7__guarded.npz")
    meta = read_json(evidence / "temporary_outage__7__guarded.json")
    np.testing.assert_array_equal(log["controller_states"], log["estimates"])
    for event in meta["events"]:
        if event.get("type") == "plan":
            k = int(np.argmin(abs(log["times"] - event["time_s"])))
            path = np.array(event["path"])
            np.testing.assert_array_equal(path[0], log["estimates"][k, :2])
            assert all(world.segment_free(a, b) for a, b in zip(path[:-1], path[1:], strict=True))
    assert np.all(np.diff(log["coverage"].astype(int), axis=0) >= 0)


def test_temporary_loss_recovers_and_persistent_loss_aborts_with_honest_completion(evidence):
    metrics = read_json(evidence / "metrics.json")
    temporary = metrics["temporary_outage__7__guarded"]
    assert (
        temporary["mission_status"] == "complete"
        and temporary["home_audit"]
        and temporary["clear_audit"]
    )
    assert temporary["waiting_time_s"] > 1
    persistent = metrics["persistent_outage__7__guarded"]
    assert persistent["mission_status"] == "localization_timeout" and persistent["clear_audit"]
    failed = metrics["persistent_outage__7__unguarded"]
    assert failed["mission_status"] == "complete" and not failed["home_audit"]
    assert failed["disk_overlap_samples"] > 0


def test_paired_modes_share_noise_and_match_until_health_intervention(evidence):
    a = load_log(evidence / "temporary_outage__7__guarded.npz")
    b = load_log(evidence / "temporary_outage__7__unguarded.npz")
    n = min(len(a["disturbance"]), len(b["disturbance"]))
    np.testing.assert_array_equal(a["disturbance"][:n], b["disturbance"][:n])
    first = int(np.flatnonzero(a["mission_states"] == STATES["braking"])[0])
    np.testing.assert_array_equal(a["states"][: first + 1], b["states"][: first + 1])


def test_exploration_evidence_verifies_and_existing_results_are_preserved(evidence):
    assert len(verify_exploration(evidence)["runs"]) == 6
    with pytest.raises(ValueError, match="not empty"):
        run_exploration(ROOT / "configs/exploration.json", evidence)


def test_invalid_viewpoints_are_rejected(tmp_path):
    c = config()
    c["policy"]["viewpoints_xy_m"] = [[4.5, 4]]
    write_json(tmp_path / "config.json", c)
    with pytest.raises(ValueError, match="free"):
        resolve_config(tmp_path / "config.json")


def test_renderer_uses_saved_mission_evidence(evidence, tmp_path, monkeypatch):
    import autonomous_explorer.exploration_evidence as experiment
    import autonomous_explorer.exploration_render as rendering

    def forbidden(*args, **kwargs):
        raise AssertionError("Rendering cannot run a physical exploration experiment")

    monkeypatch.setattr(experiment, "simulate_exploration", forbidden)
    monkeypatch.setattr(rendering, "exploration_animation", lambda *args: None)
    rendering.render_exploration(evidence, tmp_path)
    assert (tmp_path / "04_clearance.png").exists()


def test_curated_exploration_provenance_and_summary():
    root, media = ROOT / "results/exploration", ROOT / "media/exploration"
    if not (media / "provenance.json").exists():
        pytest.skip("Generate the exploration gallery with explorer explore-reproduce")
    provenance, manifest = read_json(media / "provenance.json"), read_json(root / "manifest.json")
    assert provenance["results_manifest_sha256"] == sha256(root / "manifest.json")
    assert provenance["input_files_sha256"] == manifest["files_sha256"]
    for name, digest in provenance["outputs_sha256"].items():
        assert sha256(media / name) == digest
    table = (
        (ROOT / "README.md")
        .read_text()
        .split("<!-- exploration-summary:start -->\n")[1]
        .split("<!-- exploration-summary:end -->")[0]
    )
    assert table == (root / "summary.md").read_text()
