"""Complete planning/tracking experiments, saved-data replay and honest failures."""

from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.evidence import load_log, read_json, sha256, write_json
from autonomous_explorer.navigation_evidence import (
    audit_clearance,
    run_navigation,
    verify_navigation,
)
from autonomous_explorer.path_reference import RoundedReference
from autonomous_explorer.planning import World, rover_radius
from autonomous_explorer.simulation import paired_runs

ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/navigation.json"


@pytest.fixture(scope="module")
def evidence(tmp_path_factory):
    root = tmp_path_factory.mktemp("navigation-evidence") / "results"
    run_navigation(CONFIG, root)
    return root


def test_saved_search_and_reference_certificates(evidence):
    config, summary = (
        read_json(evidence / "config.json"),
        read_json(evidence / "planning_summary.json"),
    )
    world = World.from_config(config["world"])
    radius = rover_radius(config)
    assert summary["astar"]["graph_cost_m"] == pytest.approx(summary["dijkstra"]["graph_cost_m"])
    assert summary["astar"]["expanded_nodes"] < summary["dijkstra"]["expanded_nodes"]
    assert summary["straight_connection_free"] is False
    for name in ["astar", "dijkstra"]:
        path = load_log(evidence / (name + ".npz"))["path"]
        assert all(
            world.inflate(radius + config["search_margin_m"]).segment_free(a, b)
            for a, b in zip(path[:-1], path[1:], strict=True)
        )
    plan = load_log(evidence / "plan.npz")
    certificate = read_json(evidence / "reference_certificate.json")
    assert min(plan["reference_clearance"]) >= config["reference_margin_m"]
    assert (
        certificate["bounds"]["max_abs_yaw_rate_radps"]
        <= config["reference_max_yaw_rate_radps"] + 1e-12
    )
    assert certificate["bounds"]["min_speed_mps"] > config["min_forward_speed_mps"]


def test_paired_runs_share_reference_inputs_limits_and_seeded_replay(evidence):
    config = read_json(evidence / "config.json")
    for scenario in config["scenarios"]:
        a = load_log(evidence / f"{scenario['name']}__open_loop.npz")
        b = load_log(evidence / f"{scenario['name']}__feedback.npz")
        for key in [
            "disturbance",
            "times",
            "reference_states",
            "reference_velocity",
            "reference_acceleration",
        ]:
            np.testing.assert_array_equal(a[key], b[key])
        np.testing.assert_array_equal(a["states"][0], b["states"][0])
        assert np.all(np.abs(a["applied"]) <= np.asarray(config["actuator_limits"]) + 1e-12)
        assert np.all(np.abs(b["applied"]) <= np.asarray(config["actuator_limits"]) + 1e-12)
    plan = load_log(evidence / "plan.npz")
    reference = RoundedReference(plan["bezier_controls"], plan["segment_durations"])
    repeated = paired_runs(config, reference, plan["times"], config["scenarios"][-1])
    for name, run in repeated.items():
        saved = load_log(evidence / f"seeded__{name}.npz")
        for key, value in [
            ("states", run.states),
            ("requested", run.requested),
            ("applied", run.applied),
        ]:
            np.testing.assert_array_equal(saved[key], value)


def test_failed_execution_is_retained_and_is_not_called_safe(evidence):
    outcomes = read_json(evidence / "metrics.json")
    bad = outcomes["yaw_bias__open_loop"]
    assert bad["status"] == "completed"
    assert bad["execution_status"] == "envelope_violation"
    assert bad["disk_overlap_samples"] > 0
    assert bad["center_in_obstacle_or_outside_world_samples"] > 0
    assert bad["min_disk_clearance_m"] < 0
    assert bad["first_disk_overlap_time_s"] is not None
    for case in ["nominal", "yaw_bias", "seeded"]:
        good = outcomes[f"{case}__feedback"]
        assert good["execution_status"] == "clear_within_numerical_audit"
        assert good["min_interval_clearance_bound_m"] > 0


def test_planning_failures_are_saved_without_executing_them(evidence):
    summary = read_json(evidence / "planning_summary.json")
    sealed, narrow = summary["failure_cases"]
    assert sealed["point"]["status"] == sealed["rover"]["status"] == "unreachable_on_grid"
    assert narrow["point"]["status"] == "found"
    assert narrow["rover"]["status"] == "unreachable_on_grid"
    for case in summary["failure_cases"]:
        for label in ["point", "rover"]:
            assert (evidence / case[label]["log"]).is_file()
        assert not (evidence / f"{case['name']}__feedback.npz").exists()


def test_interval_clearance_can_detect_a_missed_between_sample_collision():
    log = {
        "states": np.array([[1, 5, 4, 0], [5, 5, 4, 0]]),
        "times": np.array([0, 1]),
        "applied": np.zeros((1, 2)),
        "disturbance": np.zeros((1, 2)),
    }
    world = World((0, 10, 0, 10), ((2.9, 4, 3.1, 6),))
    clearance, lower = audit_clearance(log, world, 0.1)
    assert np.all(clearance > 0)
    assert lower[0] < 0


def test_navigation_evidence_checks_and_directory_preservation(evidence):
    assert len(verify_navigation(evidence)["runs"]) == 6
    with pytest.raises(ValueError, match="not empty"):
        run_navigation(CONFIG, evidence)


def test_primary_planning_failure_is_retained_without_rover_execution(tmp_path):
    config = read_json(CONFIG)
    config["world"] = config["failure_cases"][0]["world"]
    path, output = tmp_path / "blocked.json", tmp_path / "results"
    write_json(path, config)
    with pytest.raises(ValueError, match="no grid path"):
        run_navigation(path, output)
    failure = read_json(output / "planning_failure.json")
    assert failure["search"]["astar"]["status"] == "unreachable_on_grid"
    assert not (output / "plan.npz").exists()
    assert not list(output.glob("*feedback.npz"))


def test_navigation_renderer_does_not_run_planning_or_simulation(evidence, tmp_path, monkeypatch):
    import autonomous_explorer.navigation_evidence as experiment
    import autonomous_explorer.navigation_render as rendering
    import autonomous_explorer.planning as planning
    import autonomous_explorer.simulation as simulation

    def forbidden(*args, **kwargs):
        raise AssertionError("Rendering must only read saved evidence")

    monkeypatch.setattr(planning, "grid_search", forbidden)
    monkeypatch.setattr(simulation, "simulate", forbidden)
    monkeypatch.setattr(experiment, "run_navigation", forbidden)
    monkeypatch.setattr(rendering, "navigation_animation", lambda *args: None)
    monkeypatch.setattr(rendering, "search_animation", lambda *args: None)
    rendering.render_navigation(evidence, tmp_path / "gallery")
    assert (tmp_path / "gallery/08_clearance_audit.png").exists()


def test_curated_navigation_provenance_and_readme_table():
    root, media = ROOT / "results/navigation", ROOT / "media/navigation"
    if not (media / "provenance.json").exists():
        pytest.skip("Run explorer plan-reproduce to create the navigation gallery")
    provenance, manifest = read_json(media / "provenance.json"), verify_navigation(root)
    assert provenance["results_manifest_sha256"] == sha256(root / "manifest.json")
    assert provenance["input_files_sha256"] == manifest["files_sha256"]
    for name, digest in provenance["outputs_sha256"].items():
        assert sha256(media / name) == digest
    assert len(list(media.glob("[0-9][0-9]_*.png"))) == 11
    assert len(list(media.glob("*.gif"))) == 2
    readme = (ROOT / "README.md").read_text()
    table = readme.split("<!-- navigation-outcomes:start -->\n")[1].split(
        "<!-- navigation-outcomes:end -->"
    )[0]
    assert table == (root / "outcomes.md").read_text()
