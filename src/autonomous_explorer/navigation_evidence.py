"""Milestone 02: plan, certify a reference, run the original rover, save and audit."""

import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .evidence import load_log, metrics, read_json, save_run, sha256, write_json
from .path_reference import build_rounded_reference
from .planning import World, grid_search, rover_radius, shortcut_path
from .simulation import paired_runs, time_grid


def save_search(path, result):
    np.savez_compressed(
        path,
        path=result.path,
        expanded=result.expanded,
        occupancy=result.occupancy,
        xs=result.xs,
        ys=result.ys,
    )


def audit_clearance(log, world, radius):
    """Sampled disk clearance plus a conservative per-interval travel bound.

    At most max(|v_start+d_v|, |v_end+d_v|)*dt/2 motion separates any
    continuous-time point from its nearer interval endpoint. Signed clearance
    is 1-Lipschitz. The bound is conditioned on the numerical endpoints; it
    does not include an independently proven global RK4 error bound.
    """
    states, delta = log["states"], np.diff(log["times"])
    clearance = world.clearance(states[:, :2], radius)
    offset = log["disturbance"][:, 0]
    end_speed = states[:-1, 2] + log["applied"][:, 0] * delta
    speed_bound = np.maximum(np.abs(states[:-1, 2] + offset), np.abs(end_speed + offset))
    lower = np.minimum(clearance[:-1], clearance[1:]) - speed_bound * delta / 2
    return clearance, lower


def navigation_metrics(log, status, config):
    result = metrics(log, status, config["min_forward_speed_mps"])
    radius, world = rover_radius(config), World.from_config(config["world"])
    clearance, lower = audit_clearance(log, world, radius)
    overlap = clearance <= 0
    result.update(
        min_disk_clearance_m=float(np.min(clearance)),
        min_interval_clearance_bound_m=float(np.min(lower)) if len(lower) else None,
        disk_overlap_samples=int(np.count_nonzero(overlap)),
        center_in_obstacle_or_outside_world_samples=int(
            np.count_nonzero(world.clearance(log["states"][:, :2]) <= 0)
        ),
        first_disk_overlap_time_s=float(log["times"][np.flatnonzero(overlap)[0]])
        if np.any(overlap)
        else None,
        execution_status=(
            "simulation_failed"
            if status["status"] != "completed"
            else "envelope_violation"
            if np.any(overlap)
            else "clearance_uncertified"
            if len(lower) and np.min(lower) <= 0
            else "clear_within_numerical_audit"
        ),
        actual_path_length_m=float(
            np.sum(np.linalg.norm(np.diff(log["states"][:, :2], axis=0), axis=1))
        ),
    )
    return result


def navigation_table(outcomes):
    rows = [
        "| Case | Controller | RMS error [m] | Final error [m] | Min disk clearance [m] | Envelope overlaps |",
        "| :-- | :-- | --: | --: | --: | --: |",
    ]
    for name, m in outcomes.items():
        case, controller = name.split("__")
        final = "failed" if m["final_position_m"] is None else f"{m['final_position_m']:.4f}"
        rows.append(
            f"| {case.replace('_', ' ')} | {controller.replace('_', ' ')} | "
            f"{m['rms_position_m']:.4f} | {final} | {m['min_disk_clearance_m']:.4f} | {m['disk_overlap_samples']} |"
        )
    return "\n".join(rows) + "\n"


def run_navigation(config_path, output):
    config, root = read_json(config_path), Path(output)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"Results directory is not empty: {root}")
    root.mkdir(parents=True, exist_ok=True)
    if config["search_margin_m"] < config["reference_margin_m"] or config["reference_margin_m"] < 0:
        raise ValueError("Require search_margin >= reference_margin >= 0")
    radius, world = rover_radius(config), World.from_config(config["world"])
    search_world = world.inflate(radius + config["search_margin_m"])
    searches, summary = {}, {}
    for algorithm in ["astar", "dijkstra"]:
        search = grid_search(
            search_world,
            config["start_xy_m"],
            config["goal_xy_m"],
            config["grid_resolution_m"],
            algorithm,
        )
        searches[algorithm] = search
        save_search(root / (algorithm + ".npz"), search)
        summary[algorithm] = {
            "status": search.status,
            "expanded_nodes": len(search.expanded),
            "graph_cost_m": search.cost,
            "path_nodes": len(search.path),
        }
    if searches["astar"].status != "found":
        write_json(root / "planning_failure.json", {"configuration": config, "search": summary})
        raise ValueError("Primary map has no grid path; saved planning failure, no rover execution")
    if not np.isclose(searches["astar"].cost, searches["dijkstra"].cost, atol=1e-10):
        raise ValueError("A* and Dijkstra disagree on optimal graph cost")
    shortcut = shortcut_path(searches["astar"].path, search_world)
    reference, certificate = build_rounded_reference(
        shortcut,
        world.inflate(radius + config["reference_margin_m"]),
        config["cruise_speed_mps"],
        config["reference_max_acceleration_mps2"],
        config["reference_max_yaw_rate_radps"],
        config["dt_s"],
    )
    if certificate["bounds"]["min_speed_mps"] <= config["min_forward_speed_mps"]:
        raise ValueError("Reference violates the controller's forward-speed domain")
    config["initial_state"] = reference.sample(0).state.array().tolist()
    config["final_state"] = reference.sample(reference.duration).state.array().tolist()
    config["duration_s"] = reference.duration
    for scenario in config["scenarios"]:
        if scenario["kind"] == "yaw_bias":
            scenario["start_s"] = (
                round(reference.duration * scenario["start_fraction"] / config["dt_s"])
                * config["dt_s"]
            )
            scenario["end_s"] = min(
                reference.duration, scenario["start_s"] + scenario["duration_s"]
            )
    write_json(root / "config.json", config)
    times = time_grid(reference.duration, config["dt_s"])
    ref, vel, acc = reference.arrays(times)
    np.savez_compressed(
        root / "plan.npz",
        shortcut=shortcut,
        bezier_controls=reference.controls,
        segment_durations=reference.durations,
        segment_boundaries=reference.boundaries,
        times=times,
        reference_states=ref,
        reference_velocity=vel,
        reference_acceleration=acc,
        reference_clearance=world.clearance(ref[:, :2], radius),
    )
    write_json(root / "reference_certificate.json", certificate)
    summary["shortcut_length_m"] = float(np.sum(np.linalg.norm(np.diff(shortcut, axis=0), axis=1)))
    summary["reference_sampled_length_m"] = float(
        np.sum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))
    )
    summary["straight_connection_free"] = world.inflate(radius).segment_free(
        config["start_xy_m"], config["goal_xy_m"]
    )
    summary["rover_enclosing_radius_m"] = radius
    summary["resolution_study"] = []
    for resolution in config["resolution_study_m"]:
        s = grid_search(search_world, config["start_xy_m"], config["goal_xy_m"], resolution)
        name = "resolution_" + str(resolution).replace(".", "_")
        save_search(root / (name + ".npz"), s)
        summary["resolution_study"].append(
            {
                "resolution_m": resolution,
                "status": s.status,
                "expanded_nodes": len(s.expanded),
                "graph_cost_m": s.cost,
                "log": name + ".npz",
            }
        )
    summary["failure_cases"] = []
    for failure in config["failure_cases"]:
        failure_world = World.from_config(failure["world"])
        entry = {"name": failure["name"], "configuration": failure}
        for label, margin in [("point", 0.0), ("rover", radius + config["reference_margin_m"])]:
            s = grid_search(
                failure_world.inflate(margin),
                failure["start_xy_m"],
                failure["goal_xy_m"],
                config["grid_resolution_m"],
            )
            name = failure["name"] + "__" + label
            save_search(root / (name + ".npz"), s)
            entry[label] = {
                "status": s.status,
                "expanded_nodes": len(s.expanded),
                "log": name + ".npz",
            }
        summary["failure_cases"].append(entry)
    write_json(root / "planning_summary.json", summary)
    entries, outcomes = [], {}
    for scenario in config["scenarios"]:
        for controller, run in paired_runs(config, reference, times, scenario).items():
            name = scenario["name"] + "__" + controller
            save_run(root / (name + ".npz"), run, reference)
            metadata = {
                "scenario": scenario,
                "controller": controller,
                "status": run.status,
                "note": "Collision audit is post hoc; it does not stop or replan the controller",
            }
            write_json(root / (name + ".json"), metadata)
            log = load_log(root / (name + ".npz"))
            clearance, lower = audit_clearance(log, world, radius)
            np.savez_compressed(
                root / (name + "__clearance.npz"),
                sampled_clearance_m=clearance,
                interval_lower_bound_m=lower,
            )
            outcomes[name] = navigation_metrics(log, run.status, config)
            entries.append(
                {
                    "name": name,
                    "log": name + ".npz",
                    "metadata": name + ".json",
                    "clearance": name + "__clearance.npz",
                }
            )
    write_json(root / "metrics.json", outcomes)
    (root / "outcomes.md").write_text(navigation_table(outcomes))
    manifest = {
        "schema_version": 1,
        "milestone": 2,
        "name": config["name"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "upstream_revision": "02ee1421786b73ae4810b086cd1f59882ab972bc",
        "source_sha256": {
            p.name: sha256(p)
            for p in sorted(Path(__file__).parent.glob("*.py"))
            if p.name not in {"render.py", "navigation_render.py", "theme.py", "cli.py"}
        },
        "runs": entries,
        "files_sha256": {p.name: sha256(p) for p in sorted(root.iterdir()) if p.is_file()},
    }
    write_json(root / "manifest.json", manifest)
    return outcomes


def verify_navigation(root):
    root = Path(root)
    config, manifest = read_json(root / "config.json"), read_json(root / "manifest.json")
    for name, expected in manifest["files_sha256"].items():
        if sha256(root / name) != expected:
            raise ValueError(f"Navigation evidence checksum mismatch: {name}")
    outcomes = read_json(root / "metrics.json")
    for entry in manifest["runs"]:
        log = load_log(root / entry["log"])
        actual = navigation_metrics(log, read_json(root / entry["metadata"])["status"], config)
        if actual != outcomes[entry["name"]]:
            raise ValueError(f"Navigation metrics mismatch: {entry['name']}")
        saved_clearance = load_log(root / entry["clearance"])
        calculated = audit_clearance(log, World.from_config(config["world"]), rover_radius(config))
        for key, value in zip(
            ["sampled_clearance_m", "interval_lower_bound_m"], calculated, strict=True
        ):
            if not np.array_equal(saved_clearance[key], value):
                raise ValueError(f"Clearance mismatch: {entry['name']}")
    return manifest
