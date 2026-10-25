"""Milestone 05 experiments: causal decisions, SLAM, controls, and mission evidence."""

import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .evidence import load_log, read_json, sha256, write_json
from .localization_evidence import draw_inputs, validate_config
from .mission import STATES, Belief, Mission
from .model import step, wrap_angle
from .navigation_evidence import audit_clearance
from .planning import World, rover_radius
from .sensors import Observation, sample_observation
from .simulation import time_grid
from .slam_evidence import (
    append_snapshot,
    correct_estimator,
    empty_history,
    estimator_snapshot,
    map_metrics,
    new_estimator,
    simulator_sensors,
)

MODES = ("unguarded", "guarded")


def resolve_config(path):
    path = Path(path)
    config = read_json(path)
    if "navigation" not in config:
        base = read_json(path.parent / config["slam_config"])
        config = {**base, **config}
        config["navigation"]["grid_resolution_m"] = config["planning_grid_resolution_m"]
    validate_config({**config, "sensors": simulator_sensors(config)})
    for key in ["evaluation_home_tolerance_m", "evaluation_stopped_speed_mps"]:
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    p = config["policy"]
    initial = np.asarray(config["initial_state"], dtype=float)
    viewpoints = np.asarray(p["viewpoints_xy_m"], dtype=float)
    if initial.shape != (4,) or not np.all(np.isfinite(initial)):
        raise ValueError("Require a finite four-state initial rover prior")
    if (
        viewpoints.ndim != 2
        or viewpoints.shape[1] != 2
        or len(viewpoints) == 0
        or not np.all(np.isfinite(viewpoints))
        or len(np.unique(viewpoints, axis=0)) != len(viewpoints)
    ):
        raise ValueError("Require distinct finite 2D viewpoints")
    for key in ["distance_weight_per_m", "map_uncertainty_weight"]:
        if not np.isfinite(p[key]) or p[key] < 0:
            raise ValueError(f"{key} must be nonnegative and finite")
    if p["coverage_radius_m"] > config["sensors"]["max_range_m"]:
        raise ValueError("Coverage radius must not exceed sensor range")
    for key in [
        "coverage_resolution_m",
        "coverage_radius_m",
        "planning_margin_m",
        "cruise_speed_mps",
        "arrival_tolerance_m",
        "stopped_speed_mps",
        "speed_gain_per_s",
        "distance_gain_per_s",
        "heading_gain_per_s",
        "turn_threshold_rad",
        "max_landmark_age_s",
        "max_position_std_m",
        "replan_jump_m",
        "localization_timeout_s",
    ]:
        if not np.isfinite(p[key]) or p[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    for key in ["max_exploration_goals", "recovery_observations"]:
        if type(p[key]) is not int or p[key] <= 0:
            raise ValueError(f"{key} must be a positive integer")
    if not 0 < p["coverage_target_fraction"] <= 1:
        raise ValueError("Coverage target must be in (0, 1]")
    world = World.from_config(config["navigation"]["world"]).inflate(
        rover_radius(config["navigation"]) + p["planning_margin_m"]
    )
    for point in [config["initial_state"][:2], *p["viewpoints_xy_m"]]:
        if not world.point_free(point):
            raise ValueError("Home and viewpoints must be free in the supplied planning world")
        if not np.allclose(
            np.asarray(point) / config["navigation"]["grid_resolution_m"],
            np.rint(np.asarray(point) / config["navigation"]["grid_resolution_m"]),
        ):
            raise ValueError("Configured home/viewpoints must lie on the declared grid")
    time_grid(config["duration_s"], config["navigation"]["dt_s"])
    return config


def belief_from_filter(estimator, last_correction):
    covariances = np.array(
        [
            estimator.covariance[4 + 2 * j : 6 + 2 * j, 4 + 2 * j : 6 + 2 * j]
            for j in range(len(estimator.landmark_ids))
        ]
    ).reshape(-1, 2, 2)
    return Belief(
        estimator.state[:4].copy(),
        estimator.covariance[:4, :4].copy(),
        estimator.state[4:].reshape(-1, 2).copy(),
        covariances,
        last_correction,
    )


def new_mission(config, mode):
    if mode not in MODES:
        raise ValueError(f"Unknown mission mode: {mode}")
    return Mission(
        config["navigation"],
        config["policy"],
        config["sensors"],
        config["initial_state"][:2],
        guarded=mode == "guarded",
    )


def simulate_exploration(config, times, inputs, scenario, mode):
    nav, sensors = config["navigation"], simulator_sensors(config)
    world, count = World.from_config(nav["world"]), len(sensors["landmarks_xy_m"])
    estimator = new_estimator(config, config["initial_state"], "slam")
    mission = new_mission(config, mode)
    disturbance = inputs["disturbance"].copy()
    start, end = config["yaw_bias_interval_s"]
    disturbance[(times[:-1] >= start) & (times[:-1] < end), 1] += scenario["yaw_bias_radps"]
    history, states = empty_history(), [np.asarray(config["initial_state"], dtype=float)]
    modes, goals, covered, ages, jumps = [], [], [], [], []
    speeds, readings, masks, event_codes, innovations = [], [], [], [], []
    requested, applied, saturated = [], [], []
    last_correction, failure = 0.0, None
    for k, time in enumerate(times):
        packet = sample_observation(
            states[-1],
            k,
            time,
            sensors,
            world,
            inputs["speed_noise"],
            inputs["landmark_noise"],
            scenario["landmark_outage_s"],
        )
        prior = estimator.state[:2].copy()
        codes, nis = correct_estimator(estimator, packet, time, "slam", count)
        fresh = bool(np.any((codes >= 1) & (codes <= 3)))
        if fresh:
            last_correction = float(time)
        jump = float(np.linalg.norm(estimator.state[:2] - prior))
        command = mission.command(
            belief_from_filter(estimator, last_correction), float(time), fresh, jump
        )
        append_snapshot(
            history, estimator, count, k, sensors["landmark_every_steps"], k == len(times) - 1
        )
        modes.append(STATES[mission.state])
        goals.append(-2 if mission.goal_id is None else mission.goal_id)
        covered.append(mission.covered.copy())
        ages.append(float(time - last_correction))
        jumps.append(jump)
        values, mask = np.full((count, 2), np.nan), np.zeros(count, dtype=bool)
        values[packet.landmark_ids], mask[packet.landmark_ids] = packet.range_bearing, True
        speeds.append(np.nan if packet.speed is None else packet.speed)
        readings.append(values)
        masks.append(mask)
        event_codes.append(codes)
        innovations.append(nis)
        if mission.state in {"complete", "aborted"} or k == len(times) - 1:
            break
        used = np.clip(command, -np.asarray(nav["actuator_limits"]), nav["actuator_limits"])
        dt = times[k + 1] - time
        candidate = step(states[-1], used, dt, disturbance[k])
        if not np.all(np.isfinite(candidate)):
            failure = "nonfinite_plant"
            break
        requested.append(command)
        applied.append(used)
        saturated.append(np.abs(command - used) > 1e-12)
        states.append(candidate)
        estimator.predict(used, dt)
    n = len(states)
    if history["joint_indices"][-1] != n - 1:
        snapshot = estimator_snapshot(estimator, count)
        history["joint_indices"].append(n - 1)
        for key in ["joint_states", "joint_covariances"]:
            history[key].append(snapshot[key])
    log = {key: np.array(value) for key, value in history.items()}
    log.update(
        times=times[:n],
        states=np.array(states),
        controller_states=log["estimates"].copy(),
        requested=np.array(requested).reshape(-1, 2),
        applied=np.array(applied).reshape(-1, 2),
        saturated=np.array(saturated, dtype=bool).reshape(-1, 2),
        disturbance=disturbance[: n - 1],
        speed_observations=np.array(speeds),
        landmark_observations=np.array(readings),
        landmark_visible=np.array(masks),
        landmark_events=np.array(event_codes),
        landmark_nis=np.array(innovations),
        mission_states=np.array(modes, dtype=int),
        goal_ids=np.array(goals, dtype=int),
        coverage=np.array(covered),
        coverage_cells=mission.cells,
        landmark_age_s=np.array(ages),
        pose_jump_m=np.array(jumps),
    )
    clearance, lower = audit_clearance(log, world, rover_radius(nav))
    log["sampled_clearance_m"], log["interval_clearance_bound_m"] = clearance, lower
    status = {
        "mission_status": failure
        or (
            "complete"
            if mission.state == "complete"
            else mission.abort_reason
            if mission.state == "aborted"
            else "time_limit"
        ),
        "last_state": mission.state,
        "observed_horizon_s": float(times[n - 1]),
        "completed_intervals": n - 1,
        "planned_intervals": len(times) - 1,
        "goals_completed": mission.goals_completed,
        "plans": mission.plan_count,
    }
    return log, status, mission.events, mission.decisions


def replay_exploration(log, config, mode):
    """Reconstruct SLAM and mission solely from saved packets and applied commands."""
    estimator = new_estimator(config, config["initial_state"], "slam")
    mission, last_correction = new_mission(config, mode), 0.0
    history = empty_history()
    actions, modes, goals, coverage, codes_history, nis_history, ages, jumps = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    count = log["landmark_visible"].shape[1]
    for k, time in enumerate(log["times"]):
        ids = np.flatnonzero(log["landmark_visible"][k])
        speed = log["speed_observations"][k]
        packet = Observation(
            None if np.isnan(speed) else speed, ids, log["landmark_observations"][k, ids]
        )
        prior = estimator.state[:2].copy()
        codes, nis = correct_estimator(estimator, packet, time, "slam", count)
        fresh = bool(np.any((codes >= 1) & (codes <= 3)))
        if fresh:
            last_correction = float(time)
        jump = float(np.linalg.norm(estimator.state[:2] - prior))
        command = mission.command(
            belief_from_filter(estimator, last_correction), float(time), fresh, jump
        )
        append_snapshot(
            history,
            estimator,
            count,
            k,
            config["sensors"]["landmark_every_steps"],
            k == len(log["times"]) - 1,
        )
        modes.append(STATES[mission.state])
        goals.append(-2 if mission.goal_id is None else mission.goal_id)
        coverage.append(mission.covered.copy())
        codes_history.append(codes)
        nis_history.append(nis)
        ages.append(float(time - last_correction))
        jumps.append(jump)
        if k < len(log["applied"]):
            actions.append(command)
            estimator.predict(log["applied"][k], log["times"][k + 1] - time)
    result = {key: np.array(value) for key, value in history.items()}
    result.update(
        requested=np.array(actions).reshape(-1, 2),
        mission_states=np.array(modes),
        goal_ids=np.array(goals),
        coverage=np.array(coverage),
        landmark_events=np.array(codes_history),
        landmark_nis=np.array(nis_history),
        landmark_age_s=np.array(ages),
        pose_jump_m=np.array(jumps),
    )
    return result, mission.events, mission.decisions


def exploration_metrics(log, metadata, config):
    error = log["estimates"] - log["states"]
    error[:, 3] = wrap_angle(error[:, 3])
    clearance = log["sampled_clearance_m"]
    overlap = clearance <= 0
    home_error = float(np.linalg.norm(log["states"][-1, :2] - config["initial_state"][:2]))
    home_audit = (
        metadata["status"]["mission_status"] == "complete"
        and home_error <= config["evaluation_home_tolerance_m"]
        and abs(log["states"][-1, 2]) <= config["evaluation_stopped_speed_mps"]
    )
    return {
        **map_metrics(log, config),
        **metadata["status"],
        "rms_localization_m": float(np.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1)))),
        "estimated_coverage_fraction": float(log["coverage"][-1].mean()),
        "min_disk_clearance_m": float(clearance.min()),
        "min_interval_clearance_bound_m": float(log["interval_clearance_bound_m"].min())
        if len(log["applied"])
        else None,
        "disk_overlap_samples": int(overlap.sum()),
        "actual_home_error_m": home_error,
        "home_audit": bool(home_audit),
        "actual_final_speed_mps": float(log["states"][-1, 2]),
        "estimated_final_speed_mps": float(log["estimates"][-1, 2]),
        "path_length_m": float(np.linalg.norm(np.diff(log["states"][:, :2], axis=0), axis=1).sum()),
        "waiting_time_s": float(
            np.diff(log["times"])[log["mission_states"][:-1] == STATES["waiting"]].sum()
        ),
        "saturated_intervals": int(np.any(log["saturated"], axis=1).sum()),
        "clear_audit": bool(
            not np.any(overlap)
            and (not len(log["applied"]) or log["interval_clearance_bound_m"].min() > 0)
        ),
    }


def summary_table(outcomes, config):
    rows = [
        "| Case | Supervisor | Reports complete | Home audit | Aborted | Clear audit | Mean model coverage |",
        "| :-- | :-- | --: | --: | --: | --: | --: |",
    ]
    for scenario in config["scenarios"]:
        for mode in MODES:
            runs = [outcomes[f"{scenario['name']}__{seed}__{mode}"] for seed in config["seeds"]]
            complete = sum(m["mission_status"] == "complete" for m in runs)
            aborted = sum(m["last_state"] == "aborted" for m in runs)
            clear = sum(m["clear_audit"] for m in runs)
            coverage = np.mean([m["estimated_coverage_fraction"] for m in runs])
            home = sum(m["home_audit"] for m in runs)
            rows.append(
                f"| {scenario['name'].replace('_', ' ')} | {mode} | {complete}/{len(runs)} | "
                f"{home}/{len(runs)} | {aborted}/{len(runs)} | {clear}/{len(runs)} | {coverage:.1%} |"
            )
    return "\n".join(rows) + "\n"


def run_exploration(config_path, output):
    root = Path(output)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"Results directory is not empty: {root}")
    config = resolve_config(config_path)
    times = time_grid(config["duration_s"], config["navigation"]["dt_s"])
    root.mkdir(parents=True, exist_ok=True)
    write_json(root / "config.json", config)
    entries, outcomes = [], {}
    for seed in config["seeds"]:
        inputs = draw_inputs({**config, "sensors": simulator_sensors(config)}, times, seed)
        np.savez_compressed(root / f"seed_{seed}_inputs.npz", **inputs)
        for scenario in config["scenarios"]:
            for mode in MODES:
                name = f"{scenario['name']}__{seed}__{mode}"
                log, status, events, decisions = simulate_exploration(
                    config, times, inputs, scenario, mode
                )
                np.savez_compressed(root / (name + ".npz"), **log)
                metadata = {
                    "seed": seed,
                    "scenario": scenario,
                    "mode": mode,
                    "status": status,
                    "events": events,
                    "decisions": decisions,
                }
                write_json(root / (name + ".json"), metadata)
                outcomes[name] = exploration_metrics(
                    load_log(root / (name + ".npz")), metadata, config
                )
                entries.append({"name": name, "log": name + ".npz", "metadata": name + ".json"})
    write_json(root / "metrics.json", outcomes)
    (root / "summary.md").write_text(summary_table(outcomes, config))
    write_json(
        root / "manifest.json",
        {
            "schema_version": 1,
            "milestone": 5,
            "name": config["name"],
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "state_codes": STATES,
            "coverage": "modelled visibility footprint from estimated poses with accepted landmark packets",
            "control": "waypoint heading/speed regulation defined at zero; clipped held (acceleration,yaw rate)",
            "runs": entries,
            "source_sha256": {
                p.name: sha256(p)
                for p in sorted(Path(__file__).parent.glob("*.py"))
                if not p.name.endswith("render.py") and p.name not in {"theme.py", "cli.py"}
            },
            "files_sha256": {p.name: sha256(p) for p in sorted(root.iterdir()) if p.is_file()},
        },
    )
    return outcomes


def verify_exploration(root):
    root = Path(root)
    config, manifest = read_json(root / "config.json"), read_json(root / "manifest.json")
    for name, digest in manifest["files_sha256"].items():
        if sha256(root / name) != digest:
            raise ValueError(f"Exploration checksum mismatch: {name}")
    outcomes = read_json(root / "metrics.json")
    if (root / "summary.md").read_text() != summary_table(outcomes, config):
        raise ValueError("Exploration summary mismatch")
    for entry in manifest["runs"]:
        log, metadata = load_log(root / entry["log"]), read_json(root / entry["metadata"])
        replay, events, decisions = replay_exploration(log, config, metadata["mode"])
        for key, value in replay.items():
            if not np.array_equal(log[key], value, equal_nan=True):
                raise ValueError(f"Exploration replay mismatch: {entry['name']}/{key}")
        if events != metadata["events"] or decisions != metadata["decisions"]:
            raise ValueError(f"Exploration decision history mismatch: {entry['name']}")
        calculated = audit_clearance(
            log,
            World.from_config(config["navigation"]["world"]),
            rover_radius(config["navigation"]),
        )
        for key, value in zip(
            ["sampled_clearance_m", "interval_clearance_bound_m"], calculated, strict=True
        ):
            if not np.array_equal(log[key], value):
                raise ValueError(f"Exploration clearance mismatch: {entry['name']}")
        if exploration_metrics(log, metadata, config) != outcomes[entry["name"]]:
            raise ValueError(f"Exploration metrics mismatch: {entry['name']}")
        if not np.array_equal(log["controller_states"], log["estimates"]):
            raise ValueError(f"Exploration controller state mismatch: {entry['name']}")
    return manifest
