"""Milestone 04: build an unknown landmark map, drive with SLAM, and replay revisits."""

import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .control import Feedback, InvalidSpeed
from .evidence import load_log, read_json, sha256, write_json
from .localization import ExtendedKalmanFilter
from .localization_evidence import draw_inputs, localization_metrics, validate_config
from .model import State, step
from .navigation_evidence import audit_clearance
from .path_reference import build_rounded_reference
from .planning import World, grid_search, rover_radius, shortcut_path
from .sensors import Observation, sample_observation
from .simulation import time_grid
from .slam import EKFSLAM

MODES = ("known_map", "first_sighting", "slam")
# 0 absent, 1 initialized, 2 corrected, 3 corrected after gap, 4 rejected, 5 skipped.
EVENT_CODES = {"new": 1, "updated": 2, "revisited": 3, "rejected": 4, "skipped": 5}


def simulator_sensors(config):
    """Coordinates are available only to synthesis, evaluation, and the labelled baseline."""
    return {**config["sensors"], "landmarks_xy_m": config["truth_landmarks_xy_m"]}


def validate_slam_config(config):
    validate_config({**config, "sensors": simulator_sensors(config)})
    route = np.asarray(config["route_xy_m"], dtype=float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 3 or not np.all(np.isfinite(route)):
        raise ValueError("Require a finite route with at least three 2D checkpoints")
    if not np.array_equal(route[0], route[-1]):
        raise ValueError("The SLAM revisit experiment requires a closed positional route")
    if not np.isfinite(config["revisit_gap_s"]) or config["revisit_gap_s"] <= 0:
        raise ValueError("Revisit gap must be positive and finite")
    nav = config["navigation"]
    if not nav["search_margin_m"] >= nav["reference_margin_m"] >= 0:
        raise ValueError("Require search margin >= reference margin >= 0")
    if "landmarks_xy_m" in config["sensors"]:
        raise ValueError("SLAM sensor configuration may not contain a surveyed map")
    if not np.array_equal(route[0], nav["start_xy_m"]) or not np.array_equal(
        route[-1], nav["goal_xy_m"]
    ):
        raise ValueError("Route endpoints must match navigation start/goal")


def new_estimator(config, initial, mode):
    initial = np.asarray(initial) + config["initial_estimate_offset"]
    covariance = np.diag(np.asarray(config["initial_std"]) ** 2)
    if mode == "known_map":
        return ExtendedKalmanFilter(
            initial, covariance, config["process_held_std"], simulator_sensors(config)
        )
    if mode not in {"slam", "first_sighting", "no_revisit"}:
        raise ValueError(f"Unknown SLAM mode: {mode}")
    sensors = config["sensors"]
    return EKFSLAM(
        initial,
        covariance,
        config["process_held_std"],
        [sensors["speed_std_mps"], sensors["range_std_m"], sensors["bearing_std_rad"]],
        config["revisit_gap_s"],
    )


def correct_estimator(estimator, packet, time, mode, count):
    events, nis = np.zeros(count, dtype=np.uint8), np.full(count, np.nan)
    if mode == "known_map":
        _, values = estimator.correct(packet)
        events[packet.landmark_ids] = 2
        nis[packet.landmark_ids] = values
    else:
        recorded, values = estimator.correct(
            packet,
            time,
            use_reobservations=mode != "first_sighting",
            allow_revisits=mode != "no_revisit",
        )
        for name, code in EVENT_CODES.items():
            events[recorded[name]] = code
        for landmark_id, value in values.items():
            nis[landmark_id] = value
    return events, nis


def estimator_snapshot(estimator, count):
    """Pad only the logger's canonical ID order; the actual filter grows dynamically."""
    points = np.full((count, 2), np.nan)
    joint = np.full(4 + 2 * count, np.nan)
    covariance = np.full((len(joint), len(joint)), np.nan)
    joint[:4] = estimator.state[:4]
    if isinstance(estimator, EKFSLAM):
        canonical = list(range(4))
        for slot, landmark_id in enumerate(estimator.landmark_ids):
            points[landmark_id] = estimator.state[4 + 2 * slot : 6 + 2 * slot]
            canonical.extend([4 + 2 * landmark_id, 5 + 2 * landmark_id])
        joint[canonical] = estimator.state
        covariance[np.ix_(canonical, canonical)] = estimator.covariance
    else:
        # A known-map baseline has no uncertain map in its filter state.
        covariance[:4, :4] = estimator.covariance
    return {
        "estimates": estimator.state[:4].copy(),
        "covariances": estimator.covariance[:4, :4].copy(),
        "landmark_estimates": points,
        "joint_states": joint,
        "joint_covariances": covariance,
    }


def append_snapshot(history, estimator, count, index, stride, final=False):
    snapshot = estimator_snapshot(estimator, count)
    for key in ["estimates", "covariances", "landmark_estimates"]:
        history[key].append(snapshot[key])
    if index % stride == 0 or final:
        history["joint_indices"].append(index)
        for key in ["joint_states", "joint_covariances"]:
            history[key].append(snapshot[key])


def empty_history():
    return {
        key: []
        for key in [
            "estimates",
            "covariances",
            "landmark_estimates",
            "joint_indices",
            "joint_states",
            "joint_covariances",
        ]
    }


def simulate_slam(config, reference, times, inputs, scenario, mode):
    nav, sensors = config["navigation"], simulator_sensors(config)
    world, count = World.from_config(nav["world"]), len(sensors["landmarks_xy_m"])
    initial = reference.sample(0).state.array()
    estimator = new_estimator(config, initial, mode)
    controller = Feedback(nav["kp_per_s2"], nav["kd_per_s"], nav["min_forward_speed_mps"])
    disturbance = inputs["disturbance"].copy()
    start, end = config["yaw_bias_interval_s"]
    disturbance[(times[:-1] >= start) & (times[:-1] < end), 1] += scenario["yaw_bias_radps"]
    history = empty_history()
    states, requested, applied, saturated = [initial], [], [], []
    speeds, readings, visible, codes, innovations, priors = [], [], [], [], [], []
    status = {
        "status": "completed",
        "failure_time_s": None,
        "message": None,
        "invalid_domain_events": 0,
        "planned_intervals": len(times) - 1,
    }
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
        priors.append(estimator.state[:4].copy())
        events, nis = correct_estimator(estimator, packet, time, mode, count)
        append_snapshot(
            history, estimator, count, k, sensors["landmark_every_steps"], k == len(times) - 1
        )
        values, mask = np.full((count, 2), np.nan), np.zeros(count, dtype=bool)
        values[packet.landmark_ids], mask[packet.landmark_ids] = packet.range_bearing, True
        speeds.append(np.nan if packet.speed is None else packet.speed)
        readings.append(values)
        visible.append(mask)
        codes.append(events)
        innovations.append(nis)
        if k == len(times) - 1:
            break
        try:
            if states[-1][2] <= nav["min_forward_speed_mps"]:
                raise InvalidSpeed("Physical state left the forward-speed domain")
            u = controller.command(
                State.from_array(estimator.state[:4]), reference.sample(float(time))
            )
        except InvalidSpeed as exc:
            status.update(
                status="invalid_domain",
                failure_time_s=float(time),
                message=str(exc),
                invalid_domain_events=1,
            )
            break
        used = np.clip(u, -np.asarray(nav["actuator_limits"]), nav["actuator_limits"])
        dt = times[k + 1] - time
        candidate = step(states[-1], used, dt, disturbance[k])
        if not np.all(np.isfinite(candidate)):
            status.update(
                status="nonfinite", failure_time_s=float(time), message="Nonfinite candidate state"
            )
            break
        states.append(candidate)
        requested.append(u)
        applied.append(used)
        saturated.append(np.abs(u - used) > 1e-12)
        estimator.predict(used, dt)
    n = len(states)
    status["completed_intervals"] = n - 1
    if (
        status["status"] == "completed"
        and min(states[-1][2], estimator.state[2]) <= nav["min_forward_speed_mps"]
    ):
        status.update(
            status="invalid_domain",
            failure_time_s=float(times[n - 1]),
            message="Final state left forward-speed domain",
            invalid_domain_events=1,
        )
    # Retain a full final posterior even when a domain exit occurred between sensor ticks.
    if history["joint_indices"][-1] != n - 1:
        snapshot = estimator_snapshot(estimator, count)
        history["joint_indices"].append(n - 1)
        for key in ["joint_states", "joint_covariances"]:
            history[key].append(snapshot[key])
    ref, vel, acc = reference.arrays(times[:n])
    log = {key: np.array(value) for key, value in history.items()}
    log.update(
        times=times[:n],
        states=np.array(states),
        prior_estimates=np.array(priors),
        controller_states=log["estimates"].copy(),
        reference_states=ref,
        reference_velocity=vel,
        reference_acceleration=acc,
        requested=np.array(requested).reshape(-1, 2),
        applied=np.array(applied).reshape(-1, 2),
        saturated=np.array(saturated, dtype=bool).reshape(-1, 2),
        disturbance=disturbance[: n - 1],
        speed_observations=np.array(speeds),
        landmark_observations=np.array(readings),
        landmark_visible=np.array(visible),
        landmark_events=np.array(codes),
        landmark_nis=np.array(innovations),
    )
    clearance, lower = audit_clearance(log, world, rover_radius(nav))
    log["sampled_clearance_m"], log["interval_clearance_bound_m"] = clearance, lower
    return log, status


def replay_slam(log, config, mode):
    """Estimator-only replay. Reads no actual states, true map, or hidden disturbances for SLAM."""
    estimator = new_estimator(config, log["reference_states"][0], mode)
    count = log["landmark_visible"].shape[1]
    history = empty_history()
    events, nis, priors = [], [], []
    for k, time in enumerate(log["times"]):
        ids = np.flatnonzero(log["landmark_visible"][k])
        speed = log["speed_observations"][k]
        packet = Observation(
            None if np.isnan(speed) else speed, ids, log["landmark_observations"][k, ids]
        )
        priors.append(estimator.state[:4].copy())
        codes, innovations = correct_estimator(estimator, packet, time, mode, count)
        events.append(codes)
        nis.append(innovations)
        append_snapshot(
            history,
            estimator,
            count,
            k,
            config["sensors"]["landmark_every_steps"],
            k == len(log["times"]) - 1,
        )
        if k < len(log["applied"]):
            estimator.predict(log["applied"][k], log["times"][k + 1] - time)
    history.update(landmark_events=events, landmark_nis=nis, prior_estimates=priors)
    return {key: np.array(value) for key, value in history.items()}


def map_metrics(log, config):
    final = log["landmark_estimates"][-1]
    mask = np.all(np.isfinite(final), axis=1)
    truth = np.asarray(config["truth_landmarks_xy_m"])
    rmse = (
        float(np.sqrt(np.mean(np.sum((final[mask] - truth[mask]) ** 2, axis=1))))
        if np.any(mask)
        else None
    )
    first = np.full_like(final, np.nan)
    for j in np.flatnonzero(mask):
        samples = np.flatnonzero(log["landmark_events"][:, j] == 1)
        if len(samples):
            first[j] = log["landmark_estimates"][samples[0], j]
    first_mask = np.all(np.isfinite(first), axis=1)
    initial_rmse = (
        float(np.sqrt(np.mean(np.sum((first[first_mask] - truth[first_mask]) ** 2, axis=1))))
        if np.any(first_mask)
        else None
    )
    pair_error = []
    for i in np.flatnonzero(mask):
        for j in np.flatnonzero(mask):
            if i < j:
                pair_error.append(
                    np.linalg.norm(final[i] - final[j]) - np.linalg.norm(truth[i] - truth[j])
                )
    revisits = np.any(log["landmark_events"] == 3, axis=1)
    return {
        "mapped_landmarks": int(mask.sum()),
        "final_map_rmse_m": rmse,
        "first_sighting_map_rmse_m": initial_rmse,
        "map_pair_distance_rmse_m": float(np.sqrt(np.mean(np.array(pair_error) ** 2)))
        if pair_error
        else None,
        "new_landmarks": int(np.count_nonzero(log["landmark_events"] == 1)),
        "revisit_packets": int(revisits.sum()),
        "first_revisit_time_s": float(log["times"][np.flatnonzero(revisits)[0]])
        if np.any(revisits)
        else None,
    }


def slam_metrics(log, status, config):
    return {**localization_metrics(log, status, config), **map_metrics(log, config)}


def outcomes_table(outcomes):
    rows = [
        "| Case | Seed | Estimator | Pose RMS [m] | Map RMS [m] | Mapped | Min clearance [m] | Overlaps |",
        "| :-- | --: | :-- | --: | --: | --: | --: | --: |",
    ]
    for name, m in outcomes.items():
        case, seed, mode = name.split("__")
        mapped = f"{m['final_map_rmse_m']:.4f}" if m["final_map_rmse_m"] is not None else "supplied"
        rows.append(
            f"| {case.replace('_', ' ')} | {seed} | {mode.replace('_', ' ')} | "
            f"{m['rms_localization_m']:.4f} | {mapped} | {m['mapped_landmarks']} | "
            f"{m['min_disk_clearance_m']:.4f} | {m['disk_overlap_samples']} |"
        )
    return "\n".join(rows) + "\n"


def summary_table(outcomes, config):
    rows = [
        "| Case | Estimator | Mean pose RMS [m] | Mean map RMS [m] | Mean tracking RMS [m] | Clear runs |",
        "| :-- | :-- | --: | --: | --: | --: |",
    ]
    for scenario in config["scenarios"]:
        for mode in MODES:
            runs = [outcomes[f"{scenario['name']}__{seed}__{mode}"] for seed in config["seeds"]]
            pose = np.mean([m["rms_localization_m"] for m in runs])
            tracking = np.mean([m["rms_position_m"] for m in runs])
            maps = [m["final_map_rmse_m"] for m in runs if m["final_map_rmse_m"] is not None]
            mapped = f"{np.mean(maps):.4f}" if maps else "supplied"
            clear = sum(m["execution_status"] == "clear_within_numerical_audit" for m in runs)
            rows.append(
                f"| {scenario['name'].replace('_', ' ')} | {mode.replace('_', ' ')} | "
                f"{pose:.4f} | {mapped} | {tracking:.4f} | {clear}/{len(runs)} |"
            )
    return "\n".join(rows) + "\n"


def plan_route(config):
    nav = config["navigation"]
    world = World.from_config(nav["world"])
    search_world = world.inflate(rover_radius(nav) + nav["search_margin_m"])
    paths, searches = [], []
    for start, goal in zip(config["route_xy_m"][:-1], config["route_xy_m"][1:], strict=True):
        result = grid_search(search_world, start, goal, nav["grid_resolution_m"])
        if result.status != "found":
            raise ValueError("A route leg has no grid path; no rover execution")
        path = shortcut_path(result.path, search_world)
        paths.extend(path if not paths else path[1:])
        searches.append(
            {
                "start": start,
                "goal": goal,
                "graph_cost_m": result.cost,
                "expanded_nodes": len(result.expanded),
            }
        )
    reference, certificate = build_rounded_reference(
        np.array(paths),
        world.inflate(rover_radius(nav) + nav["reference_margin_m"]),
        nav["cruise_speed_mps"],
        nav["reference_max_acceleration_mps2"],
        nav["reference_max_yaw_rate_radps"],
        nav["dt_s"],
    )
    if certificate["bounds"]["min_speed_mps"] <= nav["min_forward_speed_mps"]:
        raise ValueError("Route reference leaves forward-speed domain")
    return reference, certificate, np.array(paths), searches


def run_slam(config_path, output):
    root, config = Path(output), read_json(config_path)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"Results directory is not empty: {root}")
    validate_slam_config(config)
    reference, certificate, path, searches = plan_route(config)
    times = time_grid(reference.duration, config["navigation"]["dt_s"])
    root.mkdir(parents=True, exist_ok=True)
    write_json(root / "config.json", config)
    write_json(root / "reference_certificate.json", certificate)
    write_json(root / "planning_summary.json", searches)
    np.savez_compressed(
        root / "plan.npz",
        times=times,
        reference_states=reference.arrays(times)[0],
        bezier_controls=reference.controls,
        segment_durations=reference.durations,
        path=path,
    )
    entries, outcomes, replays, replay_outcomes = [], {}, [], {}
    for seed in config["seeds"]:
        inputs = draw_inputs({**config, "sensors": simulator_sensors(config)}, times, seed)
        input_name = f"seed_{seed}_inputs.npz"
        np.savez_compressed(root / input_name, **inputs)
        for scenario in config["scenarios"]:
            for mode in MODES:
                name = f"{scenario['name']}__{seed}__{mode}"
                log, status = simulate_slam(config, reference, times, inputs, scenario, mode)
                np.savez_compressed(root / (name + ".npz"), **log)
                write_json(
                    root / (name + ".json"),
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "mode": mode,
                        "status": status,
                        "noise_inputs": input_name,
                        "note": "Only known_map receives surveyed coordinates; all controllers receive estimates",
                    },
                )
                entries.append({"name": name, "log": name + ".npz", "metadata": name + ".json"})
                saved = load_log(root / (name + ".npz"))
                outcomes[name] = slam_metrics(saved, status, config)
                if mode == "slam":
                    replay = replay_slam(saved, config, "no_revisit")
                    replay_name = f"{scenario['name']}__{seed}__no_revisit"
                    np.savez_compressed(root / (replay_name + ".npz"), **replay)
                    evaluated = {**saved, **replay}
                    replay_outcomes[replay_name] = {
                        **map_metrics(evaluated, config),
                        "rms_localization_m": float(
                            np.sqrt(
                                np.mean(
                                    np.sum(
                                        (replay["estimates"][:, :2] - saved["states"][:, :2]) ** 2,
                                        axis=1,
                                    )
                                )
                            )
                        ),
                    }
                    replays.append(
                        {"name": replay_name, "log": replay_name + ".npz", "source_run": name}
                    )
    write_json(root / "metrics.json", outcomes)
    write_json(root / "replay_metrics.json", replay_outcomes)
    (root / "outcomes.md").write_text(outcomes_table(outcomes))
    (root / "summary.md").write_text(summary_table(outcomes, config))
    write_json(
        root / "manifest.json",
        {
            "schema_version": 1,
            "milestone": 4,
            "name": config["name"],
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "joint_state_order": "x,y,v,theta followed by dynamic landmark x,y pairs",
            "log_joint_order": "canonical ID order; NaN means undiscovered; full covariance at joint_indices",
            "observation_timing": "correct, control posterior, held plant RK4, predict applied command",
            "map_frame": "anchored by finite initial rover prior; no truth alignment in estimation or control",
            "event_codes": EVENT_CODES,
            "runs": entries,
            "replays": replays,
            "source_sha256": {
                p.name: sha256(p)
                for p in sorted(Path(__file__).parent.glob("*.py"))
                if not p.name.endswith("render.py") and p.name not in {"theme.py", "cli.py"}
            },
            "files_sha256": {p.name: sha256(p) for p in sorted(root.iterdir()) if p.is_file()},
        },
    )
    return outcomes


def verify_slam(root):
    root = Path(root)
    manifest, config = read_json(root / "manifest.json"), read_json(root / "config.json")
    for name, digest in manifest["files_sha256"].items():
        if sha256(root / name) != digest:
            raise ValueError(f"SLAM checksum mismatch: {name}")
    outcomes = read_json(root / "metrics.json")
    if (root / "summary.md").read_text() != summary_table(outcomes, config) or (
        root / "outcomes.md"
    ).read_text() != outcomes_table(outcomes):
        raise ValueError("SLAM table mismatch")
    for entry in manifest["runs"]:
        log, metadata = load_log(root / entry["log"]), read_json(root / entry["metadata"])
        if slam_metrics(log, metadata["status"], config) != outcomes[entry["name"]]:
            raise ValueError(f"SLAM metrics mismatch: {entry['name']}")
        for key, value in replay_slam(log, config, metadata["mode"]).items():
            if not np.array_equal(log[key], value, equal_nan=True):
                raise ValueError(f"SLAM estimator replay mismatch: {entry['name']}/{key}")
        if not np.array_equal(log["controller_states"], log["estimates"]):
            raise ValueError(f"SLAM controller state mismatch: {entry['name']}")
        calculated = audit_clearance(
            log,
            World.from_config(config["navigation"]["world"]),
            rover_radius(config["navigation"]),
        )
        for key, value in zip(
            ["sampled_clearance_m", "interval_clearance_bound_m"], calculated, strict=True
        ):
            if not np.array_equal(log[key], value):
                raise ValueError(f"SLAM clearance mismatch: {entry['name']}")
    reported = read_json(root / "replay_metrics.json")
    for entry in manifest["replays"]:
        source, saved = (
            load_log(root / (entry["source_run"] + ".npz")),
            load_log(root / entry["log"]),
        )
        replay = replay_slam(source, config, "no_revisit")
        for key, value in replay.items():
            if not np.array_equal(saved[key], value, equal_nan=True):
                raise ValueError(f"Counterfactual replay mismatch: {entry['name']}/{key}")
        actual = {
            **map_metrics({**source, **saved}, config),
            "rms_localization_m": float(
                np.sqrt(
                    np.mean(
                        np.sum((saved["estimates"][:, :2] - source["states"][:, :2]) ** 2, axis=1)
                    )
                )
            ),
        }
        if actual != reported[entry["name"]]:
            raise ValueError(f"Counterfactual metrics mismatch: {entry['name']}")
    return manifest
