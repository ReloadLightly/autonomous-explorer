"""Milestone 03: causal sensing/estimation/control experiments and saved-data audits."""

import platform
import re
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .control import Feedback, InvalidSpeed
from .evidence import load_log, read_json, sha256, write_json
from .localization import ExtendedKalmanFilter
from .model import State, step, wrap_angle
from .navigation_evidence import audit_clearance, navigation_metrics
from .path_reference import build_rounded_reference
from .planning import World, grid_search, rover_radius, shortcut_path
from .sensors import Observation, sample_observation
from .simulation import time_grid

MODES = ("oracle", "dead_reckoning", "ekf")


def validate_config(config):
    sensors = config["sensors"]
    for key in ["speed_every_steps", "landmark_every_steps"]:
        if type(sensors[key]) is not int or sensors[key] <= 0:
            raise ValueError(f"{key} must be a positive integer")
    for key in ["speed_std_mps", "range_std_m", "bearing_std_rad", "max_range_m"]:
        if not np.isfinite(sensors[key]) or sensors[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    for key, size, positive in [
        ("initial_estimate_offset", 4, False),
        ("initial_std", 4, True),
        ("process_held_std", 3, True),
        ("truth_disturbance_std", 2, True),
    ]:
        values = np.asarray(config[key], dtype=float)
        if values.shape != (size,) or not np.all(np.isfinite(values)):
            raise ValueError(f"Invalid {key}")
        if positive and np.any(values <= 0):
            raise ValueError(f"{key} must be positive")
    seeds = config["seeds"]
    if (
        not seeds
        or any(type(s) is not int or s < 0 for s in seeds)
        or len(set(seeds)) != len(seeds)
    ):
        raise ValueError("Require distinct nonnegative integer seeds")
    scenarios = config["scenarios"]
    names = [s["name"] for s in scenarios]
    if (
        not names
        or len(set(names)) != len(names)
        or any(not re.fullmatch(r"[a-z][a-z0-9_]*", name) for name in names)
    ):
        raise ValueError("Require unique scenario names using lowercase letters/digits/underscores")
    for scenario in scenarios:
        if not np.isfinite(scenario["yaw_bias_radps"]):
            raise ValueError("Yaw bias must be finite")
        outage = scenario["landmark_outage_s"]
        if outage is not None and (
            np.shape(outage) != (2,)
            or not np.all(np.isfinite(outage))
            or not 0 <= outage[0] < outage[1]
        ):
            raise ValueError("Invalid landmark outage interval")
    interval = config["yaw_bias_interval_s"]
    if (
        np.shape(interval) != (2,)
        or not np.all(np.isfinite(interval))
        or not 0 <= interval[0] < interval[1]
    ):
        raise ValueError("Invalid yaw bias interval")
    landmarks = np.asarray(sensors["landmarks_xy_m"], dtype=float)
    world = World.from_config(config["navigation"]["world"])
    if (
        landmarks.ndim != 2
        or landmarks.shape[1] != 2
        or len(landmarks) == 0
        or not np.all(np.isfinite(landmarks))
        or not all(world.point_free(p) for p in landmarks)
    ):
        raise ValueError("Landmarks must be finite free-space points")


def new_filter(initial, config):
    return ExtendedKalmanFilter(
        np.asarray(initial) + config["initial_estimate_offset"],
        np.diag(np.asarray(config["initial_std"]) ** 2),
        config["process_held_std"],
        config["sensors"],
    )


def draw_inputs(config, times, seed):
    # Independent streams prevent sensor configuration from changing plant noise.
    streams = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(3)]
    count, landmarks = len(times), len(config["sensors"]["landmarks_xy_m"])
    return {
        "disturbance": streams[0].normal(size=(count - 1, 2)) * config["truth_disturbance_std"],
        "speed_noise": streams[1].normal(size=count) * config["sensors"]["speed_std_mps"],
        "landmark_noise": streams[2].normal(size=(count, landmarks, 2))
        * [
            config["sensors"]["range_std_m"],
            config["sensors"]["bearing_std_rad"],
        ],
    }


def simulate_localized(config, reference, times, inputs, scenario, mode):
    if mode not in MODES:
        raise ValueError(f"Unknown localization mode: {mode}")
    nav, sensors = config["navigation"], config["sensors"]
    world = World.from_config(nav["world"])
    initial = reference.sample(0).state.array()
    estimator = new_filter(initial, config)
    controller = Feedback(nav["kp_per_s2"], nav["kd_per_s"], nav["min_forward_speed_mps"])
    delta = np.array(inputs["disturbance"], copy=True)
    start, end = config["yaw_bias_interval_s"]
    active = (times[:-1] >= start) & (times[:-1] < end)
    delta[active, 1] += scenario["yaw_bias_radps"]
    states, estimates, covariances, supplied = [initial], [], [], []
    requested, applied, saturated, observations, visibility, speeds = [], [], [], [], [], []
    speed_nis, beacon_nis = [], []
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
        nis_v, nis_b = (None, []) if mode == "dead_reckoning" else estimator.correct(packet)
        estimate = estimator.state.copy()
        # Only the explicitly named oracle baseline is allowed actual-state feedback.
        control_state = states[-1].copy() if mode == "oracle" else estimate.copy()
        estimates.append(estimate)
        covariances.append(estimator.covariance.copy())
        supplied.append(control_state)
        values = np.full((len(sensors["landmarks_xy_m"]), 2), np.nan)
        mask = np.zeros(len(values), dtype=bool)
        values[packet.landmark_ids], mask[packet.landmark_ids] = packet.range_bearing, True
        observations.append(values)
        visibility.append(mask)
        speeds.append(np.nan if packet.speed is None else packet.speed)
        speed_nis.append(np.nan if nis_v is None else nis_v)
        per_beacon = np.full(len(values), np.nan)
        if nis_b:
            per_beacon[packet.landmark_ids] = nis_b
        beacon_nis.append(per_beacon)
        if k == len(times) - 1:
            break
        try:
            if states[-1][2] <= nav["min_forward_speed_mps"]:
                raise InvalidSpeed("Physical state left the forward-speed domain")
            u = controller.command(State.from_array(control_state), reference.sample(float(time)))
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
        candidate = step(states[-1], used, dt, delta[k])
        if not np.all(np.isfinite(candidate)):
            status.update(
                status="nonfinite", failure_time_s=float(time), message="Nonfinite plant state"
            )
            break
        requested.append(u)
        applied.append(used)
        saturated.append(np.abs(u - used) > 1e-12)
        states.append(candidate)
        estimator.predict(used, dt)
    count = len(states)
    status["completed_intervals"] = count - 1
    if status["status"] == "completed" and (
        states[-1][2] <= nav["min_forward_speed_mps"]
        or supplied[-1][2] <= nav["min_forward_speed_mps"]
    ):
        status.update(
            status="invalid_domain",
            failure_time_s=float(times[count - 1]),
            message="Final state left forward-speed domain",
            invalid_domain_events=1,
        )
    ref, vel, acc = reference.arrays(times[:count])
    log = {
        "times": times[:count],
        "states": np.array(states),
        "estimates": np.array(estimates),
        "covariances": np.array(covariances),
        "controller_states": np.array(supplied),
        "reference_states": ref,
        "reference_velocity": vel,
        "reference_acceleration": acc,
        "requested": np.array(requested).reshape(-1, 2),
        "applied": np.array(applied).reshape(-1, 2),
        "saturated": np.array(saturated, dtype=bool).reshape(-1, 2),
        "disturbance": delta[: count - 1],
        "speed_observations": np.array(speeds),
        "landmark_observations": np.array(observations),
        "landmark_visible": np.array(visibility),
        "speed_nis": np.array(speed_nis),
        "landmark_nis": np.array(beacon_nis),
    }
    clearance, lower = audit_clearance(log, world, rover_radius(nav))
    log["sampled_clearance_m"], log["interval_clearance_bound_m"] = clearance, lower
    return log, status


def localization_metrics(log, status, config):
    outcome = navigation_metrics(log, status, config["navigation"])
    error = log["estimates"] - log["states"]
    error[:, 3] = wrap_angle(error[:, 3])
    position = np.linalg.norm(error[:, :2], axis=1)
    mahalanobis = np.einsum(
        "ni,ni->n", error, np.linalg.solve(log["covariances"], error[..., None])[..., 0]
    )
    pxy = log["covariances"][:, :2, :2]
    dxy = np.einsum("ni,ni->n", error[:, :2], np.linalg.solve(pxy, error[:, :2, None])[..., 0])
    nis = log["landmark_nis"][np.isfinite(log["landmark_nis"])]
    outcome.update(
        rms_localization_m=float(np.sqrt(np.mean(position**2))),
        max_localization_m=float(np.max(position)),
        final_localization_m=float(position[-1]),
        rms_estimated_heading_rad=float(np.sqrt(np.mean(error[:, 3] ** 2))),
        mean_nees_4d=float(np.mean(mahalanobis)),
        position_95pct_ellipse_coverage=float(np.mean(dxy <= 5.991464547107979)),
        mean_landmark_nis_2d=float(np.mean(nis)) if len(nis) else None,
        landmark_observations=int(np.count_nonzero(log["landmark_visible"])),
        min_covariance_eigenvalue=float(np.min(np.linalg.eigvalsh(log["covariances"]))),
    )
    return outcome


def localization_table(outcomes):
    rows = [
        "| Case | Seed | Feedback state | Localization RMS [m] | Tracking RMS [m] | Min clearance [m] | Overlaps |",
        "| :-- | --: | :-- | --: | --: | --: | --: |",
    ]
    for name, m in outcomes.items():
        case, seed, mode = name.split("__")
        rows.append(
            f"| {case.replace('_', ' ')} | {seed} | {mode.replace('_', ' ')} | "
            f"{m['rms_localization_m']:.4f} | {m['rms_position_m']:.4f} | "
            f"{m['min_disk_clearance_m']:.4f} | {m['disk_overlap_samples']} |"
        )
    return "\n".join(rows) + "\n"


def summary_table(outcomes, config):
    """Arithmetic means of per-run RMS, not pooled-sample statistics."""
    rows = [
        "| Case | Feedback state | Mean localization RMS [m] | Mean tracking RMS [m] | Worst interval clearance [m] | Clear runs |",
        "| :-- | :-- | --: | --: | --: | --: |",
    ]
    for scenario in config["scenarios"]:
        for mode in MODES:
            runs = [outcomes[f"{scenario['name']}__{seed}__{mode}"] for seed in config["seeds"]]
            localization = np.mean([m["rms_localization_m"] for m in runs])
            tracking = np.mean([m["rms_position_m"] for m in runs])
            bounds = [
                m["min_interval_clearance_bound_m"]
                for m in runs
                if m["min_interval_clearance_bound_m"] is not None
            ]
            worst = f"{min(bounds):.4f}" if bounds else "n/a"
            clear = sum(m["execution_status"] == "clear_within_numerical_audit" for m in runs)
            rows.append(
                f"| {scenario['name'].replace('_', ' ')} | {mode.replace('_', ' ')} | "
                f"{localization:.4f} | {tracking:.4f} | {worst} | {clear}/{len(runs)} |"
            )
    return "\n".join(rows) + "\n"


def run_localization(config_path, output):
    config_path, root = Path(config_path), Path(output)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"Results directory is not empty: {root}")
    config = read_json(config_path)
    if "navigation" not in config:
        config["navigation"] = read_json(config_path.parent / config["navigation_config"])
    validate_config(config)
    nav = config["navigation"]
    radius, world = rover_radius(nav), World.from_config(nav["world"])
    if not nav["search_margin_m"] >= nav["reference_margin_m"] >= 0:
        raise ValueError("Require search margin >= reference margin >= 0")
    search_world = world.inflate(radius + nav["search_margin_m"])
    search = grid_search(
        search_world, nav["start_xy_m"], nav["goal_xy_m"], nav["grid_resolution_m"]
    )
    if search.status != "found":
        raise ValueError("No known-map route; no localization experiment executed")
    reference, certificate = build_rounded_reference(
        shortcut_path(search.path, search_world),
        world.inflate(radius + nav["reference_margin_m"]),
        nav["cruise_speed_mps"],
        nav["reference_max_acceleration_mps2"],
        nav["reference_max_yaw_rate_radps"],
        nav["dt_s"],
    )
    if certificate["bounds"]["min_speed_mps"] <= nav["min_forward_speed_mps"]:
        raise ValueError("Reference leaves forward-speed domain")
    times = time_grid(reference.duration, nav["dt_s"])
    root.mkdir(parents=True, exist_ok=True)
    write_json(root / "config.json", config)
    write_json(root / "reference_certificate.json", certificate)
    np.savez_compressed(
        root / "plan.npz",
        times=times,
        reference_states=reference.arrays(times)[0],
        bezier_controls=reference.controls,
        segment_durations=reference.durations,
        grid_path=search.path,
    )
    entries, outcomes = [], {}
    for seed in config["seeds"]:
        inputs = draw_inputs(config, times, seed)
        input_name = f"seed_{seed}_inputs.npz"
        np.savez_compressed(root / input_name, **inputs)
        for scenario in config["scenarios"]:
            for mode in MODES:
                name = f"{scenario['name']}__{seed}__{mode}"
                log, status = simulate_localized(config, reference, times, inputs, scenario, mode)
                np.savez_compressed(root / (name + ".npz"), **log)
                write_json(
                    root / (name + ".json"),
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "mode": mode,
                        "status": status,
                        "noise_inputs": input_name,
                        "note": "Clearance uses truth post hoc; controller sees controller_states only",
                    },
                )
                outcomes[name] = localization_metrics(
                    load_log(root / (name + ".npz")), status, config
                )
                entries.append({"name": name, "log": name + ".npz", "metadata": name + ".json"})
    write_json(root / "metrics.json", outcomes)
    (root / "outcomes.md").write_text(localization_table(outcomes))
    (root / "summary.md").write_text(summary_table(outcomes, config))
    write_json(
        root / "manifest.json",
        {
            "schema_version": 1,
            "milestone": 3,
            "name": config["name"],
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "state_order": ["x_m", "y_m", "v_mps", "theta_rad"],
            "process_held_error_order": ["acceleration_mps2", "speed_offset_mps", "yaw_rate_radps"],
            "random_generator": "PCG64; SeedSequence.spawn: plant, speed sensor, landmark sensor",
            "integrator": "RK4; zero-order-held applied commands and plant disturbances",
            "missing_observations": "NaN with explicit landmark_visible mask; never a fabricated zero",
            "observation_timing": "correct at t[k], control posterior, hold/integrate, predict to t[k+1]",
            "source_sha256": {
                p.name: sha256(p)
                for p in sorted(Path(__file__).parent.glob("*.py"))
                if not p.name.endswith("render.py") and p.name not in {"theme.py", "cli.py"}
            },
            "runs": entries,
            "files_sha256": {p.name: sha256(p) for p in sorted(root.iterdir()) if p.is_file()},
        },
    )
    return outcomes


def replay_estimates(log, config, mode):
    """Replay packets/applied commands from the declared reference-based initial prior."""
    estimator = new_filter(log["reference_states"][0], config)
    states, covariances = [], []
    for k, time in enumerate(log["times"]):
        ids = np.flatnonzero(log["landmark_visible"][k])
        speed = log["speed_observations"][k]
        packet = Observation(
            None if np.isnan(speed) else speed, ids, log["landmark_observations"][k, ids]
        )
        if mode != "dead_reckoning":
            estimator.correct(packet)
        states.append(estimator.state.copy())
        covariances.append(estimator.covariance.copy())
        if k < len(log["applied"]):
            estimator.predict(log["applied"][k], log["times"][k + 1] - time)
    return np.array(states), np.array(covariances)


def verify_localization(root):
    root = Path(root)
    manifest, config = read_json(root / "manifest.json"), read_json(root / "config.json")
    for name, digest in manifest["files_sha256"].items():
        if sha256(root / name) != digest:
            raise ValueError(f"Localization checksum mismatch: {name}")
    outcomes = read_json(root / "metrics.json")
    if (root / "summary.md").read_text() != summary_table(outcomes, config) or (
        root / "outcomes.md"
    ).read_text() != localization_table(outcomes):
        raise ValueError("Localization table mismatch")
    for entry in manifest["runs"]:
        log, metadata = load_log(root / entry["log"]), read_json(root / entry["metadata"])
        if localization_metrics(log, metadata["status"], config) != outcomes[entry["name"]]:
            raise ValueError(f"Localization metrics mismatch: {entry['name']}")
        states, covariance = replay_estimates(log, config, metadata["mode"])
        if not np.array_equal(log["estimates"], states) or not np.array_equal(
            log["covariances"], covariance
        ):
            raise ValueError(f"Estimator replay mismatch: {entry['name']}")
        observed = log["states"] if metadata["mode"] == "oracle" else log["estimates"]
        if not np.array_equal(log["controller_states"], observed):
            raise ValueError(f"Controller state mismatch: {entry['name']}")
        clearance, lower = audit_clearance(
            log,
            World.from_config(config["navigation"]["world"]),
            rover_radius(config["navigation"]),
        )
        if not np.array_equal(log["sampled_clearance_m"], clearance) or not np.array_equal(
            log["interval_clearance_bound_m"], lower
        ):
            raise ValueError(f"Clearance mismatch: {entry['name']}")
    return manifest
