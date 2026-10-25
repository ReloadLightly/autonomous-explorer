"""Persist numerical evidence first; metrics and rendering consume those files."""

import hashlib
import json
import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import __version__
from .control import Feedback
from .model import State, constant_turn_exact, step, wrap_angle
from .reference import CubicReference
from .simulation import paired_runs, simulate, time_grid


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def read_json(path):
    return json.loads(Path(path).read_text())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_log(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def metrics(log, status, min_speed):
    """Sample RMS over saved states; command effort integrates held inputs."""
    actual, desired = log["states"], log["reference_states"]
    position = np.linalg.norm(actual[:, :2] - desired[:, :2], axis=1)
    speed = actual[:, 2] - desired[:, 2]
    heading = wrap_angle(actual[:, 3] - desired[:, 3])
    dt = np.diff(log["times"])
    completed = status["status"] == "completed"
    return {
        "status": status["status"],
        "rms_position_m": float(np.sqrt(np.mean(position**2))),
        "max_position_m": float(np.max(position)),
        "final_position_m": float(position[-1]) if completed else None,
        "last_position_m": float(position[-1]),
        "rms_speed_mps": float(np.sqrt(np.mean(speed**2))),
        "final_speed_error_mps": float(speed[-1]) if completed else None,
        "rms_heading_rad": float(np.sqrt(np.mean(heading**2))),
        "final_heading_error_rad": float(heading[-1]) if completed else None,
        "min_actual_speed_mps": float(np.min(actual[:, 2])),
        "speed_samples_outside_forward_domain": int(np.count_nonzero(actual[:, 2] <= min_speed)),
        "saturated_intervals": int(np.count_nonzero(np.any(log["saturated"], axis=1))),
        "invalid_domain_events": status["invalid_domain_events"],
        "acceleration_effort_m2_per_s3": float(np.sum(log["applied"][:, 0] ** 2 * dt)),
        "yaw_effort_rad2_per_s": float(np.sum(log["applied"][:, 1] ** 2 * dt)),
        "observed_horizon_s": float(log["times"][-1]),
        "samples": len(actual),
    }


def save_run(path, run, reference):
    ref, vel, acc = reference.arrays(run.times)
    np.savez_compressed(
        path,
        times=run.times,
        states=run.states,
        reference_states=ref,
        reference_velocity=vel,
        reference_acceleration=acc,
        requested=run.requested,
        applied=run.applied,
        disturbance=run.disturbance,
        saturated=run.saturated,
    )


def outcome_table(outcomes):
    """A compact, reproducible Markdown table for README and saved evidence."""
    rows = [
        "| Scenario | Controller | RMS position [m] | Maximum [m] | Final [m] |",
        "| :-- | :-- | --: | --: | --: |",
    ]
    for name, result in outcomes.items():
        scenario, controller = name.split("__")
        final = (
            "failed" if result["final_position_m"] is None else f"{result['final_position_m']:.4f}"
        )
        rows.append(
            f"| {scenario.replace('_', ' ')} | {controller.replace('_', ' ')} | "
            f"{result['rms_position_m']:.4f} | {result['max_position_m']:.4f} | {final} |"
        )
    return "\n".join(rows) + "\n"


def save_foundations(root):
    """Measured primitives and analytical integration study, saved for rendering."""
    arrays = {}
    commands = {
        "straight": (4.0, lambda t: [0.0, 0.0]),
        "turn": (4.0, lambda t: [0.0, 0.35]),
        "circle": (2 * np.pi / 0.5, lambda t: [0.0, 0.5]),
        "composed": (
            10.0,
            lambda t: [
                0.10 if t < 2 else (-0.10 if t >= 8 else 0),
                0.6 if 3 <= t < 6 else (-0.4 if 7 <= t < 9 else 0),
            ],
        ),
    }
    for name, (duration, command) in commands.items():
        t = np.linspace(0, duration, round(duration / 0.02) + 1)
        states = [State(0, 0, 0.5, 0).array()]
        inputs = np.array([command(float(ti)) for ti in t[:-1]])
        for k, dt in enumerate(np.diff(t)):
            states.append(step(states[-1], inputs[k], dt))
        arrays[f"{name}_times"] = t
        arrays[f"{name}_states"] = np.array(states)
        arrays[f"{name}_controls"] = inputs
    np.savez_compressed(root / "primitives.npz", **arrays)
    timesteps = np.array([0.5, 0.25, 0.125, 0.0625, 0.03125])
    initial = State(0, 0, 0.8, 0.3)
    exact = constant_turn_exact(initial, 0.8, 0.7, [4.0])[-1]
    study = {
        "dt": timesteps,
        "exact_final": exact,
        "initial": initial.array(),
        "control": np.array([0.0, 0.7]),
        "duration_s": np.array(4.0),
    }
    for method in ["euler", "rk4"]:
        finals = []
        for dt in timesteps:
            state = initial.array()
            for _ in range(round(4 / dt)):
                state = step(state, [0, 0.7], dt, method=method)
            finals.append(state)
        study[method + "_final"] = np.array(finals)
        study[method + "_error"] = np.linalg.norm(np.array(finals)[:, :2] - exact[:2], axis=1)
    np.savez_compressed(root / "integration.npz", **study)


def run_experiments(config_path, output):
    config = read_json(config_path)
    root = Path(output)
    # Protect earlier work: an evidence directory must be empty or new.
    if root.exists() and any(root.iterdir()):
        raise ValueError(f"Results directory is not empty: {root}. Choose a new --results path.")
    root.mkdir(parents=True, exist_ok=True)
    write_json(root / "config.json", config)
    times = time_grid(config["duration_s"], config["dt_s"])
    reference = CubicReference(
        State.from_array(config["initial_state"]),
        State.from_array(config["final_state"]),
        config["duration_s"],
    )
    minimum, minimum_time = reference.minimum_speed()
    if minimum <= config["min_forward_speed_mps"]:
        raise ValueError("Reference leaves declared forward-speed domain")
    manifest = {
        "schema_version": 1,
        "experiment": config["name"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "package_version": __version__,
        "python": platform.python_version(),
        "numpy": np.__version__,
        "integrator": "RK4; zero-order-held commands/disturbances",
        "reference_min_speed_mps": minimum,
        "reference_min_speed_time_s": minimum_time,
        "reference_coefficients_tau_ascending": reference.coefficients.tolist(),
        "state_order": ["x_m", "y_m", "v_mps", "theta_rad"],
        "control_order": ["a_mps2", "omega_radps"],
        "disturbance_order": ["speed_offset_mps", "yaw_rate_offset_radps"],
        "upstream_revision": "02ee1421786b73ae4810b086cd1f59882ab972bc",
        "source_sha256": {
            p.name: sha256(p)
            for p in sorted(Path(__file__).parent.glob("*.py"))
            if p.name not in {"render.py", "theme.py", "cli.py"}
        },
        "runs": [],
    }
    for scenario in config["scenarios"]:
        for controller, run in paired_runs(config, reference, times, scenario).items():
            name = f"{scenario['name']}__{controller}"
            save_run(root / (name + ".npz"), run, reference)
            metadata = {
                "scenario": scenario,
                "controller": controller,
                "status": run.status,
                "configuration": config,
                "log": name + ".npz",
            }
            write_json(root / (name + ".json"), metadata)
            manifest["runs"].append(
                {
                    "name": name,
                    "scenario": scenario["name"],
                    "controller": controller,
                    "log": name + ".npz",
                    "metadata": name + ".json",
                }
            )
    # A deliberate failure is retained separately, never mixed into performance results.
    stopped = State(0, 0, 0, config["initial_state"][3])
    probe = simulate(
        stopped,
        reference,
        times,
        np.zeros((len(times) - 1, 2)),
        Feedback(min_speed=config["min_forward_speed_mps"]),
    )
    save_run(root / "zero_speed_probe.npz", probe, reference)
    write_json(
        root / "zero_speed_probe.json",
        {
            "purpose": "Deliberate invalid-domain probe",
            "initial_state": stopped.array().tolist(),
            "status": probe.status,
        },
    )
    save_foundations(root)
    # Reload: reported metrics come from exactly the persisted data used by rendering.
    outcome = {}
    for entry in manifest["runs"]:
        log = load_log(root / entry["log"])
        metadata = read_json(root / entry["metadata"])
        outcome[entry["name"]] = metrics(log, metadata["status"], config["min_forward_speed_mps"])
    write_json(root / "metrics.json", outcome)
    (root / "outcomes.md").write_text(outcome_table(outcome))
    manifest["files_sha256"] = {p.name: sha256(p) for p in sorted(root.iterdir()) if p.is_file()}
    write_json(root / "manifest.json", manifest)
    return outcome


def verify_evidence(root):
    root = Path(root)
    manifest = read_json(root / "manifest.json")
    for name, expected in manifest["files_sha256"].items():
        if sha256(root / name) != expected:
            raise ValueError(f"Evidence checksum mismatch: {name}")
    reported = read_json(root / "metrics.json")
    config = read_json(root / "config.json")
    for entry in manifest["runs"]:
        actual = metrics(
            load_log(root / entry["log"]),
            read_json(root / entry["metadata"])["status"],
            config["min_forward_speed_mps"],
        )
        if reported[entry["name"]] != actual:
            raise ValueError(f"Metrics differ from saved log: {entry['name']}")
    return manifest
