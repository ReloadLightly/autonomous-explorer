"""Sample, hold, integrate, log. Exogenous inputs are generated before either run."""

from dataclasses import dataclass

import numpy as np

from .control import Feedback, InvalidSpeed, OpenLoop, invert_acceleration
from .model import State, step


@dataclass
class Run:
    times: np.ndarray
    states: np.ndarray
    requested: np.ndarray
    applied: np.ndarray
    disturbance: np.ndarray
    saturated: np.ndarray
    status: dict


def time_grid(duration, dt):
    if not np.isfinite(duration) or not np.isfinite(dt) or duration <= 0 or dt <= 0:
        raise ValueError("Duration and dt must be positive and finite")
    steps = round(duration / dt)
    if steps < 1 or not np.isclose(steps * dt, duration, atol=1e-12, rtol=1e-12):
        raise ValueError("Positive duration must be an integer multiple of positive dt")
    return np.linspace(0, duration, steps + 1)


def disturbances(scenario: dict, times) -> np.ndarray:
    """Return N-1 samples, held on [t[k], t[k+1]); never called by RK4."""
    result = np.zeros((len(times) - 1, 2))
    kind = scenario["kind"]
    if kind == "yaw_bias":
        active = (times[:-1] >= scenario["start_s"]) & (times[:-1] < scenario["end_s"])
        result[active, 1] = scenario["yaw_rate_radps"]
    elif kind == "gaussian":
        if scenario.get("seed") is None:
            raise ValueError("Gaussian scenario requires an explicit seed")
        rng = np.random.default_rng(scenario["seed"])
        result = rng.normal(size=result.shape) * [
            scenario["speed_std_mps"],
            scenario["yaw_std_radps"],
        ]
    elif kind != "none":
        raise ValueError(f"Unknown disturbance kind: {kind}")
    return result


def simulate(initial: State, reference, times, exogenous, controller, limits=None) -> Run:
    if np.shape(exogenous) != (len(times) - 1, 2):
        raise ValueError("Disturbance shape must be (N-1, 2)")
    states, requested, applied, saturated = [initial.array()], [], [], []
    status = {
        "status": "completed",
        "failure_time_s": None,
        "message": None,
        "invalid_domain_events": 0,
    }
    for k, dt in enumerate(np.diff(times)):
        actual = State.from_array(states[-1])
        try:
            # Open loop cannot observe actual: only an integer index is passed.
            u = (
                controller.command(k)
                if isinstance(controller, OpenLoop)
                else controller.command(actual, reference.sample(float(times[k])))
            )
        except InvalidSpeed as exc:
            status.update(
                status="invalid_domain",
                failure_time_s=float(times[k]),
                message=str(exc),
                invalid_domain_events=1,
            )
            break
        used = u.copy() if limits is None else np.clip(u, -np.asarray(limits), limits)
        candidate = step(states[-1], used, dt, exogenous[k])
        if not np.all(np.isfinite(candidate)):
            status.update(
                status="nonfinite",
                failure_time_s=float(times[k]),
                message="Nonfinite candidate state; step was not accepted",
            )
            break
        requested.append(u)
        applied.append(used)
        saturated.append(np.abs(u - used) > 1e-12)
        states.append(candidate)
    count = len(states)
    status["completed_intervals"] = count - 1
    status["planned_intervals"] = len(times) - 1
    # Guard the final state too, where there is no next controller call.
    if (
        isinstance(controller, Feedback)
        and states[-1][2] <= controller.min_speed
        and status["status"] == "completed"
    ):
        status.update(
            status="invalid_domain",
            failure_time_s=float(times[count - 1]),
            message="Final state outside forward-speed domain",
            invalid_domain_events=1,
        )
    return Run(
        np.array(times[:count]),
        np.array(states),
        np.array(requested).reshape(-1, 2),
        np.array(applied).reshape(-1, 2),
        np.array(exogenous[: count - 1], copy=True),
        np.array(saturated, dtype=bool).reshape(-1, 2),
        status,
    )


def paired_runs(config, reference, times, scenario):
    exogenous = disturbances(scenario, times)
    feedforward = np.array(
        [
            invert_acceleration(
                reference.sample(float(t)).state,
                reference.sample(float(t)).acceleration,
                config["min_forward_speed_mps"],
            )
            for t in times[:-1]
        ]
    )
    controllers = {
        "open_loop": OpenLoop(feedforward),
        "feedback": Feedback(
            config["kp_per_s2"], config["kd_per_s"], config["min_forward_speed_mps"]
        ),
    }
    return {
        name: simulate(
            State.from_array(config["initial_state"]),
            reference,
            times,
            exogenous,
            controller,
            config["actuator_limits"],
        )
        for name, controller in controllers.items()
    }
