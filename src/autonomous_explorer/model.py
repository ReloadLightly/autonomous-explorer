"""Kinematic rover model. SI units; state array order is (x, y, v, theta)."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class State:
    x: float
    y: float
    v: float
    theta: float

    def array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.v, self.theta], dtype=float)

    @classmethod
    def from_array(cls, values: np.ndarray) -> "State":
        if np.shape(values) != (4,):
            raise ValueError("State must have shape (4,): x, y, v, theta")
        return cls(*map(float, values))

    @property
    def velocity(self) -> np.ndarray:
        return self.v * np.array([np.cos(self.theta), np.sin(self.theta)])


def wrap_angle(angle):
    """Principal angle in [-pi, pi), for errors only; integration stays unwrapped."""
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def derivative(state, control, disturbance=(0.0, 0.0)) -> np.ndarray:
    """Return (xdot, ydot, vdot, thetadot).

    control = (a [m/s²], omega [rad/s]). Exogenous disturbance =
    (body-forward speed offset [m/s], yaw-rate offset [rad/s]).
    The speed offset affects translation, NOT the stored speed derivative.
    """
    _, _, v, theta = state
    a, omega = control
    dv, domega = disturbance
    return np.array([(v + dv) * np.cos(theta), (v + dv) * np.sin(theta), a, omega + domega])


def step(state, control, dt, disturbance=(0.0, 0.0), method="rk4") -> np.ndarray:
    """Advance with inputs held constant over the entire step, including RK stages."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")
    state = np.asarray(state, dtype=float)
    k1 = derivative(state, control, disturbance)
    if method == "euler":
        return state + dt * k1
    if method != "rk4":
        raise ValueError(f"Unknown integration method: {method}")
    k2 = derivative(state + dt * k1 / 2, control, disturbance)
    k3 = derivative(state + dt * k2 / 2, control, disturbance)
    k4 = derivative(state + dt * k3, control, disturbance)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def constant_turn_exact(initial: State, speed: float, omega: float, times) -> np.ndarray:
    """Analytical benchmark for a=0, constant speed and yaw rate, no disturbance."""
    times = np.asarray(times, dtype=float)
    theta = initial.theta + omega * times
    if omega == 0:
        x = initial.x + speed * times * np.cos(initial.theta)
        y = initial.y + speed * times * np.sin(initial.theta)
    else:
        radius = speed / omega
        x = initial.x + radius * (np.sin(theta) - np.sin(initial.theta))
        y = initial.y - radius * (np.cos(theta) - np.cos(initial.theta))
    return np.column_stack([x, y, np.full_like(times, speed), theta])
