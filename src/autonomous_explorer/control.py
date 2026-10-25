"""Flatness inversion and sampled controllers; no dynamics or rendering here."""

import numpy as np

from .model import State
from .reference import ReferenceSample


class InvalidSpeed(ValueError):
    """The declared forward-speed domain of the flatness inversion was left."""


def invert_acceleration(state: State, desired_acceleration, min_speed=1e-4):
    """Invert p_ddot = [[cos(theta), -v sin(theta)],
    [sin(theta), v cos(theta)]] @ [a, omega]. Forward domain v > min_speed.
    No clamping or regularization: the caller must record an invalid-domain exit.
    """
    if not np.isfinite(state.v) or state.v <= min_speed:
        raise InvalidSpeed(f"v={state.v:.8g} m/s is outside v > {min_speed:g} m/s")
    c, s = np.cos(state.theta), np.sin(state.theta)
    wx, wy = desired_acceleration
    return np.array([c * wx + s * wy, (-s * wx + c * wy) / state.v])


class OpenLoop:
    """Precomputed commands indexed by step; API deliberately has NO actual state."""

    def __init__(self, commands):
        self.commands = np.array(commands, dtype=float, copy=True)

    def command(self, index: int):
        return self.commands[index].copy()


class Feedback:
    def __init__(self, kp=(2.0, 2.0), kd=(2.0, 2.0), min_speed=1e-4):
        self.kp, self.kd = np.asarray(kp), np.asarray(kd)
        self.min_speed = min_speed

    def command(self, actual: State, desired: ReferenceSample):
        position_error = np.array([desired.state.x - actual.x, desired.state.y - actual.y])
        velocity_error = desired.velocity - actual.velocity
        virtual_acceleration = (
            desired.acceleration + self.kp * position_error + self.kd * velocity_error
        )
        return invert_acceleration(actual, virtual_acceleration, self.min_speed)
