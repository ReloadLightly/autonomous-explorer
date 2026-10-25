"""Cubic flat-output reference, solved in normalized time tau=t/T."""

from dataclasses import dataclass

import numpy as np

from .model import State


@dataclass(frozen=True)
class ReferenceSample:
    state: State
    velocity: np.ndarray
    acceleration: np.ndarray


class CubicReference:
    def __init__(self, initial: State, final: State, duration: float):
        if duration <= 0 or not np.isfinite(duration):
            raise ValueError("Reference duration must be positive and finite")
        self.duration = float(duration)
        # Rows enforce p(0), p(1), dp/dtau(0), dp/dtau(1).
        boundary_matrix = np.array(
            [[1, 0, 0, 0], [1, 1, 1, 1], [0, 1, 0, 0], [0, 1, 2, 3]], dtype=float
        )
        boundary_values = np.array(
            [
                [initial.x, initial.y],
                [final.x, final.y],
                duration * initial.velocity,
                duration * final.velocity,
            ]
        )
        # Columns contain x and y coefficients, ascending powers of tau.
        self.coefficients = np.linalg.solve(boundary_matrix, boundary_values)

    def sample(self, t: float) -> ReferenceSample:
        if not np.isfinite(t) or t < 0 or t > self.duration + 1e-10:
            raise ValueError("Reference query lies outside its finite horizon")
        tau = t / self.duration
        p = np.array([1, tau, tau**2, tau**3]) @ self.coefficients
        velocity = np.array([0, 1, 2 * tau, 3 * tau**2]) @ self.coefficients / self.duration
        acceleration = np.array([0, 0, 2, 6 * tau]) @ self.coefficients / self.duration**2
        speed = float(np.linalg.norm(velocity))
        if speed <= 1e-12:
            raise ValueError("Flat reference has zero speed: heading is undefined")
        heading = float(np.arctan2(velocity[1], velocity[0]))
        return ReferenceSample(State(*p, speed, heading), velocity, acceleration)

    def arrays(self, times):
        samples = [self.sample(float(t)) for t in times]
        return (
            np.array([s.state.array() for s in samples]),
            np.array([s.velocity for s in samples]),
            np.array([s.acceleration for s in samples]),
        )

    def minimum_speed(self) -> tuple[float, float]:
        """Global minimum over [0,T], checking roots of d||dp/dtau||²/dtau.

        The squared norm is a quartic polynomial, so its stationary points
        are all roots of a cubic. This checks between logged time samples too.
        """
        velocity_coeffs = self.coefficients[1:] * np.arange(1, 4)[:, None] / self.duration
        squared_speed = sum(np.polynomial.polynomial.polymul(c, c) for c in velocity_coeffs.T)
        roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(squared_speed))
        candidates = [0.0, 1.0] + [
            float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < 1
        ]
        values = [float(np.polynomial.polynomial.polyval(t, squared_speed)) for t in candidates]
        i = int(np.argmin(values))
        return float(np.sqrt(max(0, values[i]))), candidates[i] * self.duration
