"""Four-state EKF; consumes applied inputs and sensor packets, never simulator truth."""

import numpy as np

from .model import derivative, wrap_angle
from .sensors import landmark_model


def transition_jacobians(state, control, dt):
    """Differentiate the actual RK4 map by propagating stage sensitivities.

    F = d f_RK4 / d state. G = d f_RK4 / d held errors (a, speed, yaw rate).
    The process deviations are independent held samples, not diffusion densities.
    """
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")

    def stage(x, dx, dw):
        _, _, v, theta = x
        c, s = np.cos(theta), np.sin(theta)
        a = np.zeros((4, 4))
        a[0, 2], a[0, 3] = c, -v * s
        a[1, 2], a[1, 3] = s, v * c
        b = np.array([[0, c, 0], [0, s, 0], [1, 0, 0], [0, 0, 1]])
        return derivative(x, control), a @ dx, a @ dw + b

    x = np.asarray(state, dtype=float)
    identity, zero = np.eye(4), np.zeros((4, 3))
    k1, f1, g1 = stage(x, identity, zero)
    k2, f2, g2 = stage(x + dt * k1 / 2, identity + dt * f1 / 2, dt * g1 / 2)
    k3, f3, g3 = stage(x + dt * k2 / 2, identity + dt * f2 / 2, dt * g2 / 2)
    k4, f4, g4 = stage(x + dt * k3, identity + dt * f3, dt * g3)
    return (
        x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6,
        identity + dt * (f1 + 2 * f2 + 2 * f3 + f4) / 6,
        dt * (g1 + 2 * g2 + 2 * g3 + g4) / 6,
    )


class ExtendedKalmanFilter:
    def __init__(self, initial, covariance, process_std, sensors):
        self.state = np.array(initial, dtype=float, copy=True)
        self.covariance = np.array(covariance, dtype=float, copy=True)
        self.process = np.diag(np.asarray(process_std, dtype=float) ** 2)
        self.sensors = sensors

    def predict(self, applied_control, dt):
        self.state, f, g = transition_jacobians(self.state, applied_control, dt)
        self.covariance = f @ self.covariance @ f.T + g @ self.process @ g.T
        self.covariance = (self.covariance + self.covariance.T) / 2

    def _correct(self, observation, expected, h, r, angular_indices=()):
        residual = np.asarray(observation) - expected
        for index in angular_indices:
            residual[index] = wrap_angle(residual[index])
        p = self.covariance
        innovation = h @ p @ h.T + r
        gain = np.linalg.solve(innovation, h @ p).T
        self.state += gain @ residual
        # Joseph form preserves PSD up to floating point error; no covariance clipping.
        remaining = np.eye(4) - gain @ h
        self.covariance = remaining @ p @ remaining.T + gain @ r @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2
        return float(residual @ np.linalg.solve(innovation, residual))

    def correct(self, packet):
        """Return NIS diagnostics (speed scalar and sequential 2D landmark updates)."""
        speed_nis = None
        if packet.speed is not None:
            speed_nis = self._correct(
                [packet.speed],
                self.state[2:3],
                np.array([[0.0, 0, 1, 0]]),
                np.array([[self.sensors["speed_std_mps"] ** 2]]),
            )
        landmark_nis = []
        r = np.diag([self.sensors["range_std_m"] ** 2, self.sensors["bearing_std_rad"] ** 2])
        for index, value in zip(packet.landmark_ids, packet.range_bearing, strict=True):
            expected, h = landmark_model(self.state, self.sensors["landmarks_xy_m"][index])
            landmark_nis.append(self._correct(value, expected, h, r, (1,)))
        return speed_nis, landmark_nis
