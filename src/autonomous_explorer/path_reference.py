"""Collision-certified rounding of a polyline into a positive-speed C1 timed reference.

Straight segments meet quadratic Bezier corner arcs with matching Cartesian velocity.
Acceleration is piecewise bounded and can jump at joins; no jerk constraint is claimed.
"""

import numpy as np

from .control import invert_acceleration
from .model import State
from .reference import ReferenceSample


class RoundedReference:
    def __init__(self, controls, durations):
        self.controls = np.asarray(
            controls, dtype=float
        )  # (segments, 3, xy); lines are degenerate quadratics
        self.durations = np.asarray(durations, dtype=float)
        if len(self.controls) == 0 or np.any(self.durations <= 0):
            raise ValueError("Positive-duration segments are required")
        self.boundaries = np.concatenate([[0.0], np.cumsum(self.durations)])
        self.duration = float(self.boundaries[-1])

    def sample(self, t):
        if not np.isfinite(t) or t < 0 or t > self.duration + 1e-9:
            raise ValueError("Reference query outside finite horizon")
        k = min(int(np.searchsorted(self.boundaries, t, side="right") - 1), len(self.durations) - 1)
        u = np.clip((t - self.boundaries[k]) / self.durations[k], 0, 1)
        p0, p1, p2 = self.controls[k]
        position = (1 - u) ** 2 * p0 + 2 * u * (1 - u) * p1 + u**2 * p2
        velocity = 2 * ((1 - u) * (p1 - p0) + u * (p2 - p1)) / self.durations[k]
        acceleration = 2 * (p2 - 2 * p1 + p0) / self.durations[k] ** 2
        speed = float(np.linalg.norm(velocity))
        if speed <= 1e-10:
            raise ValueError("Rounded reference reached zero speed")
        return ReferenceSample(
            State(*position, speed, float(np.arctan2(velocity[1], velocity[0]))),
            velocity,
            acceleration,
        )

    def arrays(self, times):
        samples = [self.sample(float(t)) for t in times]
        return (
            np.array([s.state.array() for s in samples]),
            np.array([s.velocity for s in samples]),
            np.array([s.acceleration for s in samples]),
        )

    def bounds(self):
        """Exact segmentwise extrema, including between samples and both sides of joins.

        Velocity is affine. Speed minimum is the projection onto this line in
        velocity space; speed maximum is at an endpoint. The velocity/acceleration
        cross product is constant, so |omega| peaks at minimum speed. Tangential
        acceleration is monotone, so its absolute maximum is at an endpoint.
        """
        minimum, maximum, max_a, max_omega = np.inf, 0.0, 0.0, 0.0
        for points, duration in zip(self.controls, self.durations, strict=True):
            v0, v1 = 2 * (points[1] - points[0]) / duration, 2 * (points[2] - points[1]) / duration
            change = v1 - v0
            fraction = (
                np.clip(-np.dot(v0, change) / np.dot(change, change), 0, 1)
                if np.dot(change, change) > 1e-24
                else 0
            )
            vmin = np.linalg.norm(v0 + fraction * change)
            minimum, maximum = (
                min(minimum, vmin),
                max(maximum, np.linalg.norm(v0), np.linalg.norm(v1)),
            )
            acceleration = change / duration
            if vmin <= 1e-10:
                raise ValueError("Zero-speed or reversing corner is not supported")
            max_a = max(
                max_a,
                abs(np.dot(v0, acceleration)) / np.linalg.norm(v0),
                abs(np.dot(v1, acceleration)) / np.linalg.norm(v1),
            )
            cross = v0[0] * acceleration[1] - v0[1] * acceleration[0]
            max_omega = max(max_omega, abs(cross) / vmin**2)
        return {
            "min_speed_mps": float(minimum),
            "max_speed_mps": float(maximum),
            "max_abs_acceleration_mps2": float(max_a),
            "max_abs_yaw_rate_radps": float(max_omega),
        }


def build_rounded_reference(
    path, certificate_world, cruise_speed, max_acceleration, max_yaw_rate, dt
):
    if min(cruise_speed, max_acceleration, max_yaw_rate, dt) <= 0:
        raise ValueError("Positive speed, control bounds and timestep required")
    path = np.asarray(path, dtype=float)
    vectors = np.diff(path, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    if np.any(lengths <= 1e-8):
        raise ValueError("Duplicate consecutive waypoints")
    directions = vectors / lengths[:, None]
    arcs, rounds = [], []
    for i in range(1, len(path) - 1):
        radius = 0.45 * min(lengths[i - 1], lengths[i])
        for _reductions in range(24):
            control = np.array(
                [path[i] - radius * directions[i - 1], path[i], path[i] + radius * directions[i]]
            )
            if certificate_world.hull_box_free(control):
                break
            radius /= 2
        else:
            raise ValueError("Could not certify a nonzero corner-rounding hull")
        arcs.append((control, 2 * radius / cruise_speed))
        rounds.append(
            {"waypoint_index": i, "tangent_distance_m": float(radius), "halvings": _reductions}
        )
    controls, durations, current = [], [], path[0]

    def line(end):
        nonlocal current
        distance = np.linalg.norm(end - current)
        if distance > 1e-10:
            if not certificate_world.segment_free(current, end):
                raise ValueError("Uncertified straight segment")
            controls.append([current, (current + end) / 2, end])
            durations.append(distance / cruise_speed)
        current = end

    for arc, duration in arcs:
        line(arc[0])
        controls.append(arc)
        durations.append(duration)
        current = arc[-1]
    line(path[-1])
    reference = RoundedReference(controls, durations)
    before = reference.bounds()
    scale = max(
        1.0,
        np.sqrt(before["max_abs_acceleration_mps2"] / max_acceleration),
        before["max_abs_yaw_rate_radps"] / max_yaw_rate,
    )
    # Fit an exact integer control horizon without speeding anything up.
    target_duration = np.ceil(reference.duration * scale / dt - 1e-10) * dt
    scale = target_duration / reference.duration
    reference = RoundedReference(controls, np.asarray(durations) * scale)
    metadata = {
        "corner_rounding": rounds,
        "uniform_time_scale": float(scale),
        "unscaled_bounds": before,
        "bounds": reference.bounds(),
        "continuity": "C1 position; piecewise acceleration with jumps at segment joins",
        "collision_certificate": "Each quadratic control-point AABB or straight segment is free",
    }
    # Fail explicitly rather than hiding a low-speed singularity.
    for sample_time in reference.boundaries:
        sample = reference.sample(float(sample_time))
        invert_acceleration(sample.state, sample.acceleration)
    return reference, metadata
