"""Synthetic speed and identified landmark observations; truth stays in this layer."""

from dataclasses import dataclass

import numpy as np

from .model import wrap_angle


@dataclass(frozen=True)
class Observation:
    speed: float | None
    landmark_ids: np.ndarray
    range_bearing: np.ndarray


def landmark_model(state, landmark):
    """Range/bearing and analytic Jacobian in (x, y, v, theta) order."""
    dx, dy = np.asarray(landmark) - np.asarray(state)[:2]
    squared = dx * dx + dy * dy
    if squared <= 1e-12:
        raise ValueError("Landmark range is singular at the rover position")
    distance = np.sqrt(squared)
    value = np.array([distance, wrap_angle(np.arctan2(dy, dx) - state[3])])
    jacobian = np.array(
        [[-dx / distance, -dy / distance, 0, 0], [dy / squared, -dx / squared, 0, -1]]
    )
    return value, jacobian


def sample_observation(state, index, time, config, world, speed_noise, landmark_noise, outage):
    """Full 360° field of view, finite range, opaque obstacles, known perfect IDs.

    Noise arrays are drawn before simulation, indexed even for invisible landmarks.
    Missing observations never consume a different random sequence.
    """
    speed = (
        float(state[2] + speed_noise[index]) if index % config["speed_every_steps"] == 0 else None
    )
    ids, readings = [], []
    blocked = outage is not None and outage[0] <= time < outage[1]
    if index % config["landmark_every_steps"] == 0 and not blocked:
        for j, landmark in enumerate(config["landmarks_xy_m"]):
            distance = np.linalg.norm(np.asarray(landmark) - state[:2])
            if 1e-6 < distance <= config["max_range_m"] and world.segment_free(state[:2], landmark):
                value, _ = landmark_model(state, landmark)
                value += landmark_noise[index, j]
                value[1] = wrap_angle(value[1])
                ids.append(j)
                readings.append(value)
    return Observation(speed, np.array(ids, dtype=int), np.array(readings).reshape(-1, 2))
