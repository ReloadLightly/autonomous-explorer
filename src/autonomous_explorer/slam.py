"""Online EKF landmark SLAM. The constructor receives no landmark coordinates or world."""

import numpy as np

from .localization import transition_jacobians
from .model import wrap_angle
from .sensors import landmark_model


def inverse_observation(rover, observation):
    """Initialize a Cartesian landmark and differentiate with respect to pose and reading."""
    distance, bearing = observation
    if not np.all(np.isfinite(observation)) or distance <= 1e-6:
        raise ValueError("A new landmark requires a positive finite range and finite bearing")
    angle = rover[3] + bearing
    c, s = np.cos(angle), np.sin(angle)
    point = np.asarray(rover)[:2] + distance * np.array([c, s])
    pose = np.array([[1, 0, 0, -distance * s], [0, 1, 0, distance * c]])
    reading = np.array([[c, -distance * s], [s, distance * c]])
    return point, pose, reading


def joint_observation(state, slot):
    """Full measurement Jacobian, including the estimated landmark's two columns."""
    start = 4 + 2 * slot
    value, rover_h = landmark_model(state[:4], state[start : start + 2])
    h = np.zeros((2, len(state)))
    h[:, :4] = rover_h
    h[:, start : start + 2] = -rover_h[:, :2]
    return value, h


class EKFSLAM:
    def __init__(self, initial, covariance, process_std, measurement_std, revisit_gap_s=5.0):
        self.state = np.array(initial, dtype=float, copy=True)
        self.covariance = np.array(covariance, dtype=float, copy=True)
        self.process = np.diag(np.asarray(process_std, dtype=float) ** 2)
        self.speed_variance = float(measurement_std[0]) ** 2
        self.measurement_covariance = np.diag(np.asarray(measurement_std[1:]) ** 2)
        if self.state.shape != (4,) or self.covariance.shape != (4, 4):
            raise ValueError("SLAM must start with only a four-state rover prior")
        if (
            not np.all(np.isfinite(self.state))
            or not np.all(np.isfinite(self.covariance))
            or not np.allclose(self.covariance, self.covariance.T)
            or np.linalg.eigvalsh(self.covariance).min() <= 0
            or np.shape(process_std) != (3,)
            or np.any(np.asarray(process_std) <= 0)
            or np.shape(measurement_std) != (3,)
            or np.any(np.asarray(measurement_std) <= 0)
            or not np.all(np.isfinite(process_std))
            or not np.all(np.isfinite(measurement_std))
            or not np.isfinite(revisit_gap_s)
            or revisit_gap_s <= 0
        ):
            raise ValueError("Require a finite positive covariance, noise scales, and revisit gap")
        self.landmark_ids = []
        self.slots = {}
        self.last_seen = {}
        self.revisit_gap_s = revisit_gap_s

    def predict(self, applied_control, dt):
        rover, f, g = transition_jacobians(self.state[:4], applied_control, dt)
        self.state[:4] = rover
        p = self.covariance
        cross = f @ p[:4, 4:]
        p[:4, :4] = f @ p[:4, :4] @ f.T + g @ self.process @ g.T
        p[:4, 4:], p[4:, :4] = cross, cross.T
        self.covariance = (p + p.T) / 2

    def add_landmark(self, landmark_id, observation):
        if landmark_id in self.slots:
            raise ValueError("Landmark already initialized")
        point, pose, reading = inverse_observation(self.state[:4], observation)
        p = self.covariance
        cross = pose @ p[:4, :]
        marginal = pose @ p[:4, :4] @ pose.T + reading @ self.measurement_covariance @ reading.T
        self.covariance = np.block([[p, cross.T], [cross, marginal]])
        self.state = np.concatenate([self.state, point])
        self.slots[landmark_id] = len(self.landmark_ids)
        self.landmark_ids.append(landmark_id)

    def _correct(self, observation, expected, h, r, angular=False):
        residual = np.asarray(observation) - expected
        if angular:
            residual[1] = wrap_angle(residual[1])
        p = self.covariance
        innovation = h @ p @ h.T + r
        gain = np.linalg.solve(innovation, h @ p).T
        self.state += gain @ residual
        remaining = np.eye(len(self.state)) - gain @ h
        self.covariance = remaining @ p @ remaining.T + gain @ r @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2
        return float(residual @ np.linalg.solve(innovation, residual))

    def correct(self, packet, time, use_reobservations=True, allow_revisits=True):
        """Augment on first sight, correct on later sightings; never reuse an initial reading.

        A revisit means an identified feature returns after a declared observation gap.
        This uses supplied feature IDs, not an implemented place-recognition front end.
        """
        if len(set(packet.landmark_ids.tolist())) != len(packet.landmark_ids):
            raise ValueError("A sensor packet may not contain duplicate landmark IDs")
        events = {key: [] for key in ["new", "updated", "revisited", "rejected", "skipped"]}
        nis = {}
        if packet.speed is not None:
            h = np.zeros((1, len(self.state)))
            h[0, 2] = 1
            self._correct([packet.speed], self.state[2:3], h, np.array([[self.speed_variance]]))
        for raw_id, value in zip(packet.landmark_ids, packet.range_bearing, strict=True):
            landmark_id = int(raw_id)
            if not np.all(np.isfinite(value)) or value[0] <= 1e-6:
                events["rejected"].append(landmark_id)
                continue
            if landmark_id not in self.slots:
                self.add_landmark(landmark_id, value)
                events["new"].append(landmark_id)
            else:
                revisited = time - self.last_seen[landmark_id] >= self.revisit_gap_s
                if not use_reobservations or (revisited and not allow_revisits):
                    events["skipped"].append(landmark_id)
                    # A denied revisit stays denied at subsequent timestamps as well.
                    continue
                expected, h = joint_observation(self.state, self.slots[landmark_id])
                nis[landmark_id] = self._correct(
                    value, expected, h, self.measurement_covariance, True
                )
                events["updated"].append(landmark_id)
                if revisited:
                    events["revisited"].append(landmark_id)
            self.last_seen[landmark_id] = float(time)
        return events, nis
