"""Belief-driven viewpoint selection, replanning, and localization-health supervision."""

from dataclasses import dataclass

import numpy as np

from .model import wrap_angle
from .planning import World, grid_search, rover_radius, shortcut_path

STATES = {"exploring": 0, "returning": 1, "braking": 2, "waiting": 3, "complete": 4, "aborted": 5}


@dataclass
class Belief:
    rover: np.ndarray
    covariance: np.ndarray
    landmarks: np.ndarray
    landmark_covariances: np.ndarray
    last_correction_s: float


def connect_plan(world, start, goal, resolution):
    """Explicit checked connector from a continuous estimate to a nearby grid node."""
    start, goal = np.asarray(start), np.asarray(goal)
    if not world.point_free(start) or not world.point_free(goal):
        return None
    if world.segment_free(start, goal):
        return np.array([start, goal])
    nearest = np.rint(start / resolution).astype(int)
    candidates = [resolution * (nearest + [dx, dy]) for dx in [-1, 0, 1] for dy in [-1, 0, 1]]
    candidates.sort(key=lambda p: (float(np.linalg.norm(p - start)), float(p[0]), float(p[1])))
    for node in candidates:
        if not world.segment_free(start, node):
            continue
        result = grid_search(world, node, goal, resolution)
        if result.status == "found":
            path = (
                result.path
                if np.linalg.norm(start - node) < 1e-9
                else np.vstack([start, result.path])
            )
            return shortcut_path(path, world)
    return None


class Mission:
    """Contains no true rover state, landmark coordinates, or outage schedule."""

    def __init__(self, navigation, policy, sensors, home, guarded=True):
        self.nav, self.policy, self.sensor = navigation, policy, sensors
        self.world = World.from_config(navigation["world"])
        self.planning_world = self.world.inflate(
            rover_radius(navigation) + policy["planning_margin_m"]
        )
        self.home = np.array(home, dtype=float)
        self.candidates = np.asarray(policy["viewpoints_xy_m"], dtype=float)
        step = policy["coverage_resolution_m"]
        x0, x1, y0, y1 = self.world.bounds
        cells = [
            [x, y]
            for x in np.arange(x0 + step / 2, x1, step)
            for y in np.arange(y0 + step / 2, y1, step)
            if self.world.point_free([x, y])
        ]
        self.cells = np.array(cells)
        self.covered = np.zeros(len(cells), dtype=bool)
        self.footprints = [self.visible_cells(p) for p in self.candidates]
        self.visited = set()
        self.state, self.resume_state = "exploring", "exploring"
        self.guarded = guarded
        self.path, self.path_index, self.goal_id = np.empty((0, 2)), 0, None
        self.events, self.decisions = [], []
        self.bad_since, self.good_observations = None, 0
        self.abort_reason = None
        self.goals_completed, self.plan_count = 0, 0

    def visible_cells(self, position):
        close = np.linalg.norm(self.cells - position, axis=1) <= self.policy["coverage_radius_m"]
        visible = np.zeros(len(self.cells), dtype=bool)
        for j in np.flatnonzero(close):
            visible[j] = self.world.segment_free(position, self.cells[j])
        return visible

    def transition(self, time, state, reason):
        if state != self.state:
            self.events.append(
                {"time_s": float(time), "from": self.state, "to": state, "reason": reason}
            )
            self.state = state

    def brake(self, rover):
        # This controller is defined at zero speed; it does not use flatness inversion.
        return np.array([-self.policy["speed_gain_per_s"] * rover[2], 0.0])

    def health(self, belief, time):
        if not np.all(np.isfinite(belief.rover)) or not np.all(np.isfinite(belief.covariance)):
            return "nonfinite_belief"
        sigma = np.sqrt(max(0, np.linalg.eigvalsh(belief.covariance[:2, :2]).max()))
        if time - belief.last_correction_s > self.policy["max_landmark_age_s"]:
            return "stale_landmarks"
        if sigma > self.policy["max_position_std_m"]:
            return "position_uncertainty"
        return None

    def select_goal(self, belief, time):
        scores = []
        for index, point in enumerate(self.candidates):
            if index in self.visited:
                continue
            path = connect_plan(
                self.planning_world, belief.rover[:2], point, self.nav["grid_resolution_m"]
            )
            if path is None:
                continue
            distance = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            gain = int(np.count_nonzero(self.footprints[index] & ~self.covered))
            information = 0.0
            for landmark, covariance in zip(
                belief.landmarks, belief.landmark_covariances, strict=True
            ):
                if np.linalg.norm(point - landmark) <= self.sensor[
                    "max_range_m"
                ] and self.world.segment_free(point, landmark):
                    information += float(np.trace(covariance))
            utility = (
                gain
                + self.policy["map_uncertainty_weight"] * information
                - self.policy["distance_weight_per_m"] * distance
            )
            scores.append(
                {
                    "id": index,
                    "gain_cells": gain,
                    "map_uncertainty": information,
                    "path_length_m": distance,
                    "utility": utility,
                    "path": path,
                }
            )
        if not scores:
            return None
        best = max(scores, key=lambda s: (s["utility"], -s["id"]))
        self.decisions.append(
            {
                "time_s": float(time),
                "selected": best["id"],
                "coverage_fraction": float(self.covered.mean()),
                "candidates": [{k: v for k, v in s.items() if k != "path"} for s in scores],
            }
        )
        return best["id"], best["path"]

    def install_plan(self, path, goal_id, time):
        self.path, self.path_index, self.goal_id = path, 1, goal_id
        self.plan_count += 1
        self.events.append(
            {
                "time_s": float(time),
                "type": "plan",
                "goal_id": goal_id,
                "mode": self.state,
                "path": path.tolist(),
            }
        )

    def command(self, belief, time, fresh_observation, pose_jump_m=0.0):
        p, rover = self.policy, belief.rover
        if fresh_observation:
            self.covered |= self.visible_cells(rover[:2])
        stopped = abs(rover[2]) <= p["stopped_speed_mps"]
        bad = self.health(belief, time) if self.guarded else None
        if self.state in {"complete", "aborted"}:
            return self.brake(rover)
        if self.state in {"exploring", "returning"} and (bad or pose_jump_m > p["replan_jump_m"]):
            self.resume_state = self.state
            self.bad_since, self.good_observations = float(time), 0
            self.path = np.empty((0, 2))
            self.transition(time, "braking", bad or "pose_correction")
        if self.state == "braking":
            if stopped:
                self.transition(time, "waiting", "estimated_speed_low")
            return self.brake(rover)
        if self.state == "waiting":
            self.good_observations = (
                self.good_observations + 1
                if fresh_observation and not bad
                else (0 if bad else self.good_observations)
            )
            if time - self.bad_since >= p["localization_timeout_s"]:
                self.abort_reason = "localization_timeout"
                self.transition(time, "aborted", self.abort_reason)
            elif self.good_observations >= p["recovery_observations"] and stopped:
                self.transition(time, self.resume_state, "localization_recovered")
            return self.brake(rover)
        if not len(self.path):
            if not stopped:
                return self.brake(rover)
            if self.state == "exploring" and (
                self.goals_completed >= p["max_exploration_goals"]
                or self.covered.mean() >= p["coverage_target_fraction"]
            ):
                self.transition(time, "returning", "exploration_budget_or_coverage")
                self.goal_id = -1
            if self.state == "returning":
                if np.linalg.norm(rover[:2] - self.home) <= p["arrival_tolerance_m"]:
                    self.transition(time, "complete", "home_reached_at_low_speed")
                    return self.brake(rover)
                path = connect_plan(
                    self.planning_world, rover[:2], self.home, self.nav["grid_resolution_m"]
                )
                choice = None if path is None else (-1, path)
            elif self.goal_id is not None and self.goal_id not in self.visited:
                path = connect_plan(
                    self.planning_world,
                    rover[:2],
                    self.candidates[self.goal_id],
                    self.nav["grid_resolution_m"],
                )
                choice = None if path is None else (self.goal_id, path)
            else:
                choice = self.select_goal(belief, time)
            if choice is None:
                self.abort_reason = "no_reachable_goal"
                self.transition(time, "aborted", self.abort_reason)
                return self.brake(rover)
            self.install_plan(choice[1], choice[0], time)
        target = self.path[self.path_index]
        distance = float(np.linalg.norm(target - rover[:2]))
        if distance <= p["arrival_tolerance_m"]:
            if not stopped:
                return self.brake(rover)
            if self.path_index + 1 < len(self.path):
                self.path_index += 1
            else:
                if self.goal_id == -1:
                    self.transition(time, "complete", "home_reached_at_low_speed")
                else:
                    self.visited.add(self.goal_id)
                    self.goals_completed += 1
                    self.events.append(
                        {
                            "time_s": float(time),
                            "type": "viewpoint_reached",
                            "goal_id": self.goal_id,
                        }
                    )
                    self.goal_id, self.path = None, np.empty((0, 2))
            return self.brake(rover)
        error = float(wrap_angle(np.arctan2(*(target - rover[:2])[::-1]) - rover[3]))
        desired_speed = min(p["cruise_speed_mps"], p["distance_gain_per_s"] * distance)
        if abs(error) > p["turn_threshold_rad"]:
            desired_speed = 0.0
        return np.array(
            [p["speed_gain_per_s"] * (desired_speed - rover[2]), p["heading_gain_per_s"] * error]
        )
