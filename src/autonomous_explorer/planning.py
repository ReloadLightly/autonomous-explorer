"""Known rectangular worlds, conservative rover geometry, and deterministic grid search.

World bounds: (xmin, xmax, ymin, ymax); obstacle boxes: (xmin, ymin, xmax, ymax).
All distances are metres. Planning knows the entire static world in advance.
"""

import heapq
from dataclasses import dataclass

import numpy as np


def segment_intersects_box(start, end, box):
    """Closed segment/AABB intersection by intersecting parameter slabs, t in [0,1]."""
    lower, upper = 0.0, 1.0
    start, delta = np.asarray(start), np.asarray(end) - start
    for axis in range(2):
        lo, hi = box[axis], box[axis + 2]
        if abs(delta[axis]) < 1e-14:
            if start[axis] < lo or start[axis] > hi:
                return False
        else:
            a, b = sorted([(lo - start[axis]) / delta[axis], (hi - start[axis]) / delta[axis]])
            lower, upper = max(lower, a), min(upper, b)
            if lower > upper:
                return False
    return True


@dataclass(frozen=True)
class World:
    bounds: tuple
    obstacles: tuple

    @classmethod
    def from_config(cls, config):
        return cls(tuple(config["bounds_m"]), tuple(map(tuple, config["obstacles_xyxy_m"])))

    def inflate(self, distance):
        """Conservative box offset, with workspace walls moved inward by the same distance.

        AABB expansion contains the exact disk Minkowski sum (including its rounded corners).
        It can reject feasible configurations; it cannot under-inflate the disk footprint.
        """
        if distance < 0:
            raise ValueError("Inflation must be nonnegative")
        xmin, xmax, ymin, ymax = self.bounds
        boxes = [
            (a - distance, b - distance, c + distance, d + distance)
            for a, b, c, d in self.obstacles
        ]
        return World(
            (xmin + distance, xmax - distance, ymin + distance, ymax - distance), tuple(boxes)
        )

    def point_free(self, point):
        x, y = point
        xmin, xmax, ymin, ymax = self.bounds
        return (
            xmin < x < xmax
            and ymin < y < ymax
            and not any(a <= x <= c and b <= y <= d for a, b, c, d in self.obstacles)
        )

    def segment_free(self, start, end):
        return (
            self.point_free(start)
            and self.point_free(end)
            and not any(segment_intersects_box(start, end, box) for box in self.obstacles)
        )

    def hull_box_free(self, control_points):
        """Sufficient continuous curve certificate: the entire control-point AABB is free."""
        lower, upper = np.min(control_points, axis=0), np.max(control_points, axis=0)
        xmin, xmax, ymin, ymax = self.bounds
        if not (xmin < lower[0] <= upper[0] < xmax and ymin < lower[1] <= upper[1] < ymax):
            return False
        return not any(
            lower[0] <= c and upper[0] >= a and lower[1] <= d and upper[1] >= b
            for a, b, c, d in self.obstacles
        )

    def clearance(self, points, radius=0.0):
        """Signed disk clearance: min distance to obstacles/walls minus enclosing radius.

        Inside an obstacle, signed box distance is negative. This is conservative
        for the actual oriented rover because its geometry fits inside the disk.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        xmin, xmax, ymin, ymax = self.bounds
        result = np.min(
            np.column_stack(
                [points[:, 0] - xmin, xmax - points[:, 0], points[:, 1] - ymin, ymax - points[:, 1]]
            ),
            axis=1,
        )
        for a, b, c, d in self.obstacles:
            center, half = (
                np.array([(a + c) / 2, (b + d) / 2]),
                np.array([(c - a) / 2, (d - b) / 2]),
            )
            q = np.abs(points - center) - half
            signed = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0)
            result = np.minimum(result, signed)
        return result - radius


def rover_radius(config):
    """Disk enclosing the 0.48 x 0.45 m full body/wheel bounding rectangle, all headings."""
    return float(np.hypot(config["body_length_m"] / 2, config["outer_wheel_width_m"] / 2))


@dataclass
class SearchResult:
    status: str
    path: np.ndarray
    expanded: np.ndarray
    occupancy: np.ndarray
    xs: np.ndarray
    ys: np.ndarray
    cost: float | None


def grid_search(world, start, goal, resolution, algorithm="astar", origin=(0.0, 0.0)):
    """8-connected A* or Dijkstra with Euclidean edge costs and deterministic tie breaking.

    Endpoints must lie on the lattice; they are never silently snapped. Nodes use
    integer indices. Diagonals require both adjacent axial nodes free, and EVERY
    edge is checked as a complete segment against the continuous obstacle boxes.
    """
    if resolution <= 0 or algorithm not in {"astar", "dijkstra"}:
        raise ValueError("Positive resolution and astar/dijkstra algorithm required")
    xmin, xmax, ymin, ymax = world.bounds
    origin = np.asarray(origin)
    ix = np.arange(
        np.floor((xmin - origin[0]) / resolution),
        np.ceil((xmax - origin[0]) / resolution) + 1,
        dtype=int,
    )
    iy = np.arange(
        np.floor((ymin - origin[1]) / resolution),
        np.ceil((ymax - origin[1]) / resolution) + 1,
        dtype=int,
    )
    xs, ys = origin[0] + ix * resolution, origin[1] + iy * resolution
    occupancy = np.array([[not world.point_free((x, y)) for x in xs] for y in ys])

    def key(point):
        floating = (np.asarray(point) - origin) / resolution
        nearest = np.rint(floating).astype(int)
        if not np.allclose(floating, nearest, atol=1e-9, rtol=0):
            raise ValueError("Start and goal must lie on the declared grid; no implicit snapping")
        return tuple(nearest)

    first, target = key(start), key(goal)

    def free(node):
        i, j = node[0] - ix[0], node[1] - iy[0]
        return 0 <= i < len(xs) and 0 <= j < len(ys) and not occupancy[j, i]

    def point(node):
        return origin + np.asarray(node) * resolution

    def heuristic(node):
        return float(np.linalg.norm(point(node) - goal)) if algorithm == "astar" else 0.0

    def result(status, path, expanded, cost=None):
        return SearchResult(
            status,
            np.asarray(path).reshape(-1, 2),
            np.asarray(expanded).reshape(-1, 2),
            occupancy,
            xs,
            ys,
            cost,
        )

    if not free(first) or not free(target):
        return result("invalid_endpoint", [], [])
    frontier = [(heuristic(first), 0.0, first)]
    best, parent, closed, expanded = {first: 0.0}, {}, set(), []
    while frontier:
        _, cost, node = heapq.heappop(frontier)
        if node in closed or cost > best[node] + 1e-12:
            continue
        closed.add(node)
        expanded.append(point(node))
        if node == target:
            path = [point(node)]
            while node != first:
                node = parent[node]
                path.append(point(node))
            return result("found", path[::-1], expanded, float(cost))
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
            neighbor = (node[0] + dx, node[1] + dy)
            if not free(neighbor) or neighbor in closed:
                continue
            if (
                dx
                and dy
                and (not free((node[0] + dx, node[1])) or not free((node[0], node[1] + dy)))
            ):
                continue
            if not world.segment_free(point(node), point(neighbor)):
                continue
            candidate = cost + resolution * np.hypot(dx, dy)
            if candidate < best.get(neighbor, np.inf) - 1e-12:
                best[neighbor], parent[neighbor] = candidate, node
                heapq.heappush(frontier, (candidate + heuristic(neighbor), candidate, neighbor))
    return result("unreachable_on_grid", [], expanded)


def shortcut_path(path, world):
    """Greedily connect to the farthest later visible node; validate entire segments."""
    path = np.asarray(path)
    if len(path) < 2:
        raise ValueError("A nontrivial successful path is required")
    selected, i = [path[0]], 0
    while i < len(path) - 1:
        for j in range(len(path) - 1, i, -1):
            if world.segment_free(path[i], path[j]):
                selected.append(path[j])
                i = j
                break
        else:
            raise ValueError("Input path contains an invalid edge")
    return np.array(selected)
