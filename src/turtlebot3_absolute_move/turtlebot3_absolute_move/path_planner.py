"""
Path Planner — Dijkstra pathfinding on SLAM occupancy grid.

Takes the latest OccupancyGrid from SLAM Toolbox, inflates obstacles
by the robot radius + safety margin, and runs Dijkstra to find the shortest
collision-free path from the robot's current position to a goal.

The path is simplified to remove redundant collinear waypoints, then
returned as a list of (x, y) coordinates in the map/odom frame.

Algorithm:
  1. Convert start/goal from world coords to grid coords
  2. Inflate occupied cells by robot_radius + safety_margin
  3. Dijkstra with 8-connected neighbours
     - g(n) = cumulative cost (diagonal=√2, cardinal=1)
     - h(n) = 0 (no heuristic — guarantees finding a path if one exists)
     - f(n) = g(n)
  4. Backtrace from goal to start
  5. Simplify path (remove collinear intermediate points)
  6. Enforce minimum waypoint spacing
  7. Convert back to world coordinates
"""

import math
import heapq

import numpy as np

from nav_msgs.msg import OccupancyGrid


# 8-connected moves: (delta_row, delta_col, cost)
_MOVES_8 = [
    (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
    (0, -1, 1.0),                          (0, 1, 1.0),
    (1, -1, math.sqrt(2)),  (1, 0, 1.0),   (1, 1, math.sqrt(2)),
]

# Occupancy threshold — cells with value >= this are considered occupied
_OCCUPIED_THRESHOLD = 50


class PathPlanner:
    """Dijkstra path planner on an OccupancyGrid with obstacle inflation."""

    def __init__(self, inflation_radius=0.18, waypoint_spacing=0.3,
                 unknown_as_free=False, simplify=True):
        """
        Args:
            inflation_radius: Distance in metres to inflate obstacles.
                Should be >= robot radius + safety margin.
            waypoint_spacing: Minimum spacing in metres between output
                waypoints.  Path is subsampled to this resolution.
            unknown_as_free: If True, treat unknown cells as free
                (useful during exploration when map is incomplete).
            simplify: If True, remove collinear intermediate waypoints.
        """
        self.inflation_radius = inflation_radius
        self.waypoint_spacing = waypoint_spacing
        self.unknown_as_free = unknown_as_free
        self.simplify = simplify

        # Cache the inflated grid to avoid re-inflating on every call
        # if the map hasn't changed.
        self._cached_grid_stamp = None
        self._inflated = None
        self._resolution = 0.0
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._width = 0
        self._height = 0

    def plan(self, grid_msg: OccupancyGrid,
             start_x: float, start_y: float,
             goal_x: float, goal_y: float):
        """Plan a path from start to goal on the occupancy grid.

        Args:
            grid_msg: Latest OccupancyGrid from SLAM Toolbox.
            start_x, start_y: Start position in map/odom frame (metres).
            goal_x, goal_y: Goal position in map/odom frame (metres).

        Returns:
            List of (x, y) waypoints in map frame, or None if no path.
            The first waypoint is near the start, the last is the goal.
        """
        self._update_inflated_grid(grid_msg)

        # Convert world → grid
        sr, sc = self._world_to_grid(start_x, start_y)
        gr, gc = self._world_to_grid(goal_x, goal_y)

        # Clamp to grid bounds
        sr = max(0, min(self._height - 1, sr))
        sc = max(0, min(self._width - 1, sc))
        gr = max(0, min(self._height - 1, gr))
        gc = max(0, min(self._width - 1, gc))

        # Check if goal is inside an inflated obstacle
        if self._inflated[gr, gc]:
            # Try to find the nearest free cell to the goal
            free_cell = self._nearest_free_cell(gr, gc)
            if free_cell is None:
                return None
            gr, gc = free_cell

        # Check if start is inside an inflated obstacle
        if self._inflated[sr, sc]:
            free_cell = self._nearest_free_cell(sr, sc)
            if free_cell is None:
                return None
            sr, sc = free_cell

        # Run Dijkstra
        path_grid = self._dijkstra(sr, sc, gr, gc)
        if path_grid is None:
            return None

        # Convert grid path → world coordinates
        path_world = [self._grid_to_world(r, c) for r, c in path_grid]

        # Simplify (remove collinear intermediate waypoints)
        if self.simplify and len(path_world) > 2:
            path_world = self._simplify_path(path_world)

        # Enforce minimum waypoint spacing
        if self.waypoint_spacing > 0 and len(path_world) > 2:
            path_world = self._enforce_spacing(path_world)

        return path_world

    def is_goal_reachable(self, grid_msg: OccupancyGrid,
                          goal_x: float, goal_y: float):
        """Check if a goal position is in free space (not inside an obstacle).

        Returns True if the goal is reachable, False if it's inside
        an inflated obstacle.
        """
        self._update_inflated_grid(grid_msg)
        gr, gc = self._world_to_grid(goal_x, goal_y)
        if not (0 <= gr < self._height and 0 <= gc < self._width):
            return False
        return not self._inflated[gr, gc]

    # ------------------------------------------------------------------
    # Grid inflation
    # ------------------------------------------------------------------

    def _update_inflated_grid(self, grid_msg: OccupancyGrid):
        """Rebuild the inflated obstacle grid if the map has changed."""
        stamp = (grid_msg.info.width, grid_msg.info.height,
                 grid_msg.info.resolution,
                 grid_msg.info.origin.position.x,
                 grid_msg.info.origin.position.y,
                 hash(tuple(grid_msg.data[:100])))  # quick hash

        if stamp == self._cached_grid_stamp:
            return  # map unchanged

        self._width = grid_msg.info.width
        self._height = grid_msg.info.height
        self._resolution = grid_msg.info.resolution
        self._origin_x = grid_msg.info.origin.position.x
        self._origin_y = grid_msg.info.origin.position.y

        data = np.array(grid_msg.data, dtype=np.int8).reshape(
            (self._height, self._width))

        # Build the raw obstacle mask
        obstacle_mask = (data >= _OCCUPIED_THRESHOLD)
        if not self.unknown_as_free:
            obstacle_mask |= (data == -1)

        # Inflate obstacles
        inflation_cells = max(1, int(
            math.ceil(self.inflation_radius / self._resolution)))
        self._inflated = self._inflate(obstacle_mask, inflation_cells)
        self._cached_grid_stamp = stamp

    def _inflate(self, obstacle_mask, radius_cells):
        """Inflate obstacles by radius_cells using distance transform.

        Returns a boolean mask where True = inflated obstacle (blocked).
        """
        from scipy import ndimage

        # Distance from each cell to the nearest obstacle
        # (distance_transform_edt gives distance from False cells to
        #  nearest True cell, so we invert: distance from free to obstacle)
        # Actually, we want: for each free cell, distance to nearest obstacle.
        # If dist < inflation_radius → blocked.
        try:
            dist = ndimage.distance_transform_edt(~obstacle_mask)
            return dist < radius_cells
        except Exception:
            # Fallback: manual dilation (slower but no scipy dependency)
            return self._inflate_manual(obstacle_mask, radius_cells)

    @staticmethod
    def _inflate_manual(obstacle_mask, radius_cells):
        """Manual obstacle inflation without scipy."""
        height, width = obstacle_mask.shape
        inflated = obstacle_mask.copy()

        # Create circular kernel
        kernel_size = 2 * radius_cells + 1
        kernel = np.zeros((kernel_size, kernel_size), dtype=bool)
        for r in range(kernel_size):
            for c in range(kernel_size):
                dr = r - radius_cells
                dc = c - radius_cells
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    kernel[r, c] = True

        # Convolve manually using numpy
        obs_rows, obs_cols = np.where(obstacle_mask)
        for r, c in zip(obs_rows, obs_cols):
            r_min = max(0, r - radius_cells)
            r_max = min(height, r + radius_cells + 1)
            c_min = max(0, c - radius_cells)
            c_max = min(width, c + radius_cells + 1)

            kr_min = r_min - (r - radius_cells)
            kr_max = kernel_size - ((r + radius_cells + 1) - r_max)
            kc_min = c_min - (c - radius_cells)
            kc_max = kernel_size - ((c + radius_cells + 1) - c_max)

            inflated[r_min:r_max, c_min:c_max] |= (
                kernel[kr_min:kr_max, kc_min:kc_max])

        return inflated

    # ------------------------------------------------------------------
    # Dijkstra search
    # ------------------------------------------------------------------

    def _dijkstra(self, sr, sc, gr, gc):
        """Dijkstra search on the inflated grid (no heuristic).

        Unlike A*, Dijkstra uses h=0 so it explores uniformly outward.
        This guarantees finding a path if one exists, which is critical
        for frontier exploration on partially-known SLAM maps.

        Returns list of (row, col) from start to goal, or None.
        """
        if self._inflated[sr, sc] or self._inflated[gr, gc]:
            return None

        height, width = self._height, self._width
        inflated = self._inflated

        # Priority queue: (g_score, counter, row, col)
        counter = 0
        open_set = [(0.0, counter, sr, sc)]
        came_from = {}
        g_score = np.full((height, width), np.inf)
        g_score[sr, sc] = 0.0
        closed = np.zeros((height, width), dtype=bool)

        while open_set:
            g, _, cr, cc = heapq.heappop(open_set)

            if cr == gr and cc == gc:
                # Reconstruct path
                path = [(gr, gc)]
                r, c = gr, gc
                while (r, c) in came_from:
                    r, c = came_from[(r, c)]
                    path.append((r, c))
                path.reverse()
                return path

            if closed[cr, cc]:
                continue
            closed[cr, cc] = True

            for dr, dc, move_cost in _MOVES_8:
                nr, nc = cr + dr, cc + dc
                if not (0 <= nr < height and 0 <= nc < width):
                    continue
                if closed[nr, nc] or inflated[nr, nc]:
                    continue

                tentative_g = g_score[cr, cc] + move_cost
                if tentative_g < g_score[nr, nc]:
                    g_score[nr, nc] = tentative_g
                    came_from[(nr, nc)] = (cr, cc)
                    counter += 1
                    heapq.heappush(open_set, (tentative_g, counter, nr, nc))

        return None  # no path found

    # ------------------------------------------------------------------
    # Nearest free cell (for goals inside obstacles)
    # ------------------------------------------------------------------

    def _nearest_free_cell(self, row, col, max_radius=50):
        """BFS outward from (row, col) to find the nearest non-inflated cell."""
        from collections import deque

        visited = set()
        queue = deque()
        queue.append((row, col))
        visited.add((row, col))

        while queue:
            r, c = queue.popleft()
            if not self._inflated[r, c]:
                return (r, c)

            if math.hypot(r - row, c - col) > max_radius:
                continue

            for dr, dc, _ in _MOVES_8:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self._height and 0 <= nc < self._width
                        and (nr, nc) not in visited):
                    visited.add((nr, nc))
                    queue.append((nr, nc))

        return None

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def _world_to_grid(self, wx, wy):
        """Convert world (metres) to grid (row, col)."""
        col = int((wx - self._origin_x) / self._resolution)
        row = int((wy - self._origin_y) / self._resolution)
        return row, col

    def _grid_to_world(self, row, col):
        """Convert grid (row, col) to world (metres) — cell centre."""
        wx = self._origin_x + (col + 0.5) * self._resolution
        wy = self._origin_y + (row + 0.5) * self._resolution
        return (wx, wy)

    # ------------------------------------------------------------------
    # Path simplification
    # ------------------------------------------------------------------

    def _simplify_path(self, path):
        """Remove collinear intermediate waypoints.

        Uses a line-of-sight check on the inflated grid: if two non-
        adjacent waypoints have a clear line between them, all
        intermediate waypoints are removed.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Find the farthest point from i with a clear line of sight
            farthest = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self._line_of_sight(path[i], path[j]):
                    farthest = j
                    break
            simplified.append(path[farthest])
            i = farthest

        return simplified

    def _line_of_sight(self, p1, p2):
        """Check if the straight line between two world points is free.

        Uses Bresenham's line algorithm on the inflated grid.
        """
        r1, c1 = self._world_to_grid(p1[0], p1[1])
        r2, c2 = self._world_to_grid(p2[0], p2[1])

        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1
        sc = 1 if c1 < c2 else -1

        err = dr - dc
        r, c = r1, c1

        while True:
            if not (0 <= r < self._height and 0 <= c < self._width):
                return False
            if self._inflated[r, c]:
                return False
            if r == r2 and c == c2:
                break

            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

        return True

    def _enforce_spacing(self, path):
        """Ensure minimum spacing between consecutive waypoints.

        Removes waypoints that are closer than waypoint_spacing to
        the previous kept waypoint, keeping the first and last.
        """
        if len(path) <= 2:
            return path

        result = [path[0]]
        for i in range(1, len(path) - 1):
            dist = math.hypot(
                path[i][0] - result[-1][0],
                path[i][1] - result[-1][1])
            if dist >= self.waypoint_spacing:
                result.append(path[i])
        result.append(path[-1])
        return result
