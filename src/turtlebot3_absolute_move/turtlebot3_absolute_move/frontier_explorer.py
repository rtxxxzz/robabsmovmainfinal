"""
Frontier Explorer — Autonomous frontier-based exploration.

Detects frontier cells (free cells adjacent to unknown space) in the
SLAM Toolbox occupancy grid, clusters them, and picks the best target
for the robot to drive toward.  Used by the pipeline orchestrator to
autonomously build a complete map of the environment.

Algorithm:
  1. Receive /map (OccupancyGrid from SLAM Toolbox)
  2. For each free cell (value == 0):
       Check 8-connected neighbours
       If any neighbour is unknown (value == -1), mark as frontier
  3. Cluster frontier cells using BFS flood-fill
  4. Filter clusters smaller than min_frontier_size
  5. Score each cluster: size × (1 / (1 + distance_to_robot))
  6. Pick the best-scoring frontier → centroid as exploration goal
"""

import math
import collections

import numpy as np

from nav_msgs.msg import OccupancyGrid


# 8-connected neighbour offsets
_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1),
               (0, -1),           (0, 1),
               (1, -1),  (1, 0),  (1, 1)]


class FrontierExplorer:
    """Stateless frontier detection and scoring on an OccupancyGrid."""

    def __init__(self, min_frontier_size=5, cost_weight=0.5):
        """
        Args:
            min_frontier_size: Minimum number of cells in a frontier
                cluster to be considered valid.
            cost_weight: Weighting factor for distance vs. size.
                Higher values favour closer frontiers over larger ones.
        """
        self.min_frontier_size = min_frontier_size
        self.cost_weight = cost_weight

    def find_best_frontier(self, grid_msg: OccupancyGrid,
                           robot_x: float, robot_y: float,
                           blacklist=None, blacklist_radius=0.3):
        """Find the best frontier centroid to explore next.

        Args:
            grid_msg: The latest OccupancyGrid from SLAM Toolbox.
            robot_x: Robot X position in the map/odom frame (metres).
            robot_y: Robot Y position in the map/odom frame (metres).
            blacklist: Optional set of (x, y) tuples to skip.
            blacklist_radius: Distance tolerance for blacklist matching (m).

        Returns:
            (goal_x, goal_y) in metres (map frame), or None if no
            valid frontiers remain.
        """
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        if width == 0 or height == 0:
            return None

        # Convert flat data to 2-D numpy array (row=y, col=x)
        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))

        # --- Step 1: detect frontier cells ---
        frontier_mask = self._detect_frontiers(data, width, height)

        # --- Step 2: cluster frontier cells via BFS ---
        clusters = self._cluster_frontiers(frontier_mask, width, height)

        if not clusters:
            return None

        # --- Step 3: score and pick the best cluster ---
        best_score = -1.0
        best_centroid = None

        for cluster in clusters:
            # Centroid in grid coordinates
            cy = sum(r for r, c in cluster) / len(cluster)
            cx = sum(c for r, c in cluster) / len(cluster)

            # Convert to world (map) coordinates
            world_x = origin_x + (cx + 0.5) * resolution
            world_y = origin_y + (cy + 0.5) * resolution

            # Skip blacklisted frontiers
            if blacklist:
                is_blacklisted = False
                for bx, by in blacklist:
                    if math.hypot(world_x - bx, world_y - by) < blacklist_radius:
                        is_blacklisted = True
                        break
                if is_blacklisted:
                    continue

            dist = math.hypot(world_x - robot_x, world_y - robot_y)

            # Score: larger frontiers are better, closer is better
            size_score = len(cluster)
            dist_score = 1.0 / (1.0 + dist * self.cost_weight)
            score = size_score * dist_score

            if score > best_score:
                best_score = score
                best_centroid = (world_x, world_y)

        return best_centroid

    def count_frontiers(self, grid_msg: OccupancyGrid):
        """Count the number of valid frontier clusters.

        Useful for determining when exploration is complete.
        """
        width = grid_msg.info.width
        height = grid_msg.info.height
        if width == 0 or height == 0:
            return 0
        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
        frontier_mask = self._detect_frontiers(data, width, height)
        clusters = self._cluster_frontiers(frontier_mask, width, height)
        return len(clusters)

    def get_all_frontiers(self, grid_msg: OccupancyGrid):
        """Return all valid frontier cluster centroids as [(x, y), ...].

        Used for RViz visualisation.
        """
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        if width == 0 or height == 0:
            return []

        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
        frontier_mask = self._detect_frontiers(data, width, height)
        clusters = self._cluster_frontiers(frontier_mask, width, height)

        centroids = []
        for cluster in clusters:
            cy = sum(r for r, c in cluster) / len(cluster)
            cx = sum(c for r, c in cluster) / len(cluster)
            world_x = origin_x + (cx + 0.5) * resolution
            world_y = origin_y + (cy + 0.5) * resolution
            centroids.append((world_x, world_y))
        return centroids

    def get_exploration_progress(self, grid_msg: OccupancyGrid):
        """Return the fraction of cells that are explored (free or occupied).

        Returns a float in [0, 1].  1.0 means no unknown cells remain.
        """
        data = np.array(grid_msg.data, dtype=np.int8)
        total = len(data)
        if total == 0:
            return 0.0
        known = np.sum(data >= 0)
        return float(known) / float(total)

    # ------------------------------------------------------------------
    # Internal methods
    # ------------------------------------------------------------------

    def _detect_frontiers(self, data, width, height):
        """Return a boolean mask of frontier cells.

        A frontier cell is a FREE cell (value == 0) that has at least
        one UNKNOWN neighbour (value == -1) in its 8-connected neighbourhood.
        """
        frontier = np.zeros((height, width), dtype=bool)

        # Only look at free cells (value 0 in OccupancyGrid = definitely free)
        free_mask = (data == 0)

        for dr, dc in _NEIGHBOURS:
            # Shift the data array by (dr, dc) and check for unknown
            r_start = max(0, dr)
            r_end = height + min(0, dr)
            c_start = max(0, dc)
            c_end = width + min(0, dc)

            nr_start = max(0, -dr)
            nr_end = height + min(0, -dr)
            nc_start = max(0, -dc)
            nc_end = width + min(0, -dc)

            neighbour_unknown = (data[nr_start:nr_end, nc_start:nc_end] == -1)
            frontier[r_start:r_end, c_start:c_end] |= (
                free_mask[r_start:r_end, c_start:c_end] & neighbour_unknown
            )

        return frontier

    def _cluster_frontiers(self, frontier_mask, width, height):
        """Cluster frontier cells using BFS flood-fill.

        Returns a list of clusters, each cluster being a list of
        (row, col) tuples.  Only clusters with at least
        min_frontier_size cells are returned.
        """
        visited = np.zeros((height, width), dtype=bool)
        clusters = []

        # Find all frontier cell locations
        frontier_indices = np.argwhere(frontier_mask)  # (row, col) pairs

        for r, c in frontier_indices:
            if visited[r, c]:
                continue

            # BFS from this cell
            cluster = []
            queue = collections.deque()
            queue.append((r, c))
            visited[r, c] = True

            while queue:
                cr, cc = queue.popleft()
                cluster.append((cr, cc))

                for dr, dc in _NEIGHBOURS:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and not visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters
