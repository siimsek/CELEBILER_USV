"""Rolling local occupancy grid / costmap for obstacle avoidance and planning.

Integrates LIDAR scan points and camera detections into a body-frame grid.
Applies inflation, decay, and sector metrics extraction.
All coordinates are in base_link frame (x=forward, y=left).
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Any

from compliance_profile import (
    COSTMAP_BOAT_LENGTH_M, COSTMAP_BOAT_WIDTH_M, COSTMAP_CAMERA_COST,
    COSTMAP_CLEAR_THRESHOLD, COSTMAP_DECAY_FACTOR, COSTMAP_DECAY_INTERVAL_S,
    COSTMAP_FATAL_COST, COSTMAP_FRONT_M, COSTMAP_HEIGHT_CELLS,
    COSTMAP_INFLATED_COST, COSTMAP_INFLATION_RADIUS_M,
    COSTMAP_LIDAR_MAX_RANGE_M, COSTMAP_LIDAR_MIN_RANGE_M,
    COSTMAP_LIDAR_QUALITY_MIN, COSTMAP_OBSTACLE_COST, COSTMAP_ORIGIN_X_M,
    COSTMAP_ORIGIN_Y_M, COSTMAP_REAR_M, COSTMAP_RESOLUTION_M,
    COSTMAP_SAFETY_MARGIN_M, COSTMAP_SIDE_M, COSTMAP_WIDTH_CELLS,
    COSTMAP_STALE_DECAY_BOOST, COSTMAP_STALE_DECAY_TIMEOUT_S,
    COSTMAP_WRONG_TARGET_COST,
)


@dataclass
class SectorMetrics:
    left_min_m: float = 99.0
    center_min_m: float = 99.0
    right_min_m: float = 99.0
    left_clear: bool = True
    center_clear: bool = True
    right_clear: bool = True
    near_collision: bool = False


@dataclass
class CostmapCluster:
    cx_m: float = 0.0
    cy_m: float = 0.0
    radius_m: float = 0.0
    point_count: int = 0
    source: str = "lidar"


@dataclass
class CostmapSnapshot:
    timestamp: float = 0.0
    fresh: bool = True
    frame: str = "base_link"
    resolution_m: float = COSTMAP_RESOLUTION_M
    width_cells: int = COSTMAP_WIDTH_CELLS
    height_cells: int = COSTMAP_HEIGHT_CELLS
    origin_m: tuple = (COSTMAP_ORIGIN_X_M, COSTMAP_ORIGIN_Y_M)
    grid: list = field(default_factory=list)
    min_distance_m: float = 99.0
    sector: SectorMetrics = field(default_factory=SectorMetrics)
    clusters: list = field(default_factory=list)
    selected_corridor: str = "center"
    drop_reason: str | None = None
    lidar_point_count: int = 0
    camera_detection_count: int = 0
    inflation_radius_m: float = COSTMAP_INFLATION_RADIUS_M
    boat_width_m: float = COSTMAP_BOAT_WIDTH_M

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp, "fresh": self.fresh,
            "frame": self.frame, "resolution_m": self.resolution_m,
            "width_cells": self.width_cells, "height_cells": self.height_cells,
            "origin_m": {"x": self.origin_m[0], "y": self.origin_m[1]},
            "min_distance_m": round(self.min_distance_m, 3),
            "sector": {
                "left_min_m": round(self.sector.left_min_m, 3),
                "center_min_m": round(self.sector.center_min_m, 3),
                "right_min_m": round(self.sector.right_min_m, 3),
                "left_clear": self.sector.left_clear,
                "center_clear": self.sector.center_clear,
                "right_clear": self.sector.right_clear,
                "near_collision": self.sector.near_collision,
            },
            "clusters": [
                {"cx_m": round(c.cx_m, 3), "cy_m": round(c.cy_m, 3),
                 "radius_m": round(c.radius_m, 3), "point_count": c.point_count,
                 "source": c.source} for c in self.clusters],
            "selected_corridor": self.selected_corridor,
            "drop_reason": self.drop_reason,
            "lidar_point_count": self.lidar_point_count,
            "camera_detection_count": self.camera_detection_count,
            "inflation_radius_m": self.inflation_radius_m,
            "boat_width_m": self.boat_width_m,
        }


class RollingLocalCostmap:
    """Rolling local occupancy grid centered on the vehicle.
    Frame: base_link (x=forward, y=left, z=up)."""

    def __init__(self, *, resolution_m: float = COSTMAP_RESOLUTION_M,
                 front_m: float = COSTMAP_FRONT_M, rear_m: float = COSTMAP_REAR_M,
                 side_m: float = COSTMAP_SIDE_M,
                 inflation_radius_m: float = COSTMAP_INFLATION_RADIUS_M,
                 decay_factor: float = COSTMAP_DECAY_FACTOR,
                 decay_interval_s: float = COSTMAP_DECAY_INTERVAL_S,
                 boat_width_m: float = COSTMAP_BOAT_WIDTH_M,
                 boat_length_m: float = COSTMAP_BOAT_LENGTH_M,
                 safety_margin_m: float = COSTMAP_SAFETY_MARGIN_M) -> None:
        self.resolution_m = float(resolution_m)
        self.front_m = float(front_m)
        self.rear_m = float(rear_m)
        self.side_m = float(side_m)
        self.inflation_radius_m = float(inflation_radius_m)
        self.decay_factor = float(decay_factor)
        self.decay_interval_s = float(decay_interval_s)
        self.boat_width_m = float(boat_width_m)
        self.boat_length_m = float(boat_length_m)
        self.safety_margin_m = float(safety_margin_m)
        self.width_cells = max(10, int((2.0 * self.side_m) / self.resolution_m))
        self.height_cells = max(10, int((self.front_m + self.rear_m) / self.resolution_m))
        self.origin_x_m = -self.rear_m
        self.origin_y_m = -self.side_m
        self._grid = [[0] * self.width_cells for _ in range(self.height_cells)]
        self._update_times = [[0.0] * self.width_cells for _ in range(self.height_cells)]
        self._last_decay_mono = 0.0
        self._last_update_mono = 0.0
        self._lidar_point_count = 0
        self._camera_detection_count = 0
        self._fresh = False
        self._drop_reason = None
        self._inflation_cells = max(1, int(
            (self.inflation_radius_m + self.safety_margin_m + self.boat_width_m * 0.5)
            / self.resolution_m))

    def world_to_cell(self, x_m: float, y_m: float):
        """Convert base_link coords to grid cell indices. x=forward,y=left."""
        row = int((x_m - self.origin_x_m) / self.resolution_m)
        col = int((y_m - self.origin_y_m) / self.resolution_m)
        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            return (row, col)
        return None

    def cell_to_world(self, row: int, col: int):
        """Convert grid cell indices to base_link coordinates (center)."""
        x_m = self.origin_x_m + (row + 0.5) * self.resolution_m
        y_m = self.origin_y_m + (col + 0.5) * self.resolution_m
        return (x_m, y_m)

    def _set_cell_cost(self, row, col, cost, now_mono, source=""):
        if 0 <= row < self.height_cells and 0 <= col < self.width_cells:
            if cost > self._grid[row][col]:
                self._grid[row][col] = min(cost, COSTMAP_FATAL_COST)
                self._update_times[row][col] = now_mono

    def _apply_inflation(self, row, col, cost, now_mono):
        r = self._inflation_cells
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < self.height_cells and 0 <= nc < self.width_cells:
                    dist = math.sqrt(dr * dr + dc * dc) * self.resolution_m
                    max_dist = self.inflation_radius_m + self.safety_margin_m
                    if dist <= max_dist:
                        ratio = 1.0 - (dist / max_dist) if max_dist > 0 else 0
                        inflated_cost = int(COSTMAP_INFLATED_COST * ratio)
                        if inflated_cost > self._grid[nr][nc]:
                            self._grid[nr][nc] = min(inflated_cost, cost)
                            self._update_times[nr][nc] = now_mono

    def update_lidar_points(self, points, *, quality_ratio=1.0, now_mono=None):
        """Update costmap with LIDAR points in base_link frame (x=forward, y=left)."""
        now = now_mono if now_mono is not None else time.monotonic()
        if quality_ratio < COSTMAP_LIDAR_QUALITY_MIN:
            self._drop_reason = "lidar_quality_below_threshold"
            self._lidar_point_count = 0
            return 0
        count = 0
        for x_m, y_m in points:
            dist = math.sqrt(x_m * x_m + y_m * y_m)
            if dist < COSTMAP_LIDAR_MIN_RANGE_M or dist > COSTMAP_LIDAR_MAX_RANGE_M:
                continue
            cell = self.world_to_cell(x_m, y_m)
            if cell is None:
                continue
            row, col = cell
            self._set_cell_cost(row, col, COSTMAP_OBSTACLE_COST, now, "lidar")
            self._apply_inflation(row, col, COSTMAP_OBSTACLE_COST, now)
            count += 1
        self._lidar_point_count = count
        if count > 0:
            self._last_update_mono = now
            self._fresh = True
            self._drop_reason = None
        return count

    def update_camera_detections(self, detections, *, estimated_range_m=5.0, now_mono=None):
        """Update costmap with camera detections (bearing, class, wrong_target)."""
        now = now_mono if now_mono is not None else time.monotonic()
        count = 0
        for det in detections:
            bearing_deg = float(det.get("bearing_deg", 0.0))
            is_wrong_target = bool(det.get("is_wrong_target", False))
            area_norm = float(det.get("area_norm", 0.0))
            if is_wrong_target or det.get("class_name", "") in ("SARI_ENGEL", "YESIL_SANCAK", "wrong_target"):
                # Camera bearing is +right; base_link y is +left.
                bearing_rad = math.radians(-bearing_deg)
                range_m = min(estimated_range_m, COSTMAP_LIDAR_MAX_RANGE_M)
                x_m = range_m * math.cos(bearing_rad)
                y_m = range_m * math.sin(bearing_rad)
                cell = self.world_to_cell(x_m, y_m)
                if cell is not None:
                    row, col = cell
                    # Adım 4 — Wrong-target yüksek maliyetli engel (P3 güvenlik):
                    # OBSTACLE_COST + tam inflation → planner çarpışma olarak ele edip
                    # WRONG_TARGET_AVOID ile sarmalar; FATAL değil (anında HOLD riski yok).
                    wrong_target_cost = max(COSTMAP_OBSTACLE_COST, min(COSTMAP_WRONG_TARGET_COST, COSTMAP_FATAL_COST))
                    self._set_cell_cost(row, col, wrong_target_cost, now, "camera_wrong_target")
                    self._apply_inflation(row, col, wrong_target_cost, now)
                    count += 1
        self._camera_detection_count = count
        if count > 0:
            self._last_update_mono = now
        return count

    def apply_decay(self, now_mono=None):
        """Decay old cells. Returns number of cells decayed."""
        now = now_mono if now_mono is not None else time.monotonic()
        if now - self._last_decay_mono < self.decay_interval_s:
            return 0
        decayed = 0
        # Adım 4 — Decay-tazelik: sensör bayatsa decay hızlansın (eski engeller çabuk kaybolsun).
        stale = (self._last_update_mono > 0) and ((now - self._last_update_mono) > float(COSTMAP_STALE_DECAY_TIMEOUT_S))
        boost = max(1.0, float(COSTMAP_STALE_DECAY_BOOST))
        eff_decay = math.pow(self.decay_factor, boost) if stale else self.decay_factor
        for row in range(self.height_cells):
            for col in range(self.width_cells):
                if self._grid[row][col] > 0:
                    age = now - self._update_times[row][col]
                    if age > self.decay_interval_s:
                        new_cost = int(self._grid[row][col] * eff_decay)
                        if new_cost < COSTMAP_CLEAR_THRESHOLD:
                            self._grid[row][col] = 0
                            self._update_times[row][col] = 0.0
                        else:
                            self._grid[row][col] = new_cost
                        decayed += 1
        self._last_decay_mono = now
        return decayed

    def clear(self):
        """Clear the entire costmap."""
        self._grid = [[0] * self.width_cells for _ in range(self.height_cells)]
        self._update_times = [[0.0] * self.width_cells for _ in range(self.height_cells)]
        self._fresh = False
        self._lidar_point_count = 0
        self._camera_detection_count = 0

    def _extract_sector_min_distance(self, angle_start_deg, angle_end_deg, max_range_m):
        """Extract minimum distance to obstacle in a sector (degrees, 0=forward)."""
        min_dist = max_range_m
        for row in range(self.height_cells):
            for col in range(self.width_cells):
                if self._grid[row][col] >= COSTMAP_CLEAR_THRESHOLD:
                    x_m, y_m = self.cell_to_world(row, col)
                    dist = math.sqrt(x_m * x_m + y_m * y_m)
                    if dist < 0.01:
                        continue
                    angle_deg = math.degrees(math.atan2(y_m, x_m))
                    if angle_start_deg <= angle_deg <= angle_end_deg:
                        min_dist = min(min_dist, dist)
        return min_dist

    def extract_sector_metrics(self, *, warn_distance_m=2.5):
        """Extract three-sector metrics from the costmap."""
        left_min = self._extract_sector_min_distance(30.0, 150.0, 99.0)
        center_min = self._extract_sector_min_distance(-30.0, 30.0, 99.0)
        right_min = self._extract_sector_min_distance(-150.0, -30.0, 99.0)
        return SectorMetrics(
            left_min_m=left_min, center_min_m=center_min, right_min_m=right_min,
            left_clear=left_min > warn_distance_m,
            center_clear=center_min > warn_distance_m,
            right_clear=right_min > warn_distance_m,
            near_collision=center_min < (warn_distance_m * 0.5),
        )

    def _find_clusters(self):
        """Simple cluster detection from occupied cells."""
        visited = set()
        clusters = []
        for row in range(self.height_cells):
            for col in range(self.width_cells):
                if self._grid[row][col] >= COSTMAP_OBSTACLE_COST and (row, col) not in visited:
                    component = []
                    queue = [(row, col)]
                    while queue:
                        r, c = queue.pop(0)
                        if (r, c) in visited:
                            continue
                        if r < 0 or r >= self.height_cells or c < 0 or c >= self.width_cells:
                            continue
                        if self._grid[r][c] < COSTMAP_CLEAR_THRESHOLD:
                            continue
                        visited.add((r, c))
                        component.append((r, c))
                        queue.extend([(r+1,c),(r-1,c),(r,c+1),(r,c-1)])
                        if len(component) > 200:
                            break
                    if len(component) >= 3:
                        cx = sum(self.cell_to_world(r, c)[0] for r, c in component) / len(component)
                        cy = sum(self.cell_to_world(r, c)[1] for r, c in component) / len(component)
                        max_r = max(math.sqrt((self.cell_to_world(r, c)[0]-cx)**2 + (self.cell_to_world(r, c)[1]-cy)**2) for r, c in component)
                        clusters.append(CostmapCluster(cx_m=cx, cy_m=cy, radius_m=max_r, point_count=len(component), source="lidar"))
                    if len(clusters) > 20:
                        break
            if len(clusters) > 20:
                break
        return clusters

    def _select_corridor(self, sector):
        """Select the best corridor based on sector metrics."""
        if sector.center_clear:
            return "center"
        if sector.left_clear and not sector.right_clear:
            return "left"
        if sector.right_clear and not sector.left_clear:
            return "right"
        if sector.left_clear and sector.right_clear:
            return "left" if sector.left_min_m >= sector.right_min_m else "right"
        return "blocked"

    def get_snapshot(self, *, freshness_timeout_s=2.0, now_mono=None):
        """Get complete costmap snapshot for downstream consumers."""
        now = now_mono if now_mono is not None else time.monotonic()
        age = now - self._last_update_mono if self._last_update_mono > 0 else 999.0
        fresh = age <= freshness_timeout_s
        sector = self.extract_sector_metrics()
        clusters = self._find_clusters()
        corridor = self._select_corridor(sector)
        min_dist = min(sector.left_min_m, sector.center_min_m, sector.right_min_m)
        grid_copy = [row[:] for row in self._grid]
        return CostmapSnapshot(
            timestamp=now, fresh=fresh, frame="base_link",
            resolution_m=self.resolution_m, width_cells=self.width_cells,
            height_cells=self.height_cells, origin_m=(self.origin_x_m, self.origin_y_m),
            grid=grid_copy, min_distance_m=min_dist, sector=sector,
            clusters=clusters, selected_corridor=corridor,
            drop_reason=self._drop_reason if not fresh else None,
            lidar_point_count=self._lidar_point_count,
            camera_detection_count=self._camera_detection_count,
            inflation_radius_m=self.inflation_radius_m,
            boat_width_m=self.boat_width_m,
        )

    def get_cost_at(self, x_m: float, y_m: float) -> int:
        """Get cost at a specific base_link coordinate."""
        cell = self.world_to_cell(x_m, y_m)
        if cell is None:
            return COSTMAP_FATAL_COST
        return self._grid[cell[0]][cell[1]]

    def check_trajectory_collision(self, trajectory, *, threshold=COSTMAP_INFLATED_COST):
        """Check if a trajectory collides with the costmap. Returns (collision, min_clearance)."""
        min_clearance = 99.0
        collision = False
        occupied = []
        for row in range(self.height_cells):
            for col in range(self.width_cells):
                cost = self._grid[row][col]
                if cost >= COSTMAP_CLEAR_THRESHOLD:
                    occupied.append((*self.cell_to_world(row, col), cost))
        for x_m, y_m in trajectory:
            cell = self.world_to_cell(x_m, y_m)
            if cell is None:
                collision = True
                min_clearance = 0.0
                continue
            cost = self._grid[cell[0]][cell[1]]
            if cost >= threshold:
                collision = True
            if cost >= COSTMAP_CLEAR_THRESHOLD:
                min_clearance = min(min_clearance, 0.0)
            for obs_x_m, obs_y_m, _ in occupied:
                dist = math.hypot(float(obs_x_m) - float(x_m), float(obs_y_m) - float(y_m))
                min_clearance = min(min_clearance, dist)
        return collision, min_clearance
