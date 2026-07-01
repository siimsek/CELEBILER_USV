"""
Persistent obstacle landmark tracking in world ENU frame.

Lidar and camera world points are clustered into landmarks that persist
within a mission run.  Short sensor dropouts do not erase landmarks;
confidence decays gradually.  New mission start resets all landmarks.

Used by usv_main.py for avoidance decisions and by telemetry.py for
the /api/spatial_map endpoint.
"""
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from compliance_profile import (
        OBSTACLE_LANDMARK_MERGE_RADIUS_M,
        OBSTACLE_LANDMARK_MIN_HITS_FOR_CONFIDENCE,
        OBSTACLE_LANDMARK_MIN_CONFIDENCE,
        OBSTACLE_LANDMARK_DECAY_RATE_PER_S,
        OBSTACLE_LANDMARK_MAX_AGE_S,
        OBSTACLE_LANDMARK_STALE_TOLERANCE_S,
        OBSTACLE_LANDMARK_MAX_COUNT,
    )
except ImportError:
    OBSTACLE_LANDMARK_MERGE_RADIUS_M = 1.5
    OBSTACLE_LANDMARK_MIN_HITS_FOR_CONFIDENCE = 3
    OBSTACLE_LANDMARK_MIN_CONFIDENCE = 0.3
    OBSTACLE_LANDMARK_DECAY_RATE_PER_S = 0.02
    OBSTACLE_LANDMARK_MAX_AGE_S = 120.0
    OBSTACLE_LANDMARK_STALE_TOLERANCE_S = 5.0
    OBSTACLE_LANDMARK_MAX_COUNT = 64


@dataclass
class ObstacleLandmark:
    """A single persistent obstacle landmark in ENU world coordinates."""
    landmark_id: int
    centroid_east_m: float
    centroid_north_m: float
    peak_east_m: float       # Highest-density signal point
    peak_north_m: float
    radius_m: float
    hit_count: int
    confidence: float         # 0..1, increases with hits, decays with time
    last_seen_mono: float     # time.monotonic() of last observation
    first_seen_mono: float    # time.monotonic() of first observation
    # internal accumulators
    _sum_east: float = 0.0
    _sum_north: float = 0.0
    _point_count: int = 0
    _peak_density: float = 0.0

    def age_s(self, now_mono: float) -> float:
        return max(0.0, now_mono - self.last_seen_mono)

    def total_age_s(self, now_mono: float) -> float:
        return max(0.0, now_mono - self.first_seen_mono)

    def is_stale(self, now_mono: float) -> bool:
        return self.age_s(now_mono) > float(OBSTACLE_LANDMARK_STALE_TOLERANCE_S)

    def is_expired(self, now_mono: float) -> bool:
        return (
            self.age_s(now_mono) > float(OBSTACLE_LANDMARK_MAX_AGE_S)
            or self.confidence < float(OBSTACLE_LANDMARK_MIN_CONFIDENCE) * 0.5
        )

    def to_dict(self) -> dict:
        now = time.monotonic()
        return {
            "landmark_id": self.landmark_id,
            "centroid_east_m": round(self.centroid_east_m, 3),
            "centroid_north_m": round(self.centroid_north_m, 3),
            "peak_east_m": round(self.peak_east_m, 3),
            "peak_north_m": round(self.peak_north_m, 3),
            "radius_m": round(self.radius_m, 3),
            "hit_count": self.hit_count,
            "confidence": round(self.confidence, 4),
            "age_s": round(self.age_s(now), 3),
            "total_age_s": round(self.total_age_s(now), 3),
            "stale": self.is_stale(now),
        }


class ObstacleLandmarkMap:
    """
    Maintains a world-frame persistent obstacle landmark map.

    Points arriving within merge_radius of an existing landmark are
    absorbed into that landmark (centroid updated, hit_count incremented,
    peak_point tracked).  New points far from any landmark create new
    landmarks.

    Confidence increases with hit_count and decays with time since
    last_seen.  Expired landmarks (too old or too low confidence) are
    pruned.
    """

    def __init__(self):
        self._landmarks: Dict[int, ObstacleLandmark] = {}
        self._next_id: int = 1
        self._last_decay_mono: float = time.monotonic()

    def reset(self) -> None:
        """Clear all landmarks — call at mission start."""
        self._landmarks.clear()
        self._next_id = 1
        self._last_decay_mono = time.monotonic()

    def update(
        self,
        world_enu_points: List[Tuple[float, float]],
        timestamp_mono: float,
    ) -> None:
        """
        Incorporate new ENU world points into the landmark map.

        Each point is (east_m, north_m).  Points near existing landmarks
        are merged; others create new landmarks.
        """
        if not world_enu_points:
            self._decay_and_prune(timestamp_mono)
            return

        merge_r = float(OBSTACLE_LANDMARK_MERGE_RADIUS_M)
        merge_r_sq = merge_r * merge_r

        for pt in world_enu_points:
            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                continue
            try:
                e_m = float(pt[0])
                n_m = float(pt[1])
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(e_m) and math.isfinite(n_m)):
                continue

            # Find nearest existing landmark
            best_lm: Optional[ObstacleLandmark] = None
            best_dist_sq = merge_r_sq
            for lm in self._landmarks.values():
                de = e_m - lm.centroid_east_m
                dn = n_m - lm.centroid_north_m
                d_sq = de * de + dn * dn
                if d_sq < best_dist_sq:
                    best_dist_sq = d_sq
                    best_lm = lm

            if best_lm is not None:
                # Merge into existing landmark
                self._merge_point(best_lm, e_m, n_m, timestamp_mono)
            else:
                # Create new landmark
                if len(self._landmarks) < int(OBSTACLE_LANDMARK_MAX_COUNT):
                    self._create_landmark(e_m, n_m, timestamp_mono)

        self._decay_and_prune(timestamp_mono)

    def _merge_point(
        self, lm: ObstacleLandmark, e_m: float, n_m: float, ts: float,
    ) -> None:
        """Merge a new point into an existing landmark."""
        lm.hit_count += 1
        lm.last_seen_mono = ts
        lm._point_count += 1
        lm._sum_east += e_m
        lm._sum_north += n_m

        # Update centroid as running mean
        lm.centroid_east_m = lm._sum_east / lm._point_count
        lm.centroid_north_m = lm._sum_north / lm._point_count

        # Update radius as max distance from centroid
        de = e_m - lm.centroid_east_m
        dn = n_m - lm.centroid_north_m
        r = math.sqrt(de * de + dn * dn)
        lm.radius_m = max(lm.radius_m, r)

        # Track peak point: use local density proxy (hit_count / radius)
        density = lm.hit_count / max(0.1, lm.radius_m)
        if density > lm._peak_density or lm.hit_count <= 2:
            lm.peak_east_m = e_m
            lm.peak_north_m = n_m
            lm._peak_density = density

        # Confidence: asymptotic rise with hit_count
        min_hits = max(1, int(OBSTACLE_LANDMARK_MIN_HITS_FOR_CONFIDENCE))
        lm.confidence = min(1.0, lm.hit_count / (lm.hit_count + min_hits))

    def _create_landmark(self, e_m: float, n_m: float, ts: float) -> None:
        """Create a new landmark from a single observation."""
        lm = ObstacleLandmark(
            landmark_id=self._next_id,
            centroid_east_m=e_m,
            centroid_north_m=n_m,
            peak_east_m=e_m,
            peak_north_m=n_m,
            radius_m=0.2,  # initial small radius
            hit_count=1,
            confidence=1.0 / (1.0 + max(1, int(OBSTACLE_LANDMARK_MIN_HITS_FOR_CONFIDENCE))),
            last_seen_mono=ts,
            first_seen_mono=ts,
            _sum_east=e_m,
            _sum_north=n_m,
            _point_count=1,
            _peak_density=1.0,
        )
        self._landmarks[self._next_id] = lm
        self._next_id += 1

    def _decay_and_prune(self, now_mono: float) -> None:
        """Decay confidence of unseen landmarks and prune expired ones."""
        dt = max(0.0, now_mono - self._last_decay_mono)
        self._last_decay_mono = now_mono
        if dt < 0.1:
            return  # Don't decay faster than 10Hz

        decay_rate = float(OBSTACLE_LANDMARK_DECAY_RATE_PER_S)
        stale_tol = float(OBSTACLE_LANDMARK_STALE_TOLERANCE_S)
        to_remove = []

        for lid, lm in self._landmarks.items():
            age = lm.age_s(now_mono)
            if age > stale_tol:
                # Decay confidence for unseen landmarks
                lm.confidence = max(0.0, lm.confidence - decay_rate * dt)
            if lm.is_expired(now_mono):
                to_remove.append(lid)

        for lid in to_remove:
            del self._landmarks[lid]

    def get_landmarks(self) -> List[ObstacleLandmark]:
        """Return all active landmarks sorted by confidence (descending)."""
        return sorted(self._landmarks.values(), key=lambda lm: -lm.confidence)

    def get_confident_landmarks(self) -> List[ObstacleLandmark]:
        """Return landmarks with confidence above threshold."""
        min_conf = float(OBSTACLE_LANDMARK_MIN_CONFIDENCE)
        return [lm for lm in self.get_landmarks() if lm.confidence >= min_conf]

    def get_threat_landmarks(
        self,
        boat_east_m: float,
        boat_north_m: float,
        threat_radius_m: float,
    ) -> List[ObstacleLandmark]:
        """Return confident landmarks within threat_radius_m of the boat."""
        now = time.monotonic()
        min_conf = float(OBSTACLE_LANDMARK_MIN_CONFIDENCE)
        result = []
        r_sq = threat_radius_m * threat_radius_m
        for lm in self._landmarks.values():
            if lm.confidence < min_conf:
                continue
            de = lm.centroid_east_m - boat_east_m
            dn = lm.centroid_north_m - boat_north_m
            if (de * de + dn * dn) <= r_sq:
                result.append(lm)
        return sorted(result, key=lambda lm: -lm.confidence)

    def get_threat_bearing_bias(
        self,
        boat_east_m: float,
        boat_north_m: float,
        boat_heading_rad: float,
        threat_radius_m: float,
        max_bias_deg: float = 30.0,
    ) -> Tuple[float, str]:
        """
        Compute a heading bias (degrees) to avoid nearby confident landmarks.

        Returns (bias_deg, reason).  Positive bias = turn right.
        """
        threats = self.get_threat_landmarks(boat_east_m, boat_north_m, threat_radius_m)
        if not threats:
            return 0.0, "no_landmark_threat"

        # Weighted average threat bearing relative to boat heading
        total_weight = 0.0
        weighted_bearing_sin = 0.0
        weighted_bearing_cos = 0.0
        for lm in threats:
            if lm.radius_m <= 1.0:
                de = lm.centroid_east_m - boat_east_m
                dn = lm.centroid_north_m - boat_north_m
            else:
                de = lm.peak_east_m - boat_east_m
                dn = lm.peak_north_m - boat_north_m
            dist = math.sqrt(de * de + dn * dn)
            if dist < 0.01:
                continue
            # ENU bearing: atan2(east, north)
            bearing = math.atan2(de, dn) - boat_heading_rad
            # Proximity and confidence weight
            weight = lm.confidence * (1.0 / max(0.5, dist))
            total_weight += weight
            weighted_bearing_sin += weight * math.sin(bearing)
            weighted_bearing_cos += weight * math.cos(bearing)

        if total_weight < 0.01:
            return 0.0, "no_landmark_threat"

        avg_bearing = math.atan2(weighted_bearing_sin, weighted_bearing_cos)
        avg_bearing_deg = math.degrees(avg_bearing)

        # Bias away from threat: if threat is on the left (negative bearing), steer right
        if abs(avg_bearing_deg) < 5.0:
            # Threat directly ahead — pick side with more clearance
            left_threats = sum(
                1 for lm in threats
                if math.atan2(lm.peak_east_m - boat_east_m, lm.peak_north_m - boat_north_m) - boat_heading_rad < 0
            )
            right_threats = len(threats) - left_threats
            bias_sign = 1.0 if left_threats >= right_threats else -1.0
            bias_magnitude = min(max_bias_deg, 15.0 + 10.0 * len(threats))
        else:
            bias_sign = -1.0 if avg_bearing_deg > 0.0 else 1.0
            proximity_factor = min(1.0, len(threats) / 3.0)
            bias_magnitude = min(max_bias_deg, abs(avg_bearing_deg) * 0.6 * proximity_factor + 10.0)

        bias_deg = bias_sign * bias_magnitude
        return round(bias_deg, 3), "landmark_avoidance"

    @property
    def landmark_count(self) -> int:
        return len(self._landmarks)

    @property
    def confident_count(self) -> int:
        min_conf = float(OBSTACLE_LANDMARK_MIN_CONFIDENCE)
        return sum(1 for lm in self._landmarks.values() if lm.confidence >= min_conf)

    def to_dict(self) -> dict:
        """Export for mission_state / API."""
        landmarks_list = [lm.to_dict() for lm in self.get_landmarks()]
        return {
            "landmark_count": self.landmark_count,
            "confident_count": self.confident_count,
            "landmarks": landmarks_list,
        }


# ── Singleton ────────────────────────────────────────────────────────────────
_instance: Optional[ObstacleLandmarkMap] = None


def get_obstacle_landmark_map() -> ObstacleLandmarkMap:
    """Return the singleton ObstacleLandmarkMap instance."""
    global _instance
    if _instance is None:
        _instance = ObstacleLandmarkMap()
    return _instance
