"""Dynamic obstacle tracking and TTC helpers for body-frame detections."""

from __future__ import annotations

from dataclasses import dataclass
import math

from compliance_profile import (
    COSTMAP_BOAT_WIDTH_M,
    OBSTACLE_TTC_DANGER_S,
    OBSTACLE_TTC_EMERGENCY_S,
    OBSTACLE_TTC_STOP_S,
    OBSTACLE_TTC_WARN_S,
)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass
class TrackedObstacle:
    track_id: int
    x_m: float
    y_m: float
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    radius_m: float = 0.3
    confidence: float = 0.0
    last_seen_s: float = 0.0
    age_s: float = 0.0
    hits: int = 1
    ttc_s: float | None = None
    ttc_level: str = "normal"

    def to_dict(self) -> dict[str, float | int | str | None]:
        return {
            "track_id": int(self.track_id),
            "x_m": round(float(self.x_m), 3),
            "y_m": round(float(self.y_m), 3),
            "vx_mps": round(float(self.vx_mps), 3),
            "vy_mps": round(float(self.vy_mps), 3),
            "radius_m": round(float(self.radius_m), 3),
            "confidence": round(float(self.confidence), 3),
            "age_s": round(float(self.age_s), 3),
            "hits": int(self.hits),
            "ttc_s": round(float(self.ttc_s), 3) if self.ttc_s is not None and math.isfinite(float(self.ttc_s)) else None,
            "ttc_level": str(self.ttc_level),
        }


def classify_ttc(ttc_s: float | None) -> str:
    if ttc_s is None or not math.isfinite(float(ttc_s)):
        return "normal"
    if ttc_s <= float(OBSTACLE_TTC_STOP_S):
        return "stop"
    if ttc_s <= float(OBSTACLE_TTC_EMERGENCY_S):
        return "emergency"
    if ttc_s <= float(OBSTACLE_TTC_DANGER_S):
        return "danger"
    if ttc_s <= float(OBSTACLE_TTC_WARN_S):
        return "warn"
    return "normal"


def ttc_speed_factor(ttc_s: float | None) -> float:
    level = classify_ttc(ttc_s)
    if level in ("stop", "emergency"):
        return 0.0
    if level == "danger":
        return 0.25
    if level == "warn":
        return 0.50
    return 1.0


class DynamicObstacleTracker:
    """Nearest-neighbor obstacle tracker in vehicle base_link frame."""

    def __init__(
        self,
        *,
        match_gate_m: float = 1.25,
        stale_s: float = 2.5,
        velocity_alpha: float = 0.45,
        boat_half_width_m: float = COSTMAP_BOAT_WIDTH_M * 0.5,
        lateral_margin_m: float = 0.45,
    ) -> None:
        self.match_gate_m = float(match_gate_m)
        self.stale_s = float(stale_s)
        self.velocity_alpha = _clamp(velocity_alpha, 0.05, 1.0)
        self.boat_half_width_m = float(boat_half_width_m)
        self.lateral_margin_m = float(lateral_margin_m)
        self._tracks: dict[int, TrackedObstacle] = {}
        self._next_track_id = 1

    def reset(self) -> None:
        self._tracks.clear()
        self._next_track_id = 1

    def _match_track(self, x_m: float, y_m: float, used: set[int]) -> TrackedObstacle | None:
        best = None
        best_dist = float("inf")
        for track in self._tracks.values():
            if track.track_id in used:
                continue
            dist = math.hypot(float(track.x_m) - float(x_m), float(track.y_m) - float(y_m))
            if dist < best_dist:
                best = track
                best_dist = dist
        if best is not None and best_dist <= self.match_gate_m:
            return best
        return None

    def _update_ttc(self, track: TrackedObstacle, boat_speed_mps: float) -> None:
        if track.x_m <= 0.0:
            track.ttc_s = None
            track.ttc_level = "normal"
            return
        lateral_limit = self.boat_half_width_m + float(track.radius_m) + self.lateral_margin_m
        if abs(float(track.y_m)) > lateral_limit:
            track.ttc_s = None
            track.ttc_level = "normal"
            return
        distance = max(0.0, float(track.x_m) - max(0.0, float(track.radius_m)))
        bearing_abs = abs(math.atan2(float(track.y_m), max(float(track.x_m), 0.01)))
        ego_closing = max(0.0, float(boat_speed_mps) * math.cos(bearing_abs))
        relative_closing = max(0.0, -float(track.vx_mps))
        closing = max(relative_closing, ego_closing)
        if closing <= 0.05:
            track.ttc_s = None
            track.ttc_level = "normal"
            return
        track.ttc_s = distance / closing
        track.ttc_level = classify_ttc(track.ttc_s)

    def update(
        self,
        detections: list[dict],
        *,
        now_s: float,
        boat_speed_mps: float = 0.0,
    ) -> list[TrackedObstacle]:
        now = float(now_s)
        used: set[int] = set()
        for det in detections:
            try:
                x_m = float(det.get("x_m", det.get("cx_m", 0.0)))
                y_m = float(det.get("y_m", det.get("cy_m", 0.0)))
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(x_m) and math.isfinite(y_m)):
                continue
            radius = max(0.05, float(det.get("radius_m", 0.25) or 0.25))
            confidence = _clamp(det.get("confidence", 1.0), 0.0, 1.0)
            track = self._match_track(x_m, y_m, used)
            if track is None:
                track = TrackedObstacle(
                    track_id=self._next_track_id,
                    x_m=x_m,
                    y_m=y_m,
                    radius_m=radius,
                    confidence=confidence,
                    last_seen_s=now,
                )
                self._tracks[track.track_id] = track
                self._next_track_id += 1
            else:
                dt = max(0.02, min(1.0, now - float(track.last_seen_s or now)))
                vx = (x_m - float(track.x_m)) / dt
                vy = (y_m - float(track.y_m)) / dt
                alpha = self.velocity_alpha
                track.vx_mps = (alpha * vx) + ((1.0 - alpha) * float(track.vx_mps))
                track.vy_mps = (alpha * vy) + ((1.0 - alpha) * float(track.vy_mps))
                track.x_m = x_m
                track.y_m = y_m
                track.radius_m = radius
                track.confidence = max(confidence, float(track.confidence) * 0.85)
                track.last_seen_s = now
                track.hits += 1
            used.add(track.track_id)

        stale = []
        for track_id, track in self._tracks.items():
            age = max(0.0, now - float(track.last_seen_s or now))
            track.age_s = age
            if age > self.stale_s:
                stale.append(track_id)
                continue
            track.confidence *= max(0.0, 1.0 - (0.12 * age))
            self._update_ttc(track, boat_speed_mps)
        for track_id in stale:
            self._tracks.pop(track_id, None)
        return self.tracks()

    def tracks(self) -> list[TrackedObstacle]:
        return sorted(self._tracks.values(), key=lambda t: (t.ttc_s if t.ttc_s is not None else 9999.0, t.track_id))

    def summary(self) -> dict:
        tracks = self.tracks()
        finite_ttc = [float(t.ttc_s) for t in tracks if t.ttc_s is not None and math.isfinite(float(t.ttc_s))]
        min_ttc = min(finite_ttc) if finite_ttc else None
        return {
            "track_count": int(len(tracks)),
            "min_ttc_s": round(float(min_ttc), 3) if min_ttc is not None else None,
            "ttc_level": classify_ttc(min_ttc),
            "ttc_speed_factor": round(float(ttc_speed_factor(min_ttc)), 3),
            "tracks": [t.to_dict() for t in tracks[:8]],
        }
