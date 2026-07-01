"""P3 vision target memory and search bearing helper."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass
class TargetMemoryState:
    detected: bool = False
    active: bool = False
    bearing_deg: float = 0.0
    area_norm: float = 0.0
    confidence: float = 0.0
    lost_s: float = 999.0
    stable_s: float = 0.0
    bearing_rate_dps: float = 0.0
    source: str = "none"

    def to_dict(self) -> dict[str, float | bool | str]:
        return {
            "detected": bool(self.detected),
            "active": bool(self.active),
            "bearing_deg": round(float(self.bearing_deg), 3),
            "area_norm": round(float(self.area_norm), 4),
            "confidence": round(float(self.confidence), 4),
            "lost_s": round(float(self.lost_s), 3),
            "stable_s": round(float(self.stable_s), 3),
            "bearing_rate_dps": round(float(self.bearing_rate_dps), 3),
            "source": str(self.source),
        }


class TargetMemory:
    """Keeps a decaying target estimate after short vision dropouts."""

    def __init__(
        self,
        *,
        memory_s: float = 8.0,
        bearing_rate_alpha: float = 0.35,
        max_bearing_deg: float = 45.0,
        search_period_s: float = 6.0,
    ) -> None:
        self.memory_s = max(0.1, float(memory_s))
        self.bearing_rate_alpha = _clamp(bearing_rate_alpha, 0.05, 1.0)
        self.max_bearing_deg = max(1.0, float(max_bearing_deg))
        self.search_period_s = max(1.0, float(search_period_s))
        self._last_seen_s = 0.0
        self._first_seen_s = 0.0
        self._last_update_s = 0.0
        self._last_bearing_deg = 0.0
        self._last_area_norm = 0.0
        self._bearing_rate_dps = 0.0
        self._search_start_s = 0.0

    @staticmethod
    def _wrap_180(deg: float) -> float:
        value = (float(deg) + 180.0) % 360.0 - 180.0
        if value == -180.0:
            return 180.0
        return value

    def reset(self) -> None:
        self._last_seen_s = 0.0
        self._first_seen_s = 0.0
        self._last_update_s = 0.0
        self._last_bearing_deg = 0.0
        self._last_area_norm = 0.0
        self._bearing_rate_dps = 0.0
        self._search_start_s = 0.0

    def search_bearing(self, now_s: float) -> float:
        now = float(now_s)
        if self._search_start_s <= 0.0:
            self._search_start_s = now
        elapsed = max(0.0, now - self._search_start_s)
        cycle = elapsed / self.search_period_s
        amp = min(self.max_bearing_deg, 10.0 + 7.5 * math.floor(cycle))
        phase = (cycle % 1.0) * 2.0 * math.pi
        return float(amp * math.sin(phase))

    def update(
        self,
        *,
        detected: bool,
        bearing_deg: float,
        area_norm: float,
        confidence: float,
        now_s: float,
    ) -> TargetMemoryState:
        now = float(now_s)
        bearing = _clamp(float(bearing_deg), -self.max_bearing_deg, self.max_bearing_deg)
        area = _clamp(float(area_norm), 0.0, 1.0)
        conf = _clamp(float(confidence), 0.0, 1.0)

        if bool(detected):
            if self._last_seen_s <= 0.0:
                self._first_seen_s = now
            dt = max(0.02, min(1.0, now - float(self._last_update_s or now)))
            if self._last_update_s > 0.0:
                rate = self._wrap_180(bearing - self._last_bearing_deg) / dt
                alpha = self.bearing_rate_alpha
                self._bearing_rate_dps = (alpha * rate) + ((1.0 - alpha) * self._bearing_rate_dps)
            self._last_seen_s = now
            self._last_update_s = now
            self._last_bearing_deg = bearing
            self._last_area_norm = area
            self._search_start_s = 0.0
            return TargetMemoryState(
                detected=True,
                active=True,
                bearing_deg=bearing,
                area_norm=area,
                confidence=conf,
                lost_s=0.0,
                stable_s=max(0.0, now - float(self._first_seen_s or now)),
                bearing_rate_dps=self._bearing_rate_dps,
                source="vision_live",
            )

        if self._last_seen_s <= 0.0:
            return TargetMemoryState(
                detected=False,
                active=False,
                bearing_deg=self.search_bearing(now),
                area_norm=0.0,
                confidence=0.0,
                lost_s=999.0,
                stable_s=0.0,
                bearing_rate_dps=0.0,
                source="search_spiral",
            )

        lost_s = max(0.0, now - self._last_seen_s)
        if lost_s <= self.memory_s:
            decay = max(0.0, 1.0 - (lost_s / self.memory_s))
            predicted = self._last_bearing_deg + (self._bearing_rate_dps * lost_s)
            predicted = _clamp(self._wrap_180(predicted), -self.max_bearing_deg, self.max_bearing_deg)
            return TargetMemoryState(
                detected=False,
                active=True,
                bearing_deg=predicted,
                area_norm=self._last_area_norm * decay,
                confidence=decay,
                lost_s=lost_s,
                stable_s=0.0,
                bearing_rate_dps=self._bearing_rate_dps,
                source="vision_memory",
            )

        return TargetMemoryState(
            detected=False,
            active=False,
            bearing_deg=self.search_bearing(now),
            area_norm=0.0,
            confidence=0.0,
            lost_s=lost_s,
            stable_s=0.0,
            bearing_rate_dps=self._bearing_rate_dps,
            source="search_spiral",
        )
