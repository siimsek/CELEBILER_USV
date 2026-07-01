"""Sensor Fusion Runtime Layer - Fuses LIDAR/camera/GPS for P2/P3."""
from __future__ import annotations
import math, time
from dataclasses import dataclass, field
from compliance_profile import (FUSION_BEARING_WINDOW_DEG, FUSION_DEGRADED_SPEED_FACTOR,
    FUSION_ENABLED, FUSION_GATE_EVENT_CONFIRM_WINDOW_S, FUSION_HOLD_SPEED_MPS,
    FUSION_CONF_SLOW_THRESHOLD, FUSION_CONF_HOLD_THRESHOLD,
    FUSION_LIDAR_MIN_VALID_M, FUSION_LIDAR_STALE_TIMEOUT_S,
    FUSION_CAMERA_STALE_TIMEOUT_S, FUSION_GPS_STALE_TIMEOUT_S,
    KALMAN_HEADING_OUTLIER_CHI2, KALMAN_POSITION_OUTLIER_CHI2)
from kalman_filter import ExtendedKalmanFilter6D
from trajectory_tracker import DynamicObstacleTracker

@dataclass
class FusionState:
    timestamp: float = 0.0
    enabled: bool = FUSION_ENABLED
    policy: str = "lidar_primary_camera_assist"
    lidar_age_s: float = 999.0
    camera_age_s: float = 999.0
    gps_age_s: float = 999.0
    lidar_fresh: bool = False
    camera_fresh: bool = False
    gps_fresh: bool = False
    validated_obstacles: list = field(default_factory=list)
    dynamic_obstacles: list = field(default_factory=list)
    min_ttc_s: float | None = None
    ttc_level: str = "normal"
    ttc_speed_factor: float = 1.0
    validated_gate_center: dict | None = None
    validated_target_bearing: float | None = None
    wrong_target_risk: bool = False
    kalman_state: dict = field(default_factory=dict)
    gps_outlier_rejected: bool = False
    heading_outlier_rejected: bool = False
    fusion_confidence: float = 0.0
    confidence_breakdown: dict = field(default_factory=dict)
    degraded_mode: bool = False
    degraded_reason: str = ""
    hold_recommended: bool = False
    lidar_influence: str = "none"
    camera_influence: str = "none"
    decision_reason: str = "idle"

class SensorFusionLayer:
    def __init__(self):
        self.state = FusionState()
        self._last_lidar = self._last_cam = self._last_gps = 0.0
        self._gate_start = None
        self._gate_count = 0
        self._tracker = DynamicObstacleTracker()
        self._ekf = ExtendedKalmanFilter6D()
        self._gps_origin = None
        self._ekf_last_ts = 0.0
    
    def update_lidar(self, *, left_m, center_m, right_m, point_count, clusters, timestamp=None, boat_speed_mps=0.0):
        now = timestamp or time.monotonic()
        self._last_lidar = now
        self.state.lidar_age_s = now - self._last_lidar
        self.state.lidar_fresh = self.state.lidar_age_s <= FUSION_LIDAR_STALE_TIMEOUT_S
        validated = []
        for c in clusters:
            cx, cy, r = float(c.get("cx_m",0)), float(c.get("cy_m",0)), float(c.get("radius_m",0))
            dist = math.sqrt(cx*cx + cy*cy)
            if FUSION_LIDAR_MIN_VALID_M <= dist <= 50 and r <= 5.0:
                validated.append({"x_m": cx, "y_m": cy, "radius_m": r, "distance_m": dist, "source": "lidar", "confidence": 1.0})
        self.state.validated_obstacles = validated
        tracks = self._tracker.update(validated, now_s=now, boat_speed_mps=boat_speed_mps)
        summary = self._tracker.summary()
        self.state.dynamic_obstacles = [track.to_dict() for track in tracks[:8]]
        self.state.min_ttc_s = summary.get("min_ttc_s")
        self.state.ttc_level = str(summary.get("ttc_level", "normal"))
        self.state.ttc_speed_factor = float(summary.get("ttc_speed_factor", 1.0) or 1.0)
        self.state.timestamp = now
    
    def update_camera(self, *, gate_detected, gate_bearing_deg, gate_confidence,
                      target_detected, target_bearing_deg, target_area_ratio,
                      wrong_target_visible, wrong_target_bearing_deg, timestamp=None):
        now = timestamp or time.monotonic()
        self._last_cam = now
        self.state.camera_age_s = now - self._last_cam
        self.state.camera_fresh = self.state.camera_age_s <= FUSION_CAMERA_STALE_TIMEOUT_S
        if gate_detected and gate_confidence > 0.5:
            if self._gate_start is None:
                self._gate_start, self._gate_count = now, 1
            elif now - self._gate_start <= FUSION_GATE_EVENT_CONFIRM_WINDOW_S:
                self._gate_count += 1
            else:
                self._gate_start, self._gate_count = now, 1
            if self._gate_count >= 2:
                self.state.validated_gate_center = {"bearing_deg": gate_bearing_deg, "confidence": gate_confidence, "stable_s": now - self._gate_start}
                self.state.camera_influence = "gate_assist"
        else:
            self.state.validated_gate_center = None
            self._gate_start = self._gate_count = 0
        self.state.validated_target_bearing = target_bearing_deg if (target_detected and target_area_ratio > 0.01) else None
        self.state.wrong_target_risk = wrong_target_visible
        self.state.timestamp = now
    
    def update_gps(self, *, lat, lon, heading_deg, fix_type, timestamp=None):
        now = timestamp or time.monotonic()
        if self._ekf_last_ts > 0.0:
            self._ekf.predict(now - self._ekf_last_ts)
        self._ekf_last_ts = now
        self._last_gps = now
        self.state.gps_age_s = now - self._last_gps
        self.state.gps_fresh = self.state.gps_age_s <= FUSION_GPS_STALE_TIMEOUT_S and fix_type >= 3
        self.state.gps_outlier_rejected = False
        self.state.heading_outlier_rejected = False
        try:
            lat_f = float(lat)
            lon_f = float(lon)
            if self._gps_origin is None and lat_f != 0.0 and lon_f != 0.0:
                self._gps_origin = (lat_f, lon_f)
            if self._gps_origin is not None:
                lat0, lon0 = self._gps_origin
                meters_per_deg_lat = 111_320.0
                meters_per_deg_lon = meters_per_deg_lat * max(0.1, math.cos(math.radians(lat0)))
                x_m = (lat_f - lat0) * meters_per_deg_lat
                y_m = (lon_f - lon0) * meters_per_deg_lon
                if not self._ekf.initialized:
                    self._ekf.reset(x_m=x_m, y_m=y_m, heading_deg=float(heading_deg))
                else:
                    pos_update = self._ekf.update_position(
                        x_m,
                        y_m,
                        measurement_var=2.25,
                        chi2_threshold=KALMAN_POSITION_OUTLIER_CHI2,
                    )
                    self.state.gps_outlier_rejected = not bool(pos_update.accepted)
                    heading_update = self._ekf.update_heading(
                        float(heading_deg),
                        measurement_var=25.0,
                        chi2_threshold=KALMAN_HEADING_OUTLIER_CHI2,
                    )
                    self.state.heading_outlier_rejected = not bool(heading_update.accepted)
                self.state.kalman_state = self._ekf.to_dict()
                if self.state.gps_outlier_rejected:
                    self.state.gps_fresh = False
        except Exception:
            self.state.kalman_state = self._ekf.to_dict()
        self.state.timestamp = now
    
    def get_fusion_state(self):
        now = time.monotonic()
        if self._last_lidar > 0:
            self.state.lidar_age_s = now - self._last_lidar
            self.state.lidar_fresh = self.state.lidar_age_s <= FUSION_LIDAR_STALE_TIMEOUT_S
        if self._last_cam > 0:
            self.state.camera_age_s = now - self._last_cam
            self.state.camera_fresh = self.state.camera_age_s <= FUSION_CAMERA_STALE_TIMEOUT_S
        if self._last_gps > 0:
            self.state.gps_age_s = now - self._last_gps
            self.state.gps_fresh = self.state.gps_age_s <= FUSION_GPS_STALE_TIMEOUT_S
            if self.state.gps_outlier_rejected:
                self.state.gps_fresh = False
        # Adım 3 — Per-sensor güven (yaş bazlı 0-1) + sensörler arası tutarlılık.
        # fusion_confidence agrega korunur (geriye dönük uyumlu); breakdown zenginleşir.
        _lidar_conf = 0.0
        if self.state.lidar_fresh and FUSION_LIDAR_STALE_TIMEOUT_S > 0:
            _lidar_conf = max(0.0, 1.0 - (self.state.lidar_age_s / float(FUSION_LIDAR_STALE_TIMEOUT_S)))
        _camera_conf = 0.0
        if self.state.camera_fresh and FUSION_CAMERA_STALE_TIMEOUT_S > 0:
            _camera_conf = max(0.0, 1.0 - (self.state.camera_age_s / float(FUSION_CAMERA_STALE_TIMEOUT_S)))
        _gps_conf = 0.0
        if self.state.gps_fresh and FUSION_GPS_STALE_TIMEOUT_S > 0:
            _gps_conf = max(0.0, 1.0 - (self.state.gps_age_s / float(FUSION_GPS_STALE_TIMEOUT_S)))
        _fresh_count = int(sum([1 for x in (self.state.lidar_fresh, self.state.camera_fresh, self.state.gps_fresh) if x]))
        _sensor_consistency = _fresh_count / 3.0
        self.state.fusion_confidence = (0.5 if self.state.lidar_fresh else 0) + (0.3 if self.state.camera_fresh else 0) + (0.2 if self.state.gps_fresh else 0)
        self.state.confidence_breakdown = {
            "lidar": round(float(_lidar_conf), 3),
            "camera": round(float(_camera_conf), 3),
            "gps": round(float(_gps_conf), 3),
            "sensor_consistency": round(float(_sensor_consistency), 3),
            "fresh_count": int(_fresh_count),
        }
        self.state.degraded_mode = self.state.hold_recommended = False
        self.state.degraded_reason = ""
        if not self.state.lidar_fresh:
            self.state.degraded_mode, self.state.degraded_reason = True, "lidar_stale"
            self.state.lidar_influence = "stale_speed_reduction"
            if self.state.lidar_age_s > FUSION_LIDAR_STALE_TIMEOUT_S * 2:
                self.state.hold_recommended, self.state.decision_reason = True, "lidar_stale_hold"
        if not self.state.camera_fresh:
            self.state.degraded_mode = True
            if not self.state.degraded_reason: self.state.degraded_reason = "camera_stale"
            self.state.camera_influence = "stale_no_bias"
            self.state.validated_gate_center = self.state.validated_target_bearing = None
        if not self.state.gps_fresh:
            self.state.degraded_mode = True
            if not self.state.degraded_reason: self.state.degraded_reason = "gps_stale"
            if self.state.gps_age_s > FUSION_GPS_STALE_TIMEOUT_S * 2:
                self.state.hold_recommended, self.state.decision_reason = True, "gps_stale_hold"
        if not self.state.degraded_mode:
            self.state.lidar_influence = "obstacle_avoidance" if (self.state.lidar_fresh and self.state.validated_obstacles) else "none"
            self.state.camera_influence = ("gate_assist" if self.state.validated_gate_center else "target_tracking" if self.state.validated_target_bearing is not None else "none") if self.state.camera_fresh else "none"
            self.state.decision_reason = "normal_fusion"
        if self.state.ttc_level in ("stop", "emergency"):
            self.state.degraded_mode = True
            self.state.hold_recommended = True
            self.state.degraded_reason = "ttc_emergency"
            self.state.decision_reason = "dynamic_obstacle_ttc_hold"
        elif self.state.ttc_level in ("danger", "warn") and not self.state.hold_recommended:
            self.state.degraded_mode = True
            self.state.degraded_reason = f"ttc_{self.state.ttc_level}"
            self.state.decision_reason = "dynamic_obstacle_ttc_slow"
        # Adım 3 — Merkezî güven eşikleriyle degrade/HOLD ağacı (mevcut staleness/ttc
        # ağacını tamamlayıcı; yalnızca ekler, hiçbir kararı değiştirmez/override etmez).
        # Toplam sensör kaybı (confidence < HOLD) → anında HOLD (2x age beklenmeden).
        if not self.state.hold_recommended and self.state.fusion_confidence < float(FUSION_CONF_HOLD_THRESHOLD):
            self.state.hold_recommended = True
            self.state.degraded_mode = True
            if not self.state.degraded_reason:
                self.state.degraded_reason = "low_confidence_hold"
            self.state.decision_reason = "low_confidence_hold"
        elif not self.state.degraded_mode and self.state.fusion_confidence < float(FUSION_CONF_SLOW_THRESHOLD):
            self.state.degraded_mode = True
            if not self.state.degraded_reason:
                self.state.degraded_reason = "low_confidence_slow"
            if self.state.decision_reason == "normal_fusion":
                self.state.decision_reason = "low_confidence_slow"
        return self.state
    
    def get_speed_factor(self):
        state = self.get_fusion_state()
        return FUSION_HOLD_SPEED_MPS if state.hold_recommended else FUSION_DEGRADED_SPEED_FACTOR if state.degraded_mode else 1.0
    
    def correlate_lidar_camera_obstacle(self, lidar_obstacle, camera_bearing_deg, heading_deg):
        dx, dy = float(lidar_obstacle.get("x_m",0)), float(lidar_obstacle.get("y_m",0))
        lidar_world = (heading_deg + math.degrees(math.atan2(dy, dx))) % 360.0
        diff = abs(lidar_world - camera_bearing_deg)
        if diff > 180.0: diff = 360.0 - diff
        return diff <= FUSION_BEARING_WINDOW_DEG

_fusion_layer = None
def get_sensor_fusion_layer():
    global _fusion_layer
    if _fusion_layer is None: _fusion_layer = SensorFusionLayer()
    return _fusion_layer
