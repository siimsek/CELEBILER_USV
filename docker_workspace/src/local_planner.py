"""Predictive local planner for P2/P3 GUIDED setpoints.

The planner works in the vehicle ``base_link`` frame:
``x`` is forward, ``y`` is left. Project heading errors use the existing USV
convention where positive heading error means turn right, so trajectory
projection maps heading_error to ``-body_angle``.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from compliance_profile import (
    COSTMAP_INFLATED_COST,
    PLANNER_APPROACH_SPEED_MPS,
    PLANNER_CONFIDENCE_HOLD_THRESHOLD,
    PLANNER_CONFIDENCE_SLOW_THRESHOLD,
    PLANNER_HEADING_RATE_LIMIT_DEG,
    PLANNER_HEADING_STEP_DEG,
    PLANNER_HORIZON_S,
    PLANNER_HYSTERESIS_SCORE_MARGIN,
    PLANNER_MAX_HEADING_ERROR_DEG,
    PLANNER_MAX_SPEED_P2_MPS,
    PLANNER_MAX_SPEED_P3_MPS,
    PLANNER_MIN_CLEARANCE_NORMAL_M,
    PLANNER_SMOOTHING_ALPHA,
    PLANNER_STUCK_PROGRESS_EPS_M,
    PLANNER_STUCK_TIMEOUT_S,
    PLANNER_TRAJECTORY_LOG_LIMIT,
    PLANNER_W_BOUNDARY,
    PLANNER_W_COLLISION,
    PLANNER_W_GATE,
    PLANNER_W_PROGRESS,
    PLANNER_W_SENSOR,
    PLANNER_W_SMOOTHNESS,
    PLANNER_W_WRONG_TARGET,
    PLANNER_GATE_CENTER_BONUS,
    PLANNER_CORRIDOR_HYSTERESIS_BONUS,
    PREDICTED_TRAJECTORY_HORIZON_S,
    PREDICTED_TRAJECTORY_STEP_S,
)
from obstacle_avoidance import speed_profile_for_ttc


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _wrap_180(deg: float) -> float:
    value = (float(deg) + 180.0) % 360.0 - 180.0
    if value == -180.0:
        return 180.0
    return value


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
        if math.isfinite(parsed):
            return parsed
    except (TypeError, ValueError):
        pass
    return float(default)


@dataclass
class PlannerDecision:
    """Output of the local planner for one cycle."""

    timestamp: float = 0.0
    state: str = ""
    global_bearing_deg: float = 0.0
    selected_heading_deg: float = 0.0
    selected_heading_error_deg: float = 0.0
    v_target_mps: float = 0.0
    confidence: float = 0.0
    confidence_breakdown: dict = field(default_factory=dict)
    cost_breakdown: dict = field(default_factory=dict)
    decision_reason: str = ""
    selected_subgoal_m: dict = field(default_factory=dict)
    predicted_trajectory_m: list = field(default_factory=list)
    guidance_source: str = ""
    collision_detected: bool = False
    min_clearance_m: float = 99.0
    corridor: str = "center"
    wrong_target_risk: bool = False
    degraded: bool = False
    degraded_reason: str = ""
    active_behavior: str = "HOLD"
    previous_behavior: str = "HOLD"
    candidate_count: int = 0
    rejected_collision: int = 0
    rejected_boundary: int = 0
    candidates: list = field(default_factory=list)
    selected_candidate_score: float = 0.0
    speed_factor: float = 1.0
    dynamic_ttc_s: float | None = None
    ttc_level: str = "normal"
    stuck_detected: bool = False
    recovery_action: str = "none"
    gate_candidate: dict = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "state": self.state,
            "global_bearing_deg": round(self.global_bearing_deg, 2),
            "selected_heading_deg": round(self.selected_heading_deg, 2),
            "selected_heading_error_deg": round(self.selected_heading_error_deg, 2),
            "v_target_mps": round(self.v_target_mps, 3),
            "confidence": round(self.confidence, 3),
            "confidence_breakdown": self.confidence_breakdown,
            "cost_breakdown": self.cost_breakdown,
            "decision_reason": self.decision_reason,
            "selected_subgoal_m": self.selected_subgoal_m,
            "predicted_trajectory_m": self.predicted_trajectory_m,
            "guidance_source": self.guidance_source,
            "collision_detected": self.collision_detected,
            "min_clearance_m": round(self.min_clearance_m, 3),
            "corridor": self.corridor,
            "wrong_target_risk": self.wrong_target_risk,
            "degraded": self.degraded,
            "degraded_reason": self.degraded_reason,
            "active_behavior": self.active_behavior,
            "previous_behavior": self.previous_behavior,
            "candidate_count": int(self.candidate_count),
            "rejected_collision": int(self.rejected_collision),
            "rejected_boundary": int(self.rejected_boundary),
            "candidates": list(self.candidates),
            "selected_candidate_score": round(float(self.selected_candidate_score), 3),
            "speed_factor": round(float(self.speed_factor), 3),
            "dynamic_ttc_s": round(float(self.dynamic_ttc_s), 3) if self.dynamic_ttc_s is not None else None,
            "ttc_level": str(self.ttc_level),
            "stuck_detected": bool(self.stuck_detected),
            "recovery_action": str(self.recovery_action),
            "gate_candidate": dict(self.gate_candidate),
        }


class PredictiveLocalPlanner:
    """Candidate-heading local planner with collision checking and hysteresis."""

    def __init__(self) -> None:
        self._last_heading_error_deg = 0.0
        self._last_corridor = "center"
        self._last_behavior = "HOLD"
        self._last_progress_m = None
        self._last_distance_m = None
        self._stuck_since = None
        self._last_decision_ts = 0.0

    @staticmethod
    def _corridor_for_error(heading_error_deg: float) -> str:
        err = float(heading_error_deg)
        if err <= -10.0:
            return "left"
        if err >= 10.0:
            return "right"
        return "center"

    @staticmethod
    def _predict_body_trajectory(
        *,
        heading_error_deg: float,
        speed_mps: float,
        horizon_s: float = PREDICTED_TRAJECTORY_HORIZON_S,
        step_s: float = PREDICTED_TRAJECTORY_STEP_S,
    ) -> list[tuple[float, float]]:
        speed = max(0.0, float(speed_mps))
        body_angle_rad = math.radians(-float(heading_error_deg))
        x_m = 0.0
        y_m = 0.0
        trajectory: list[tuple[float, float]] = []
        steps = max(1, int(float(horizon_s) / max(0.05, float(step_s))))
        for _ in range(steps):
            x_m += speed * math.cos(body_angle_rad) * float(step_s)
            y_m += speed * math.sin(body_angle_rad) * float(step_s)
            trajectory.append((round(x_m, 3), round(y_m, 3)))
        return trajectory

    def _candidate_errors(self, target_error_deg: float, gate_error_deg: float | None) -> list[float]:
        max_err = float(PLANNER_MAX_HEADING_ERROR_DEG)
        step = max(5.0, min(20.0, float(PLANNER_HEADING_STEP_DEG)))
        seeds = {0.0, _clamp(target_error_deg, -max_err, max_err)}
        if gate_error_deg is not None:
            seeds.add(_clamp(gate_error_deg, -max_err, max_err))
        for offset in (-60.0, -45.0, -30.0, -15.0, 15.0, 30.0, 45.0, 60.0):
            seeds.add(_clamp(float(target_error_deg) + offset, -max_err, max_err))
        count_each_side = int(max_err / step)
        for idx in range(-count_each_side, count_each_side + 1):
            seeds.add(round(float(idx) * step, 3))
        return sorted(seeds)

    def _detect_stuck(self, *, progress_m: float, distance_m: float, commanded_speed_mps: float, now: float) -> bool:
        progress = _finite(progress_m, 0.0)
        distance = _finite(distance_m, 9999.0)
        speed = _finite(commanded_speed_mps, 0.0)
        if speed < 0.20:
            self._stuck_since = None
            self._last_progress_m = progress
            self._last_distance_m = distance
            return False
        progress_ok = False
        if self._last_progress_m is not None:
            progress_ok = progress > (float(self._last_progress_m) + float(PLANNER_STUCK_PROGRESS_EPS_M))
        distance_ok = False
        if self._last_distance_m is not None:
            distance_ok = distance < (float(self._last_distance_m) - float(PLANNER_STUCK_PROGRESS_EPS_M))
        if progress_ok or distance_ok or self._last_progress_m is None:
            self._stuck_since = None
            self._last_progress_m = progress
            self._last_distance_m = distance
            return False
        if self._stuck_since is None:
            self._stuck_since = now
            return False
        return (now - float(self._stuck_since)) >= float(PLANNER_STUCK_TIMEOUT_S)

    def decide(
        self,
        *,
        state: str,
        current_heading_deg: float,
        global_heading_error_deg: float,
        base_speed_mps: float,
        current_speed_mps: float,
        progress_m: float,
        distance_m: float,
        costmap: Any,
        costmap_snapshot: dict | None,
        fusion_state: dict | None = None,
        gate_bearing_deg: float | None = None,
        gate_confidence: float = 0.0,
        wrong_target_risk: bool = False,
        wrong_target_bearing_deg: float | None = None,
        target_visible: bool = False,
        now: float | None = None,
        boat_x_m: float | None = None,
        boat_y_m: float | None = None,
    ) -> PlannerDecision:
        now_mono = time.monotonic() if now is None else float(now)
        current_heading = _finite(current_heading_deg, 0.0) % 360.0
        target_error = _clamp(_wrap_180(global_heading_error_deg), -PLANNER_MAX_HEADING_ERROR_DEG, PLANNER_MAX_HEADING_ERROR_DEG)
        base_speed = _clamp(base_speed_mps, 0.0, max(float(PLANNER_MAX_SPEED_P2_MPS), float(PLANNER_MAX_SPEED_P3_MPS)))
        state_name = str(state or "")
        max_speed = float(PLANNER_MAX_SPEED_P3_MPS if "P3" in state_name else PLANNER_MAX_SPEED_P2_MPS)
        base_speed = min(base_speed, max_speed)

        snapshot = costmap_snapshot if isinstance(costmap_snapshot, dict) else {}
        fusion = fusion_state if isinstance(fusion_state, dict) else {}
        costmap_fresh = bool(snapshot.get("fresh", False))
        sector = snapshot.get("sector", {}) if isinstance(snapshot.get("sector", {}), dict) else {}
        sensor_conf = _clamp(fusion.get("fusion_confidence", 1.0 if costmap_fresh else 0.35), 0.0, 1.0)
        degraded_reason = str(fusion.get("degraded_reason", "costmap_stale" if not costmap_fresh else ""))
        hold_recommended = bool(fusion.get("hold_recommended", False))
        gate_conf = _clamp(gate_confidence, 0.0, 1.0)
        gate_error = _finite(gate_bearing_deg, 0.0) if gate_bearing_deg is not None and gate_conf >= 0.45 else None
        wrong_bearing = _finite(wrong_target_bearing_deg, 0.0) if wrong_target_bearing_deg is not None else None
        min_ttc_raw = fusion.get("min_ttc_s")
        min_ttc_s = _finite(min_ttc_raw, 9999.0) if min_ttc_raw is not None else None
        ttc_profile = speed_profile_for_ttc(base_speed, min_ttc_s)
        stuck = self._detect_stuck(
            progress_m=progress_m,
            distance_m=distance_m,
            commanded_speed_mps=base_speed,
            now=now_mono,
        )

        candidates = []
        rejected_collision = 0
        rejected_boundary = 0
        for error_deg in self._candidate_errors(target_error, gate_error):
            turn_abs = abs(float(error_deg))
            turn_speed_factor = 1.0 - (0.55 * _clamp(turn_abs / max(PLANNER_MAX_HEADING_ERROR_DEG, 1.0), 0.0, 1.0))
            candidate_speed = max(float(PLANNER_APPROACH_SPEED_MPS), base_speed * turn_speed_factor)
            candidate_speed = min(base_speed, candidate_speed, max_speed)
            trajectory = self._predict_body_trajectory(
                heading_error_deg=error_deg,
                speed_mps=max(candidate_speed, 0.20),
                horizon_s=min(float(PLANNER_HORIZON_S), float(PREDICTED_TRAJECTORY_HORIZON_S)),
            )
            collision = False
            min_clearance = 99.0
            if costmap is not None and hasattr(costmap, "check_trajectory_collision"):
                collision, min_clearance = costmap.check_trajectory_collision(
                    trajectory,
                    threshold=COSTMAP_INFLATED_COST,
                )
            if collision:
                rejected_collision += 1
            if min_clearance <= 0.0:
                rejected_boundary += 1
            progress_cost = abs(_wrap_180(error_deg - target_error)) / 90.0
            smooth_cost = abs(_wrap_180(error_deg - self._last_heading_error_deg)) / 90.0
            boundary_cost = _clamp((turn_abs - 45.0) / 45.0, 0.0, 1.0)
            gate_cost = 0.0
            if gate_error is not None:
                gate_cost = gate_conf * abs(_wrap_180(error_deg - gate_error)) / 90.0
            wrong_cost = 0.0
            if wrong_target_risk:
                wrong_cost = 1.0
                if wrong_bearing is not None:
                    wrong_cost = max(0.0, 1.0 - (abs(_wrap_180(error_deg - wrong_bearing)) / 75.0))
            clearance_cost = 0.0 if min_clearance >= PLANNER_MIN_CLEARANCE_NORMAL_M else (
                (float(PLANNER_MIN_CLEARANCE_NORMAL_M) - float(min_clearance)) / max(float(PLANNER_MIN_CLEARANCE_NORMAL_M), 0.1)
            )
            score = (
                (float(PLANNER_W_COLLISION) if collision else 0.0)
                + (float(PLANNER_W_COLLISION) * 0.25 * _clamp(clearance_cost, 0.0, 1.0))
                + (float(PLANNER_W_PROGRESS) * progress_cost)
                + (float(PLANNER_W_SMOOTHNESS) * smooth_cost)
                + (float(PLANNER_W_BOUNDARY) * boundary_cost)
                + (float(PLANNER_W_GATE) * gate_cost)
                + (float(PLANNER_W_SENSOR) * (1.0 - sensor_conf))
                + (float(PLANNER_W_WRONG_TARGET) * wrong_cost)
            )
            corridor = self._corridor_for_error(error_deg)
            if corridor == self._last_corridor:
                score -= float(PLANNER_CORRIDOR_HYSTERESIS_BONUS)
            # Adım 2 — Gate-center sub-goal güçlendirme: gate yüksek güvenli + merkez
            # sektör açıksa, kapı bearing'ine en yakın adaya ek bonus → dar koridorda
            # merkezlenme. Yalnızca çarpışmasız adalar için; mevcut gate_cost'u güçlendirir.
            if (
                gate_error is not None
                and gate_conf >= 0.60
                and bool(sector.get("center_clear", True))
                and not collision
                and abs(_wrap_180(error_deg - gate_error)) <= max(float(PLANNER_HEADING_STEP_DEG), 5.0)
            ):
                score -= float(PLANNER_GATE_CENTER_BONUS)
            candidates.append(
                {
                    "heading_error_deg": round(float(error_deg), 3),
                    "heading_deg": round(float((current_heading + error_deg) % 360.0), 3),
                    "speed_mps": round(float(candidate_speed), 3),
                    "score": round(float(score), 4),
                    "collision": bool(collision),
                    "min_clearance_m": round(float(min_clearance), 3),
                    "corridor": corridor,
                    "cost": {
                        "progress": round(float(progress_cost), 4),
                        "smoothness": round(float(smooth_cost), 4),
                        "boundary": round(float(boundary_cost), 4),
                        "gate": round(float(gate_cost), 4),
                        "wrong_target": round(float(wrong_cost), 4),
                        "clearance": round(float(clearance_cost), 4),
                    },
                    "trajectory_m": [[round(x, 3), round(y, 3)] for x, y in trajectory[:PLANNER_TRAJECTORY_LOG_LIMIT]],
                }
            )

        if stuck and candidates:
            preferred_sign = 1.0 if self._last_corridor == "left" else -1.0
            alternates = [
                c for c in candidates
                if not c["collision"] and math.copysign(1.0, float(c["heading_error_deg"]) or preferred_sign) == preferred_sign
            ]
            if alternates:
                candidates = alternates

        non_collision = [c for c in candidates if not c["collision"]]
        pool = non_collision or candidates
        selected = min(pool, key=lambda c: (float(c["score"]), abs(float(c["heading_error_deg"])))) if pool else None
        if selected is None:
            selected = {
                "heading_error_deg": 0.0,
                "heading_deg": current_heading,
                "speed_mps": 0.0,
                "score": float(PLANNER_W_COLLISION),
                "collision": True,
                "min_clearance_m": 0.0,
                "corridor": "blocked",
                "cost": {},
                "trajectory_m": [],
            }

        target_sign = 1 if target_error > 1.0 else (-1 if target_error < -1.0 else 0)
        previous_match = None
        for c in pool:
            candidate_error = float(c["heading_error_deg"])
            candidate_sign = 1 if candidate_error > 1.0 else (-1 if candidate_error < -1.0 else 0)
            if (
                target_sign
                and candidate_sign
                and candidate_sign != target_sign
                and abs(float(target_error)) >= 20.0
                and not bool(c.get("collision"))
            ):
                continue
            if abs(_wrap_180(candidate_error - self._last_heading_error_deg)) <= max(5.0, PLANNER_HEADING_RATE_LIMIT_DEG):
                previous_match = c
                break
        if (
            previous_match is not None
            and float(previous_match["score"]) <= float(selected["score"]) + float(PLANNER_HYSTERESIS_SCORE_MARGIN)
        ):
            selected = previous_match

        # P3 target collision bypass: Charge directly at correct target, ignoring costmap collisions
        if "P3" in state_name and target_visible and not wrong_target_risk:
            selected = {
                "heading_error_deg": target_error,
                "heading_deg": (current_heading + target_error) % 360.0,
                "speed_mps": base_speed,
                "score": 0.0,
                "collision": False,
                "min_clearance_m": 99.0,
                "corridor": "center",
                "cost": {},
                "trajectory_m": [],
            }

        selected_error = _clamp(float(selected["heading_error_deg"]), -PLANNER_MAX_HEADING_ERROR_DEG, PLANNER_MAX_HEADING_ERROR_DEG)
        alpha = _clamp(PLANNER_SMOOTHING_ALPHA, 0.05, 1.0)
        smoothed_error = _wrap_180(self._last_heading_error_deg + (alpha * _wrap_180(selected_error - self._last_heading_error_deg)))
        if bool(selected.get("collision")):
            smoothed_error = selected_error
        elif (
            target_sign
            and abs(float(target_error)) >= 20.0
            and (smoothed_error * target_error) < 0.0
            and not bool(wrong_target_risk)
        ):
            max_delta = max(10.0, float(PLANNER_HEADING_RATE_LIMIT_DEG))
            bounded_target = _clamp(target_error, -max_delta, max_delta)
            smoothed_error = float(bounded_target if abs(bounded_target) >= 1.0 else target_error)
        selected_heading = (current_heading + smoothed_error) % 360.0
        min_clearance = _finite(selected.get("min_clearance_m"), 99.0)
        clearance_conf = _clamp(min_clearance / max(float(PLANNER_MIN_CLEARANCE_NORMAL_M), 0.1), 0.0, 1.0)
        collision_conf = 0.0 if bool(selected.get("collision")) else 1.0
        stability_conf = 1.0 - _clamp(abs(_wrap_180(smoothed_error - self._last_heading_error_deg)) / max(PLANNER_MAX_HEADING_ERROR_DEG, 1.0), 0.0, 1.0)
        confidence = _clamp(
            (0.42 * sensor_conf) + (0.30 * clearance_conf) + (0.18 * collision_conf) + (0.10 * stability_conf),
            0.0,
            1.0,
        )

        speed_factor = 1.0
        active_behavior = "FOLLOW_WAYPOINT"
        decision_reason = "clear_waypoint_progress"
        selected_collision = bool(selected.get("collision"))
        selected_corridor = str(selected.get("corridor", "center"))
        sector_min = min(
            _finite(sector.get("left_min_m"), 99.0),
            _finite(sector.get("center_min_m"), 99.0),
            _finite(sector.get("right_min_m"), 99.0),
        )
        is_blocked = (snapshot.get("selected_corridor") == "blocked")
        is_near_collision = bool(sector.get("near_collision", False))
        
        has_landmark_threat = False
        if not costmap_fresh and boat_x_m is not None and boat_y_m is not None:
            try:
                from obstacle_landmarks import get_obstacle_landmark_map
                lm_map = get_obstacle_landmark_map()
                threats = lm_map.get_threat_landmarks(boat_x_m, boat_y_m, threat_radius_m=7.0)
                if threats:
                    has_landmark_threat = True
            except Exception:
                pass

        obstacle_evidence = bool(
            selected_collision
            or rejected_collision > 0
            or min_clearance < float(PLANNER_MIN_CLEARANCE_NORMAL_M)
            or (costmap_fresh and sector_min < float(PLANNER_MIN_CLEARANCE_NORMAL_M))
            or (costmap_fresh and not bool(sector.get("center_clear", True)))
            or is_blocked
            or is_near_collision
            or has_landmark_threat
        )
        if hold_recommended or confidence < float(PLANNER_CONFIDENCE_HOLD_THRESHOLD):
            active_behavior = "DEGRADED_SENSOR"
            decision_reason = "confidence_hold" if not hold_recommended else "fusion_hold_recommended"
            speed_factor = 0.0
        elif "P3" in state_name and target_visible and not wrong_target_risk:
            active_behavior = "ENGAGE_TARGET"
            decision_reason = "target_visible_approach"
            speed_factor = 0.80
        elif selected_collision or min_clearance < 0.75 or is_near_collision:
            active_behavior = "STOP_FOR_OBSTACLE"
            decision_reason = "predicted_collision" if not is_near_collision else "costmap_near_collision"
            speed_factor = 0.0
        elif wrong_target_risk:
            active_behavior = "WRONG_TARGET_AVOID"
            decision_reason = "wrong_target_obstacle"
            speed_factor = 0.35
        elif stuck:
            active_behavior = "RECOVERY"
            decision_reason = "stuck_switch_corridor"
            speed_factor = 0.45
        elif obstacle_evidence:
            active_behavior = "AVOID_OBSTACLE"
            decision_reason = "safe_corridor_selected" if not is_blocked else "costmap_blocked_avoidance"
            if has_landmark_threat:
                decision_reason = "landmark_threat_avoidance"
            speed_factor = 0.75
        elif gate_error is not None:
            active_behavior = "FOLLOW_GATE"
            decision_reason = "gate_center_bias"
            speed_factor = 0.75
        elif "P3" in state_name and target_visible:
            active_behavior = "ENGAGE_TARGET"
            decision_reason = "target_visible_approach"
            speed_factor = 0.80
        if ttc_profile.active:
            speed_factor = min(float(speed_factor), float(ttc_profile.speed_factor))
            if ttc_profile.level in ("stop", "emergency"):
                active_behavior = "STOP_FOR_OBSTACLE"
                decision_reason = str(ttc_profile.reason)
            elif active_behavior in ("FOLLOW_WAYPOINT", "FOLLOW_GATE", "ENGAGE_TARGET"):
                active_behavior = "AVOID_OBSTACLE"
                decision_reason = str(ttc_profile.reason)
        if confidence < float(PLANNER_CONFIDENCE_SLOW_THRESHOLD) and speed_factor > 0.0:
            speed_factor = min(speed_factor, 0.45)
            if active_behavior == "FOLLOW_WAYPOINT":
                active_behavior = "DEGRADED_SENSOR"
                decision_reason = "confidence_slow"

        selected_speed = min(float(selected.get("speed_mps", base_speed)), base_speed, max_speed) * speed_factor
        if speed_factor > 0.0:
            selected_speed = max(0.0, selected_speed)
        else:
            selected_speed = 0.0

        recovery_action = "none"
        if stuck:
            recovery_action = f"switch_corridor_{self._last_corridor}_to_{selected_corridor}"

        decision = PlannerDecision(
            timestamp=round(time.time(), 3),
            state=state_name,
            global_bearing_deg=round(float((current_heading + target_error) % 360.0), 3),
            selected_heading_deg=round(float(selected_heading), 3),
            selected_heading_error_deg=round(float(smoothed_error), 3),
            v_target_mps=round(float(selected_speed), 3),
            confidence=round(float(confidence), 3),
            confidence_breakdown={
                "sensor": round(float(sensor_conf), 3),
                "clearance": round(float(clearance_conf), 3),
                "collision_free": round(float(collision_conf), 3),
                "stability": round(float(stability_conf), 3),
            },
            cost_breakdown=dict(selected.get("cost", {})),
            decision_reason=decision_reason,
            selected_subgoal_m=self._subgoal_from_heading(smoothed_error, selected_speed),
            predicted_trajectory_m=list(selected.get("trajectory_m", [])),
            guidance_source="predictive_local_planner",
            collision_detected=bool(selected_collision),
            min_clearance_m=float(min_clearance),
            corridor=selected_corridor,
            wrong_target_risk=bool(wrong_target_risk),
            degraded=bool(hold_recommended or confidence < float(PLANNER_CONFIDENCE_SLOW_THRESHOLD)),
            degraded_reason=degraded_reason if degraded_reason else ("low_confidence" if confidence < float(PLANNER_CONFIDENCE_SLOW_THRESHOLD) else ""),
            active_behavior=active_behavior,
            previous_behavior=str(self._last_behavior),
            candidate_count=len(candidates),
            rejected_collision=int(rejected_collision),
            rejected_boundary=int(rejected_boundary),
            candidates=sorted(candidates, key=lambda c: float(c["score"]))[: int(PLANNER_TRAJECTORY_LOG_LIMIT)],
            selected_candidate_score=float(selected.get("score", 0.0)),
            speed_factor=float(speed_factor),
            dynamic_ttc_s=float(min_ttc_s) if min_ttc_s is not None and math.isfinite(float(min_ttc_s)) else None,
            ttc_level=str(ttc_profile.level),
            stuck_detected=bool(stuck),
            recovery_action=recovery_action,
            gate_candidate={
                "bearing_deg": round(float(gate_error), 3) if gate_error is not None else None,
                "confidence": round(float(gate_conf), 3),
                "safe_to_pass": bool(gate_error is not None and not selected_collision and confidence >= 0.5),
            },
        )
        self._last_heading_error_deg = float(smoothed_error)
        self._last_corridor = selected_corridor
        self._last_behavior = active_behavior
        self._last_decision_ts = now_mono
        return decision

    @staticmethod
    def _subgoal_from_heading(heading_error_deg: float, speed_mps: float) -> dict[str, float]:
        lookahead_s = _clamp(float(PLANNER_HORIZON_S) * 0.4, 1.0, 3.0)
        distance = max(1.0, float(speed_mps) * lookahead_s)
        body_angle_rad = math.radians(-float(heading_error_deg))
        return {
            "x": round(distance * math.cos(body_angle_rad), 3),
            "y": round(distance * math.sin(body_angle_rad), 3),
        }
