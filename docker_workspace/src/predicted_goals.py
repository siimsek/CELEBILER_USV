"""Predicted Goals Module for CELEBILER USV.

Generates predicted trajectory and goals for P2/P3 phases.
Logs to predicted_goals.jsonl for race delivery compliance.
"""
from __future__ import annotations
import json
import math
import time
from pathlib import Path
from typing import Any
from compliance_profile import (
    PREDICTED_GOALS_LOG_HZ, PREDICTED_TRAJECTORY_HORIZON_S,
    PREDICTED_TRAJECTORY_STEP_S, LOG_DIR,
)

class PredictedGoalsLogger:
    """Generate and log predicted trajectory and goals."""
    def __init__(self, log_dir: str | None = None):
        self.log_dir = Path(log_dir or LOG_DIR)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / "predicted_goals.jsonl"
        self._last_log_time = 0.0
        self._log_interval = 1.0 / PREDICTED_GOALS_LOG_HZ
    
    def predict_trajectory(self, *, current_x_m: float, current_y_m: float,
                          current_heading_deg: float, speed_mps: float,
                          heading_target_deg: float | None = None,
                          dt_s: float = PREDICTED_TRAJECTORY_STEP_S,
                          horizon_s: float = PREDICTED_TRAJECTORY_HORIZON_S) -> list[tuple[float, float]]:
        """
        Predict trajectory based on current state and target heading.
        Returns list of (x_m, y_m) points.
        """
        if heading_target_deg is None:
            heading_target_deg = current_heading_deg
        trajectory = []
        x, y = current_x_m, current_y_m
        heading_rad = math.radians(current_heading_deg)
        heading_target_rad = math.radians(heading_target_deg)
        steps = int(horizon_s / dt_s)
        for _ in range(steps):
            # Smooth heading transition
            heading_diff = heading_target_rad - heading_rad
            while heading_diff > math.pi: heading_diff -= 2 * math.pi
            while heading_diff < -math.pi: heading_diff += 2 * math.pi
            heading_rad += heading_diff * min(1.0, dt_s * 2.0)  # Smooth turn
            # Update position
            x += speed_mps * math.cos(heading_rad) * dt_s
            y += speed_mps * math.sin(heading_rad) * dt_s
            trajectory.append((round(x, 3), round(y, 3)))
        return trajectory
    
    def log_predicted_goals(self, *, state: str, frame: str,
                           global_goal_lat: float, global_goal_lon: float,
                           local_goal_x_m: float, local_goal_y_m: float,
                           selected_subgoal_x_m: float, selected_subgoal_y_m: float,
                           trajectory_m: list[tuple[float, float]],
                           confidence: float, reason: str,
                           timestamp: float | None = None) -> None:
        """Log predicted goals to JSONL file."""
        now = timestamp or time.monotonic()
        if now - self._last_log_time < self._log_interval:
            return
        self._last_log_time = now
        record = {
            "timestamp": round(time.time(), 3),
            "state": state,
            "frame": frame,
            "global_goal": {"lat": round(global_goal_lat, 7), "lon": round(global_goal_lon, 7)},
            "local_goal_m": {"x": round(local_goal_x_m, 3), "y": round(local_goal_y_m, 3)},
            "selected_subgoal_m": {"x": round(selected_subgoal_x_m, 3), "y": round(selected_subgoal_y_m, 3)},
            "trajectory_m": [[round(p[0], 3), round(p[1], 3)] for p in trajectory_m],
            "confidence": round(confidence, 3),
            "reason": reason,
        }
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception as e:
            print(f"[PRED_GOALS] Log write error: {e}")

_predicted_goals_logger: PredictedGoalsLogger | None = None

def get_predicted_goals_logger(log_dir: str | None = None) -> PredictedGoalsLogger:
    """Get or create the global predicted goals logger instance."""
    global _predicted_goals_logger
    if _predicted_goals_logger is None:
        _predicted_goals_logger = PredictedGoalsLogger(log_dir)
    return _predicted_goals_logger