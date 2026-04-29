"""
Shared twin-thruster motor controller primitives.

The runtime state machine owns safety and MAVLink I/O. This module owns the
neutral/clamp/mix/ramp contract so sim and real paths use one PWM profile:
1500 neutral, 1100 reverse, 1900 forward.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

from compliance_profile import (
    PWM_DEADBAND_US,
    PWM_JERK_LIMIT_US_PER_S2,
    PWM_MAX_US,
    PWM_MIN_EFFECTIVE_US,
    PWM_MIN_US,
    PWM_NEUTRAL_US,
    PWM_SLEW_RATE_US_PER_S,
    PWM_TRIM_ERR_BAND_DEG,
    PWM_TRIM_LEARN_GAIN,
    PWM_TRIM_MAX_US,
)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass(frozen=True)
class MotorMix:
    left_pwm: int
    right_pwm: int
    surge_norm: float
    yaw_norm: float
    clipped: bool
    limit_reason: str


def mix_twin_thrusters(
    *,
    speed_mps: float,
    heading_error_deg: float,
    neutral_pwm: int = PWM_NEUTRAL_US,
    pwm_span: int | None = None,
    max_speed_mps: float = 1.5,
) -> MotorMix:
    """Map speed + heading error to raw left/right PWM before ramp hardening."""
    neutral = int(neutral_pwm)
    span = int(pwm_span if pwm_span is not None else max(abs(PWM_MAX_US - neutral), abs(neutral - PWM_MIN_US)))
    speed = float(speed_mps)
    heading_error = float(heading_error_deg)
    heading_abs = abs(heading_error)

    surge_norm = clamp(speed / max(float(max_speed_mps), 0.1), -1.0, 1.0)
    yaw_norm = clamp(heading_error / 72.0, -1.0, 1.0)
    limit_reason = "nominal"

    if speed > 0.0:
        if heading_abs >= 75.0:
            surge_norm = min(surge_norm, 0.32)
            limit_reason = "hard_turn_speed_cap"
        elif heading_abs >= 50.0:
            surge_norm = min(surge_norm, 0.48)
            limit_reason = "medium_turn_speed_cap"
        elif heading_abs >= 25.0:
            surge_norm = min(surge_norm, 0.68)
            limit_reason = "soft_turn_speed_cap"
        elif heading_abs >= 10.0:
            surge_norm = min(surge_norm, 0.52)
            limit_reason = "align_turn_speed_cap"
        if 0.0 < surge_norm < 0.14:
            surge_norm = 0.14

    yaw_gain = 0.92 - (0.22 * abs(surge_norm))
    left_mix = surge_norm - (yaw_norm * yaw_gain)
    right_mix = surge_norm + (yaw_norm * yaw_gain)

    if speed > 0.0 and heading_abs < 70.0:
        left_mix = max(0.0, left_mix)
        right_mix = max(0.0, right_mix)

    clipped = False
    max_mag = max(abs(left_mix), abs(right_mix), 1.0)
    if max_mag > 1.0:
        left_mix /= max_mag
        right_mix /= max_mag
        clipped = True
        limit_reason = "mix_normalized"

    left_pwm = int(round(neutral + (left_mix * span)))
    right_pwm = int(round(neutral + (right_mix * span)))
    left_pwm = int(clamp(left_pwm, int(PWM_MIN_US), int(PWM_MAX_US)))
    right_pwm = int(clamp(right_pwm, int(PWM_MIN_US), int(PWM_MAX_US)))

    pwm_dead = 8
    if abs(left_pwm - neutral) < pwm_dead:
        left_pwm = neutral
    if abs(right_pwm - neutral) < pwm_dead:
        right_pwm = neutral

    return MotorMix(
        left_pwm=left_pwm,
        right_pwm=right_pwm,
        surge_norm=float(surge_norm),
        yaw_norm=float(yaw_norm),
        clipped=bool(clipped),
        limit_reason=str(limit_reason),
    )


class MotorController:
    """Stateful trim + slew limiter around a raw twin-thruster allocation."""

    def __init__(self) -> None:
        self.prev_left_pwm = float(PWM_NEUTRAL_US)
        self.prev_right_pwm = float(PWM_NEUTRAL_US)
        self.prev_left_rate = 0.0
        self.prev_right_rate = 0.0
        self.last_ts = time.monotonic()
        self.trim_port_us = 0.0
        self.trim_stbd_us = 0.0
        self.trim_last_ts = time.monotonic()

    def snapshot(self) -> dict[str, float]:
        return {
            "prev_left_pwm": float(self.prev_left_pwm),
            "prev_right_pwm": float(self.prev_right_pwm),
            "prev_left_rate": float(self.prev_left_rate),
            "prev_right_rate": float(self.prev_right_rate),
            "last_ts": float(self.last_ts),
            "trim_port_us": float(self.trim_port_us),
            "trim_stbd_us": float(self.trim_stbd_us),
            "trim_last_ts": float(self.trim_last_ts),
        }

    def reset(self, *, keep_trim: bool = True) -> None:
        self.prev_left_pwm = float(PWM_NEUTRAL_US)
        self.prev_right_pwm = float(PWM_NEUTRAL_US)
        self.prev_left_rate = 0.0
        self.prev_right_rate = 0.0
        self.last_ts = time.monotonic()
        if not keep_trim:
            self.trim_port_us = 0.0
            self.trim_stbd_us = 0.0

    def neutral(self, *, keep_trim: bool = True) -> tuple[int, int]:
        self.reset(keep_trim=keep_trim)
        neutral = int(PWM_NEUTRAL_US)
        return neutral, neutral

    def apply_deadband_min_effective(self, pwm_us: float) -> int:
        neutral = int(PWM_NEUTRAL_US)
        pwm = int(round(float(pwm_us)))
        delta = int(pwm - neutral)
        if abs(delta) <= int(PWM_DEADBAND_US):
            return neutral
        if abs(delta) < int(PWM_MIN_EFFECTIVE_US):
            delta = int(PWM_MIN_EFFECTIVE_US if delta > 0 else -PWM_MIN_EFFECTIVE_US)
        return int(clamp(neutral + delta, int(PWM_MIN_US), int(PWM_MAX_US)))

    def update_trim(
        self,
        *,
        heading_error_deg: float,
        yaw_norm: float,
        current_speed_mps: float,
        current_yaw_rate_dps: float,
        mission_active: bool,
    ) -> None:
        now = time.monotonic()
        dt = max(0.01, min(0.30, now - float(self.trim_last_ts or now)))
        self.trim_last_ts = now
        steady_heading = abs(float(heading_error_deg)) <= float(PWM_TRIM_ERR_BAND_DEG)
        light_turn = abs(float(yaw_norm)) <= 0.15
        speed_ok = float(current_speed_mps) >= 0.2

        if mission_active and steady_heading and light_turn and speed_ok:
            step = float(current_yaw_rate_dps) * float(PWM_TRIM_LEARN_GAIN) * dt
            step = clamp(step, -1.2, 1.2)
            self.trim_port_us = float(clamp(
                self.trim_port_us + step,
                -float(PWM_TRIM_MAX_US),
                float(PWM_TRIM_MAX_US),
            ))
            self.trim_stbd_us = float(clamp(
                self.trim_stbd_us - step,
                -float(PWM_TRIM_MAX_US),
                float(PWM_TRIM_MAX_US),
            ))
            return

        decay = max(0.0, min(1.0, dt * 0.35))
        keep = 1.0 - decay
        self.trim_port_us *= keep
        self.trim_stbd_us *= keep

    def apply_slew_and_jerk(self, target_left_pwm: float, target_right_pwm: float) -> tuple[int, int]:
        now = time.monotonic()
        dt = max(0.01, min(0.30, now - float(self.last_ts or now)))
        self.last_ts = now
        max_step = float(PWM_SLEW_RATE_US_PER_S) * dt
        max_rate_delta = float(PWM_JERK_LIMIT_US_PER_S2) * dt

        left_step = clamp(float(target_left_pwm) - float(self.prev_left_pwm), -max_step, max_step)
        right_step = clamp(float(target_right_pwm) - float(self.prev_right_pwm), -max_step, max_step)

        left_rate_cmd = left_step / dt
        right_rate_cmd = right_step / dt
        left_rate = float(self.prev_left_rate) + clamp(
            left_rate_cmd - float(self.prev_left_rate),
            -max_rate_delta,
            max_rate_delta,
        )
        right_rate = float(self.prev_right_rate) + clamp(
            right_rate_cmd - float(self.prev_right_rate),
            -max_rate_delta,
            max_rate_delta,
        )

        left_pwm = float(self.prev_left_pwm) + (left_rate * dt)
        right_pwm = float(self.prev_right_pwm) + (right_rate * dt)

        if (target_left_pwm - self.prev_left_pwm) >= 0:
            left_pwm = min(left_pwm, float(target_left_pwm))
        else:
            left_pwm = max(left_pwm, float(target_left_pwm))
        if (target_right_pwm - self.prev_right_pwm) >= 0:
            right_pwm = min(right_pwm, float(target_right_pwm))
        else:
            right_pwm = max(right_pwm, float(target_right_pwm))

        left_pwm = float(clamp(left_pwm, float(PWM_MIN_US), float(PWM_MAX_US)))
        right_pwm = float(clamp(right_pwm, float(PWM_MIN_US), float(PWM_MAX_US)))

        self.prev_left_pwm = float(left_pwm)
        self.prev_right_pwm = float(right_pwm)
        self.prev_left_rate = float(left_rate)
        self.prev_right_rate = float(right_rate)
        return int(round(left_pwm)), int(round(right_pwm))

    def harden_allocation(
        self,
        *,
        allocation: object,
        heading_error_deg: float,
        speed_mps: float,
        current_speed_mps: float,
        current_yaw_rate_dps: float,
        mission_active: bool,
    ) -> tuple[int, int]:
        if abs(float(speed_mps)) <= 1e-3 and abs(float(heading_error_deg)) <= 1e-3:
            return self.neutral(keep_trim=True)

        self.update_trim(
            heading_error_deg=heading_error_deg,
            yaw_norm=float(getattr(allocation, "yaw_norm", 0.0)),
            current_speed_mps=current_speed_mps,
            current_yaw_rate_dps=current_yaw_rate_dps,
            mission_active=mission_active,
        )
        target_left = float(getattr(allocation, "left_pwm", PWM_NEUTRAL_US)) + float(self.trim_port_us)
        target_right = float(getattr(allocation, "right_pwm", PWM_NEUTRAL_US)) + float(self.trim_stbd_us)
        target_left = clamp(target_left, float(PWM_MIN_US), float(PWM_MAX_US))
        target_right = clamp(target_right, float(PWM_MIN_US), float(PWM_MAX_US))
        target_left = self.apply_deadband_min_effective(target_left)
        target_right = self.apply_deadband_min_effective(target_right)
        return self.apply_slew_and_jerk(target_left, target_right)
