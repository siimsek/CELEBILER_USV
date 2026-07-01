"""Small dependency-free EKF-style state estimator helpers.

State vector:
    [x_m, y_m, vx_mps, vy_mps, heading_deg, heading_rate_dps]

The model is intentionally conservative: it smooths GPS/heading measurements
and rejects large innovations, but it does not own control or safety decisions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math


def _wrap_180(deg: float) -> float:
    value = (float(deg) + 180.0) % 360.0 - 180.0
    if value == -180.0:
        return 180.0
    return value


def _identity(n: int) -> list[list[float]]:
    return [[1.0 if r == c else 0.0 for c in range(n)] for r in range(n)]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    rows = len(a)
    cols = len(b[0]) if b else 0
    inner = len(b)
    return [[sum(float(a[r][k]) * float(b[k][c]) for k in range(inner)) for c in range(cols)] for r in range(rows)]


def _transpose(a: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*a)]


@dataclass
class KalmanUpdate:
    accepted: bool
    chi2: float
    threshold: float
    innovation: list[float] = field(default_factory=list)


class ExtendedKalmanFilter6D:
    """Linearized constant-velocity estimator with heading wrap handling."""

    def __init__(self) -> None:
        self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.p = _identity(6)
        self.initialized = False
        self.last_update = KalmanUpdate(True, 0.0, 0.0, [])

    def reset(self, *, x_m: float = 0.0, y_m: float = 0.0, heading_deg: float = 0.0) -> None:
        self.x = [float(x_m), float(y_m), 0.0, 0.0, float(heading_deg) % 360.0, 0.0]
        self.p = _identity(6)
        self.initialized = True
        self.last_update = KalmanUpdate(True, 0.0, 0.0, [])

    def predict(self, dt_s: float, *, process_var: float = 0.08) -> None:
        dt = max(0.001, min(2.0, float(dt_s)))
        if not self.initialized:
            self.reset()
        f = _identity(6)
        f[0][2] = dt
        f[1][3] = dt
        f[4][5] = dt
        state_col = [[v] for v in self.x]
        self.x = [row[0] for row in _matmul(f, state_col)]
        self.x[4] = self.x[4] % 360.0
        ft = _transpose(f)
        q = _identity(6)
        for i in range(6):
            q[i][i] = float(process_var)
        self.p = _matmul(_matmul(f, self.p), ft)
        for r in range(6):
            self.p[r][r] += q[r][r]

    def update_position(
        self,
        x_m: float,
        y_m: float,
        *,
        measurement_var: float = 1.0,
        chi2_threshold: float = 9.21,
    ) -> KalmanUpdate:
        if not self.initialized:
            self.reset(x_m=x_m, y_m=y_m)
            self.last_update = KalmanUpdate(True, 0.0, chi2_threshold, [0.0, 0.0])
            return self.last_update

        innovation = [float(x_m) - self.x[0], float(y_m) - self.x[1]]
        s00 = self.p[0][0] + float(measurement_var)
        s01 = self.p[0][1]
        s10 = self.p[1][0]
        s11 = self.p[1][1] + float(measurement_var)
        det = (s00 * s11) - (s01 * s10)
        if abs(det) < 1e-9:
            self.last_update = KalmanUpdate(False, float("inf"), chi2_threshold, innovation)
            return self.last_update
        inv = [[s11 / det, -s01 / det], [-s10 / det, s00 / det]]
        chi2 = (
            innovation[0] * (inv[0][0] * innovation[0] + inv[0][1] * innovation[1])
            + innovation[1] * (inv[1][0] * innovation[0] + inv[1][1] * innovation[1])
        )
        if chi2 > float(chi2_threshold):
            self.last_update = KalmanUpdate(False, float(chi2), float(chi2_threshold), innovation)
            return self.last_update

        k = [[0.0, 0.0] for _ in range(6)]
        for r in range(6):
            k[r][0] = (self.p[r][0] * inv[0][0]) + (self.p[r][1] * inv[1][0])
            k[r][1] = (self.p[r][0] * inv[0][1]) + (self.p[r][1] * inv[1][1])
        for r in range(6):
            self.x[r] += (k[r][0] * innovation[0]) + (k[r][1] * innovation[1])
        kh = [[0.0] * 6 for _ in range(6)]
        for r in range(6):
            kh[r][0] = k[r][0]
            kh[r][1] = k[r][1]
        i_minus_kh = _identity(6)
        for r in range(6):
            for c in range(6):
                i_minus_kh[r][c] -= kh[r][c]
        self.p = _matmul(i_minus_kh, self.p)
        self.last_update = KalmanUpdate(True, float(chi2), float(chi2_threshold), innovation)
        return self.last_update

    def update_heading(
        self,
        heading_deg: float,
        *,
        measurement_var: float = 16.0,
        chi2_threshold: float = 6.63,
    ) -> KalmanUpdate:
        if not self.initialized:
            self.reset(heading_deg=heading_deg)
            self.last_update = KalmanUpdate(True, 0.0, chi2_threshold, [0.0])
            return self.last_update
        innovation = [_wrap_180(float(heading_deg) - self.x[4])]
        s = self.p[4][4] + float(measurement_var)
        if s <= 1e-9:
            self.last_update = KalmanUpdate(False, float("inf"), chi2_threshold, innovation)
            return self.last_update
        chi2 = (innovation[0] * innovation[0]) / s
        if chi2 > float(chi2_threshold):
            self.last_update = KalmanUpdate(False, float(chi2), float(chi2_threshold), innovation)
            return self.last_update
        k = [self.p[r][4] / s for r in range(6)]
        for r in range(6):
            self.x[r] += k[r] * innovation[0]
        self.x[4] = self.x[4] % 360.0
        for r in range(6):
            for c in range(6):
                self.p[r][c] -= k[r] * self.p[4][c]
        self.last_update = KalmanUpdate(True, float(chi2), float(chi2_threshold), innovation)
        return self.last_update

    def to_dict(self) -> dict[str, float | bool]:
        return {
            "initialized": bool(self.initialized),
            "x_m": round(float(self.x[0]), 3),
            "y_m": round(float(self.x[1]), 3),
            "vx_mps": round(float(self.x[2]), 3),
            "vy_mps": round(float(self.x[3]), 3),
            "heading_deg": round(float(self.x[4] % 360.0), 3),
            "heading_rate_dps": round(float(self.x[5]), 3),
        }
