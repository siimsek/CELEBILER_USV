# Control Systems

## Patterns

### **PID Controller**
#### **Name**
PID Controller Implementation
#### **Description**
Classic proportional-integral-derivative control with anti-windup
#### **Pattern**
```python
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class PIDGains:
    """PID gains with optional derivative filter."""
    kp: float = 1.0      # Proportional gain
    ki: float = 0.0      # Integral gain
    kd: float = 0.0      # Derivative gain
    tau_d: float = 0.1   # Derivative filter time constant

class PIDController:
    """PID controller with anti-windup and derivative filtering.

    Features:
    - Integral anti-windup (clamping and back-calculation)
    - Derivative on measurement (avoids derivative kick)
    - Low-pass filter on derivative term
    - Bumpless transfer for gain changes
    """

    def __init__(self, gains: PIDGains, dt: float,
                 output_limits: tuple = (-np.inf, np.inf)):
        self.gains = gains
        self.dt = dt
        self.output_min, self.output_max = output_limits

        # State
        self.integral = 0.0
        self.prev_measurement = None
        self.prev_derivative = 0.0
        self.prev_output = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """Compute control output.

        Args:
            setpoint: Desired value
            measurement: Current measured value

        Returns:
            Control output (clamped to limits)
        """
        error = setpoint - measurement

        # Proportional term
        p_term = self.gains.kp * error

        # Integral term with clamping anti-windup
        self.integral += error * self.dt
        i_term = self.gains.ki * self.integral

        # Derivative term on measurement (not error)
        # Avoids derivative kick on setpoint changes
        if self.prev_measurement is None:
            d_term = 0.0
        else:
            # Raw derivative
            d_raw = -(measurement - self.prev_measurement) / self.dt

            # Low-pass filter on derivative
            alpha = self.dt / (self.gains.tau_d + self.dt)
            d_filtered = alpha * d_raw + (1 - alpha) * self.prev_derivative
            self.prev_derivative = d_filtered

            d_term = self.gains.kd * d_filtered

        self.prev_measurement = measurement

        # Compute output
        output_unsat = p_term + i_term + d_term

        # Clamp output
        output = np.clip(output_unsat, self.output_min, self.output_max)

        # Back-calculation anti-windup
        if self.gains.ki != 0:
            saturation_error = output - output_unsat
            self.integral += saturation_error / self.gains.ki

        self.prev_output = output
        return output

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_measurement = None
        self.prev_derivative = 0.0

    def set_gains(self, gains: PIDGains):
        """Update gains with bumpless transfer."""
        # Adjust integral to maintain output continuity
        if self.gains.ki != 0 and gains.ki != 0:
            self.integral *= self.gains.ki / gains.ki
        self.gains = gains

# Ziegler-Nichols tuning helper
def ziegler_nichols_tuning(ku: float, tu: float, controller_type: str = 'PID') -> PIDGains:
    """Compute PID gains using Ziegler-Nichols method.

    Args:
        ku: Ultimate gain (gain at sustained oscillation)
        tu: Oscillation period at ku
        controller_type: 'P', 'PI', or 'PID'
    """
    if controller_type == 'P':
        return PIDGains(kp=0.5 * ku)
    elif controller_type == 'PI':
        return PIDGains(
            kp=0.45 * ku,
            ki=0.45 * ku / (0.83 * tu)
        )
    else:  # PID
        return PIDGains(
            kp=0.6 * ku,
            ki=0.6 * ku / (0.5 * tu),
            kd=0.6 * ku * 0.125 * tu
        )
```

### **State-Space Control**
#### **Name**
LQR Controller Design
#### **Description**
Linear Quadratic Regulator for optimal state feedback
#### **Pattern**
```python
import numpy as np
from scipy import linalg
from dataclasses import dataclass
from typing import Tuple

@dataclass
class LinearSystem:
    """Continuous-time linear system: dx/dt = Ax + Bu."""
    A: np.ndarray  # State matrix
    B: np.ndarray  # Input matrix
    C: np.ndarray = None  # Output matrix
    D: np.ndarray = None  # Feedthrough matrix

    def discretize(self, dt: float) -> 'DiscreteLinearSystem':
        """Convert to discrete-time using zero-order hold."""
        n = self.A.shape[0]
        m = self.B.shape[1]

        # Build augmented matrix
        M = np.zeros((n + m, n + m))
        M[:n, :n] = self.A
        M[:n, n:] = self.B

        # Matrix exponential
        expM = linalg.expm(M * dt)

        Ad = expM[:n, :n]
        Bd = expM[:n, n:]

        return DiscreteLinearSystem(Ad, Bd, self.C, self.D)

class LQRController:
    """Linear Quadratic Regulator.

    Minimizes: J = sum(x'Qx + u'Ru)
    """

    def __init__(self, system: LinearSystem, Q: np.ndarray, R: np.ndarray):
        self.system = system
        self.Q = Q  # State cost
        self.R = R  # Input cost

        # Solve Riccati equation
        self.P = linalg.solve_continuous_are(system.A, system.B, Q, R)

        # Compute optimal gain
        self.K = linalg.inv(R) @ system.B.T @ self.P

    def compute_control(self, state: np.ndarray, reference: np.ndarray = None) -> np.ndarray:
        """Compute optimal control input."""
        if reference is None:
            reference = np.zeros_like(state)

        error = reference - state
        return self.K @ error

    def get_closed_loop_system(self) -> LinearSystem:
        """Return closed-loop system matrix."""
        A_cl = self.system.A - self.system.B @ self.K
        return LinearSystem(A_cl, self.system.B, self.system.C, self.system.D)

# Discrete LQR
class DiscreteLQRController:
    """Discrete-time LQR controller."""

    def __init__(self, system: 'DiscreteLinearSystem', Q: np.ndarray, R: np.ndarray):
        self.system = system

        # Solve discrete Riccati equation
        self.P = linalg.solve_discrete_are(system.A, system.B, Q, R)

        # Compute gain
        self.K = linalg.inv(R + system.B.T @ self.P @ system.B) @ system.B.T @ self.P @ system.A

    def compute_control(self, state: np.ndarray, reference: np.ndarray = None) -> np.ndarray:
        if reference is None:
            reference = np.zeros_like(state)
        error = reference - state
        return self.K @ error
```

### **Trajectory Tracking**
#### **Name**
Smooth Trajectory Generation
#### **Description**
Generate smooth trajectories for motion control
#### **Pattern**
```python
import numpy as np
from typing import Tuple

def minimum_jerk_trajectory(start: float, end: float,
                             duration: float, t: float) -> Tuple[float, float, float]:
    """Minimum jerk trajectory for smooth motion.

    Returns position, velocity, acceleration at time t.
    """
    if t < 0:
        return start, 0.0, 0.0
    if t > duration:
        return end, 0.0, 0.0

    tau = t / duration
    tau3 = tau ** 3
    tau4 = tau ** 4
    tau5 = tau ** 5

    # Position
    s = 10 * tau3 - 15 * tau4 + 6 * tau5
    pos = start + (end - start) * s

    # Velocity
    ds = (30 * tau**2 - 60 * tau3 + 30 * tau4) / duration
    vel = (end - start) * ds

    # Acceleration
    dds = (60 * tau - 180 * tau**2 + 120 * tau3) / duration**2
    acc = (end - start) * dds

    return pos, vel, acc

def trapezoidal_velocity_profile(start: float, end: float,
                                  v_max: float, a_max: float,
                                  t: float) -> Tuple[float, float, float]:
    """Trapezoidal velocity profile (bang-bang with cruise)."""
    distance = end - start
    sign = np.sign(distance)
    distance = abs(distance)

    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc**2

    if 2 * d_acc >= distance:
        # Triangle profile
        t_acc = np.sqrt(distance / a_max)
        t_total = 2 * t_acc
        t_cruise = 0
    else:
        # Trapezoidal profile
        d_cruise = distance - 2 * d_acc
        t_cruise = d_cruise / v_max
        t_total = 2 * t_acc + t_cruise

    if t < 0:
        return start, 0.0, 0.0
    elif t < t_acc:
        pos = start + sign * 0.5 * a_max * t**2
        vel = sign * a_max * t
        acc = sign * a_max
    elif t < t_acc + t_cruise:
        pos = start + sign * (d_acc + v_max * (t - t_acc))
        vel = sign * v_max
        acc = 0.0
    elif t < t_total:
        t_dec = t - t_acc - t_cruise
        pos = start + sign * (d_acc + v_max * t_cruise + v_max * t_dec - 0.5 * a_max * t_dec**2)
        vel = sign * (v_max - a_max * t_dec)
        acc = -sign * a_max
    else:
        return end, 0.0, 0.0

    return pos, vel, acc
```

## Anti-Patterns

### **Derivative Kick**
#### **Problem**
Derivative of error causes spikes on setpoint change
#### **Solution**
Use derivative on measurement: d/dt(measurement), not d/dt(error)

### **Integral Windup**
#### **Problem**
Integral accumulates during saturation, causes overshoot
#### **Solution**
Implement anti-windup: clamping, back-calculation, or conditional integration

### **Tuning At One Point**
#### **Problem**
Controller works at one speed/load, fails at others
#### **Solution**
Use gain scheduling or adaptive control for varying conditions

### **Ignoring Actuator Limits**
#### **Problem**
Controller commands exceed physical limits
#### **Solution**
Include constraints in control design (MPC) or saturate output
