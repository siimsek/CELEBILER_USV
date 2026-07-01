# Control Systems - Sharp Edges

## Wrong PID Tuning Order Causes Oscillation

### **Severity**: critical
### **Summary**
Tuning I before P, or D before I, leads to instability
### **Symptoms**
- System oscillates at any gain setting
- Increasing gain makes things worse
- Controller never stabilizes
### **Why**
PID terms interact. Tuning in wrong order creates instability:

- P alone: Proportional response, steady-state error
- I without P: Phase lag, guaranteed oscillation
- D without P: Amplifies noise, no steady tracking

Correct order: P first (for response), then I (eliminate error),
finally D (reduce overshoot).

Starting with I or having too much I relative to P
causes phase lag that leads to oscillation.

### **Solution**
```python
# Systematic tuning procedure
# 1. Set I = 0, D = 0, increase P until oscillation
pid = PIDController(kp=0, ki=0, kd=0)

# Find critical gain (Ku) where oscillation starts
for kp in np.linspace(0, 100, 100):
    pid.gains.kp = kp
    # Test and check for sustained oscillation

# 2. Use Ziegler-Nichols or similar method
ku = 50  # Critical gain
tu = 0.5  # Oscillation period

# PID: kp = 0.6*ku, ki = 1.2*ku/tu, kd = 0.075*ku*tu
gains = ziegler_nichols_tuning(ku, tu, 'PID')

# 3. Fine-tune from there
# Reduce I if overshoot, increase D if oscillating
```

## Continuous PID Formulas Don't Work at Low Sample Rates

### **Severity**: high
### **Summary**
Textbook PID formulas assume continuous time, fail when sampled slowly
### **Why**
Continuous PID: u = Kp*e + Ki*integral(e) + Kd*de/dt

Discrete implementation matters:
- Integral: Euler vs trapezoidal vs exact
- Derivative: Forward vs backward vs filtered

At low sample rates, these differences are significant.
Bilinear (Tustin) transform preserves stability better
than simple Euler integration.

### **Solution**
```python
# Use proper discrete-time formulation

# 1. Trapezoidal integration (more accurate)
self.integral += 0.5 * (error + self.prev_error) * dt

# 2. Filtered derivative (reduces noise)
# First-order filter: d_filt = alpha * d_raw + (1-alpha) * d_prev
tau_d = 0.1  # Filter time constant
alpha = dt / (tau_d + dt)
d_raw = (error - self.prev_error) / dt
d_filtered = alpha * d_raw + (1 - alpha) * self.prev_derivative

# 3. Or use bilinear transform for entire controller
# s -> 2/T * (z-1)/(z+1)

# 4. Derivative on measurement, not error
d_raw = -(measurement - self.prev_measurement) / dt
```

## MPC Model Mismatch Causes Poor Performance

### **Severity**: critical
### **Summary**
MPC relies on accurate model; errors cause suboptimal or unstable control
### **Why**
MPC optimizes based on predicted future states.
If the model is wrong, predictions are wrong,
and the "optimal" control is actually suboptimal.

Common model errors:
- Wrong time constants
- Unmodeled delays
- Nonlinearities treated as linear
- Parameter drift over time

### **Solution**
```python
# 1. System identification first
# Collect input/output data, fit model
from scipy.optimize import curve_fit

def system_model(t, tau, K, delay):
    # First-order + delay
    return K * (1 - np.exp(-(t - delay) / tau))

params, _ = curve_fit(system_model, t_data, y_data)

# 2. Validate model on test data
# Check prediction error over horizon

# 3. Add robustness margins
# Increase R (control penalty) if model uncertain
Q = np.diag([10, 1, 1])  # State costs
R = np.diag([1.0])        # Input cost (increase if uncertain)

# 4. Use adaptive MPC for changing dynamics
# Re-identify model periodically
```

## Control Loop Rate Too Slow

### **Severity**: critical
### **Summary**
Control loop slower than system dynamics causes instability
### **Why**
Nyquist criterion: sample at least 2x fastest dynamic.
In practice: 10-20x for good performance.

If system bandwidth is 10Hz, control loop should be 100-200Hz.

### **Solution**
```python
# 1. Use hardware timer for precise control loop
def timer_isr():
    """1kHz control interrupt."""
    global position, setpoint
    u = pid.update(setpoint, position)
    motor.set_pwm(u)

setup_timer_interrupt(frequency=1000, callback=timer_isr)

# 2. Separate fast and slow loops
# Fast: current/velocity (hardware timer, 1-10kHz)
# Slow: position/trajectory (software, 100-500Hz)

# 3. For ROS2: Use realtime-safe callback groups
from rclpy.callback_groups import RealtimeCallbackGroup

self.control_timer = self.create_timer(
    0.001,  # 1ms = 1kHz
    self.control_callback,
    callback_group=RealtimeCallbackGroup()
)
```

## Ignoring Actuator Saturation

### **Severity**: high
### **Summary**
Controller commands exceed physical limits, causes windup and instability
### **Why**
Every actuator has limits:
- Motors: max current, max voltage
- Servos: max position, max velocity
- Pumps: max flow rate

If controller outputs exceed these, the actuator saturates.
The controller keeps integrating error, causing windup.
When error reduces, the accumulated integral causes overshoot.

### **Solution**
```python
# 1. Clamp output and implement anti-windup
pid = PIDController(
    gains, dt,
    output_limits=(-24.0, 24.0)  # Voltage limits
)

# 2. Back-calculation anti-windup
output_unsat = p + i + d
output = np.clip(output_unsat, -24, 24)
if ki != 0:
    anti_windup = (output - output_unsat) / ki
    integral += anti_windup

# 3. Conditional integration
if not saturated:
    integral += error * dt
# Don't integrate while saturated

# 4. Use MPC with explicit constraints
mpc = LinearMPC(
    A, B, params,
    u_min=np.array([-24.0]),
    u_max=np.array([24.0])
)
```

## Setpoint Jump Causes Actuator Stress

### **Severity**: medium
### **Summary**
Step changes in setpoint cause aggressive control action
### **Why**
Derivative term amplifies sudden changes.
Step change in setpoint = infinite derivative = kick.

### **Solution**
```python
# 1. Derivative on measurement (not error)
derivative = -(measurement - prev_measurement) / dt
# Not: derivative = (error - prev_error) / dt

# 2. Setpoint ramping/filtering
class SetpointFilter:
    def __init__(self, rate_limit, dt):
        self.rate = rate_limit
        self.dt = dt
        self.filtered = 0

    def update(self, setpoint):
        delta = setpoint - self.filtered
        max_delta = self.rate * self.dt
        delta = np.clip(delta, -max_delta, max_delta)
        self.filtered += delta
        return self.filtered

# 3. Use trajectory generator
# Instead of step: use minimum-jerk or trapezoidal profile

# 4. Setpoint weighting (P acts on weighted setpoint)
# u = Kp * (b * setpoint - measurement) + ...
# b < 1 reduces kick
```
