# Control Systems - Validations

## Derivative Computed on Error

### **Severity**: warning
### **Type**: regex
### **Pattern**
- error\s*-\s*prev_error|prev_error\s*-\s*error
- self\.error\s*-\s*self\.prev_error
- d_term.*error.*prev
### **Message**
Computing derivative on error causes derivative kick on setpoint change.
### **Fix Action**
Use derivative on measurement: -(measurement - prev_measurement) / dt

## PID Without Anti-Windup

### **Severity**: warning
### **Type**: regex
### **Pattern**
- integral\s*\+=.*error.*dt(?![\s\S]{0,200}(clip|clamp|windup|limit|saturate))
### **Message**
Integral accumulation without anti-windup causes overshoot when saturated.
### **Fix Action**
Add clamping or back-calculation anti-windup

## PID Without Output Limits

### **Severity**: warning
### **Type**: regex
### **Pattern**
- class.*PID.*:(?![\s\S]{0,500}(output_max|u_max|limit|clip|clamp))
### **Message**
PID controller should have output limits matching actuator constraints.
### **Fix Action**
Add output_limits parameter and clamp control output

## Unfiltered Derivative Term

### **Severity**: info
### **Type**: regex
### **Pattern**
- /\s*dt(?![\s\S]{0,50}(filter|alpha|tau|lpf))
- d_term\s*=.*-.*prev.*(?!filter)
### **Message**
Unfiltered derivative amplifies high-frequency noise.
### **Fix Action**
Add low-pass filter: d_filt = alpha * d_raw + (1-alpha) * d_prev

## Simple Euler Integration for Integral Term

### **Severity**: info
### **Type**: regex
### **Pattern**
- integral\s*\+=\s*error\s*\*\s*dt
### **Message**
Simple Euler integration can accumulate error. Consider trapezoidal integration.
### **Fix Action**
Use: integral += 0.5 * (error + prev_error) * dt for better accuracy

## Control Loop with sleep() Call

### **Severity**: warning
### **Type**: regex
### **Pattern**
- time\.sleep\(0\.[1-9]|time\.sleep\([1-9]
- rospy\.sleep\(0\.[1-9]
### **Message**
Control loop may be too slow. Use hardware timer for consistent timing.
### **Fix Action**
Use hardware timer interrupt or ROS2 timer for precise control loop

## MPC Without Input Constraints

### **Severity**: warning
### **Type**: regex
### **Pattern**
- class.*MPC.*:(?![\s\S]{0,800}(u_min|u_max|bounds|constraint))
### **Message**
MPC without constraints loses main advantage. Add actuator limits.
### **Fix Action**
Add u_min/u_max constraints matching physical actuator limits

## Hardcoded PID Gains Without Comments

### **Severity**: info
### **Type**: regex
### **Pattern**
- kp\s*=\s*\d+\.?\d*\s*(?!#)
- ki\s*=\s*\d+\.?\d*\s*(?!#)
- PIDGains\(kp=\d+.*\)\s*$
### **Message**
Document tuning rationale for PID gains, or load from config.
### **Fix Action**
Add comment explaining tuning method, or use parameter server

## Step Setpoint Without Trajectory Generation

### **Severity**: info
### **Type**: regex
### **Pattern**
- setpoint\s*=\s*target(?![\s\S]{0,100}(filter|ramp|trajectory|profile))
### **Message**
Sudden setpoint changes stress mechanical systems.
### **Fix Action**
Use trajectory generator (minimum-jerk, trapezoidal) for smooth motion

## Float Equality in Control Logic

### **Severity**: warning
### **Type**: regex
### **Pattern**
- if.*error\s*==\s*0
- if.*position\s*==\s*setpoint
### **Message**
Floating-point equality rarely holds. Use tolerance-based comparison.
### **Fix Action**
Use: if abs(error) < tolerance or np.isclose()

## LQR/MPC Without Explicit Jacobian

### **Severity**: info
### **Type**: regex
### **Pattern**
- lqr|LQR|design_lqr(?![\s\S]{0,300}jacobian)
### **Message**
Ensure linearization Jacobian is computed correctly for nonlinear systems.
### **Fix Action**
Verify Jacobian analytically or use automatic differentiation
