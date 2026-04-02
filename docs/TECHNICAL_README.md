# BalanceBot Technical README (Code-Centric)

This document focuses on implementation-level techniques used in the BalanceBot workspace, with code snippets from firmware, simulation, control-theory, and telemetry analysis.

## 1. Technical Scope

The codebase combines four layers:

1. Embedded firmware control loops (Arduino C/C++).
2. CoppeliaSim controller scripts and scene interaction.
3. State-space and optimal-control analysis (Python/MATLAB).
4. Data parsing and visualization for tuning.

---

## 2. Core Techniques Used in Code

## 2.1 Cascaded PID for Balance + Motion

Technique:

- Outer loop regulates body position/attitude and outputs a velocity reference.
- Inner loop tracks this velocity target and drives wheel joints.

Snippet (simulation controller pattern):

```python
# Outer PID loop
err_outer = (ref + xErr) - roll
P_outer = Kp_outer * err_outer
I_outer += Ki_outer * err_outer
D_outer = Kd_outer * (err_outer - prevErr_outer)
ref_velocity = P_outer + I_outer + D_outer
prevErr_outer = err_outer

# Anti-windup
if abs(I_outer) > Imax_outer:
    I_outer = Imax_outer * (I_outer / abs(I_outer))

# Inner PID loop
err_inner = ref_velocity - sim.getObjectPosition(body, -1)[0]
P_inner = Kp_inner * err_inner
I_inner += Ki_inner * err_inner
D_inner = Kd_inner * (err_inner - prevErr_inner)
pid_inner = P_inner + I_inner + D_inner
prevErr_inner = err_inner

if abs(I_inner) > Imax_inner:
    I_inner = Imax_inner * (I_inner / abs(I_inner))

sim.setJointTargetVelocity(motor1, pid_inner)
sim.setJointTargetVelocity(motor2, pid_inner)
```

Where this appears:

- 2A/Task2a_new/Task2A/task2a_solution.py

---

## 2.2 Anti-Windup and Output Saturation

Technique:

- Integrator terms are clamped to avoid windup.
- Motor commands are saturated to actuator-safe range.

Snippet (firmware pattern):

```cpp
integral_tilt += tilt * 0.012;
integral_tilt = constrain(integral_tilt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

float control = (kp * (tilt - desired_tilt)) + (kd * tilt_dot) + (ki * integral_tilt);
control_output = constrain(control, -255, 255);

motor_control_L(control_output);
motor_control_R(control_output);
```

Where this appears:

- SBRAMAN/SBRAMAN.ino
- BB_1545_Task5/BB_1545_Task5.ino

---

## 2.3 Interrupt-Driven Real-Time Updates

Technique:

- Timer overflow ISR runs control-cycle logic at fixed cadence.
- IMU is updated in interrupt context, then feedback and control equation are executed.

Snippet (firmware ISR pattern):

```cpp
ISR(TIMER1_OVF_vect) {
    sei();
    TCNT1H = 0xA2;
    TCNT1L = 0x3F;
    mpu.update();
    cli();

    feedback();
    control_eqn();
}
```

Why used:

- Improves timing consistency of control loop.
- Reduces jitter versus variable loop() timing.

---

## 2.4 Differential Motor Direction Control

Technique:

- Sign of control value determines motor direction.
- Magnitude sets PWM duty.

Snippet:

```cpp
void motor_control_L(int pwm) {
    if (pwm < 0) {
        digitalWrite(InL1, HIGH);
        digitalWrite(InL2, LOW);
        pwm = -pwm;
    } else {
        digitalWrite(InL1, LOW);
        digitalWrite(InL2, HIGH);
    }
    analogWrite(PWML, pwm);
}
```

Applied similarly for right motor channel.

---

## 2.5 Coordinate Compensation in Simulation

Technique:

- Roll/pitch are corrected using yaw-coupling terms before PID.
- Helps decouple body tilt from heading rotation in simulation frame.

Snippet:

```python
yaw = dummy_yaw
pitch = BodyPos[0]
roll = BodyPos[1]

roll -= pitch * math.sin(yaw)
pitch += roll * math.sin(yaw)
```

Where this appears:

- 2A/Task2a_new/Task2A/task2a_solution.py

---

## 2.6 Keyboard-Driven Manipulation + Mobility Commands

Technique:

- Controller reads simulator key events and maps them to:
  - arm/prismatic joint speeds,
  - forward/backward reference updates,
  - turning commands.

Snippet:

```python
if data[0] == 2008:   # up arrow
    ref += 0.002
elif data[0] == 2007: # down arrow
    ref -= 0.002
elif data[0] == 2010: # left arrow
    Ldrive()
elif data[0] == 2009: # right arrow
    Rdrive()
```

This creates a teleop-style test harness for balancing and task execution.

---

## 2.7 Symbolic Linearization and Stability Analysis

Technique:

- Build nonlinear equations symbolically.
- Compute Jacobians A, B at equilibrium points.
- Evaluate eigenvalues for local stability.

Snippet:

```python
A_matrix = sp.Matrix([
    [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
    [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
])

B_matrix = sp.Matrix([
    [sp.diff(x1_dot, u)],
    [sp.diff(x2_dot, u)]
])
```

Where this appears:

- Task1A/Task1A_windows/Task1A/Task_1A.py

---

## 2.8 LQR Gain Synthesis (Python)

Technique:

- Use continuous-time algebraic Riccati equation (CARE).
- Compute feedback gain \(K = R^{-1}B^T P\).

Snippet:

```python
P = linalg.solve_continuous_are(A, B, Q, R)
K = np.dot(np.linalg.inv(R), np.dot(B.T, P))
```

Where this appears:

- Task1A/Task1A_windows/Task1A/Task_1A.py

---

## 2.9 LQR Design and Closed-Loop ODE Simulation (MATLAB)

Technique:

- Define pendulum-cart nonlinear dynamics.
- Design LQR with weighted Q/R.
- Simulate with ode45 and animate cart + pendulum.

Snippet:

```matlab
K = lqr(A, B, Q, R);
u = @(x) -K*(x - wr);
pendcart_with_control = @(t,x) pendcart(x, m, M, L, g, d, u(x));
[t, x] = ode45(pendcart_with_control, tspan, x0);
```

Where this appears:

- MAtlab/cartpend.m

---

## 2.10 Discrete Time Integration for Fast Prototyping (Python)

Technique:

- Manually integrate nonlinear pendulum equations with fixed step \(dt\).
- Shift reference during runtime to test disturbance/re-targeting behavior.

Snippet:

```python
for i in range(0, N):
    X = np.matrix([[th[i], dth[i], x[i] - x0, v[i]]]).T
    f.append((-K * X)[0, 0])

    d2th = (g * sin(th[i]) - f[i] * cos(th[i])) / L
    dth1 = dth[i] + d2th * dt

    v.append(v[i] + f[i] * dt)
    x.append(x[i] + v[i] * dt)
    th.append(th[i] + dth1 * dt)
    dth.append(dth1)

    if i == 5000:
        x0 = 0.1
```

Where this appears:

- MAtlab/sim.py

---

## 2.11 Log Parsing and Multi-Signal Visualization

Technique:

- Parse custom text logs with regex.
- Build DataFrame and overlay multiple traces.

Snippet:

```python
match = re.search(
    r'tur_yaw = ([^,]+), bal_yaw = ([^,]+), pitch = ([^,]+), roll = ([^,]+), YxRef = ([^,]+), TxRef = ([^,]+)',
    line
)

if match:
    tur_yaw_values.append(float(match.group(1)))
    bal_yaw_values.append(float(match.group(2)))
    pitch_values.append(float(match.group(3)))
    roll_values.append(float(match.group(4)))
```

Where this appears:

- 2A/data.py

---

## 3. Techniques Matrix (Quick Reference)

| Area | Technique | Purpose |
|---|---|---|
| Embedded control | Cascaded PID | Stabilize and command motion simultaneously |
| Embedded reliability | Anti-windup + saturation | Prevent integrator runaway and protect motors |
| Embedded timing | Timer ISR loop | Deterministic update interval |
| Simulation control | CoppeliaSim API loops | Rapid testing and safe tuning |
| Simulation interaction | Keyboard event mapping | Human-in-the-loop testing |
| Control theory | Jacobian linearization | Local model around equilibria |
| Optimal control | LQR via CARE | Balanced state regulation/control effort |
| Analysis | Regex + plotting | Tuning based on measured behavior |

---

## 4. Practical Method Recipes

## Recipe A: Tune cascaded PID safely

1. Tune inner loop first with small reference changes.
2. Add outer loop with conservative gains.
3. Clamp integrators and output from day one.
4. Validate in simulation, then hardware.

## Recipe B: Convert nonlinear model to feedback law

1. Derive nonlinear dynamics.
2. Find equilibrium and Jacobians.
3. Verify eigenvalues.
4. Synthesize LQR and simulate disturbances.

## Recipe C: Build a repeatable tuning run

1. Run fixed scenario for a fixed duration.
2. Log tilt, roll, yaw, reference, command.
3. Parse and plot all signals in one chart.
4. Compare runs by gain-set ID.

---

## 5. Suggested Next Code Improvements

1. Extract PID calculations into reusable functions/classes.
2. Add a single configuration file for all gains.
3. Normalize log output format across firmware/simulation.
4. Add automated metric calculation: overshoot, settling time, RMS error.
5. Keep one canonical controller per layer and archive old variants.

---

## 6. File References Used For This README

- BB_1545_Task5/BB_1545_Task5.ino
- SBRAMAN/SBRAMAN.ino
- 2A/Task2a_new/Task2A/task2a_solution.py
- Task1A/Task1A_windows/Task1A/Task_1A.py
- MAtlab/cartpend.m
- MAtlab/sim.py
- 2A/data.py
