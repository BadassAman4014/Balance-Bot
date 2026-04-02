# BalanceBot Detailed Technical Report

## A. Executive Summary

BalanceBot is a robotics workspace focused on two-wheeled inverted pendulum control across simulation and embedded deployment. The repository includes:

- Practical balancing firmware implementations.
- CoppeliaSim task automation and controller experiments.
- Control theory workflows (symbolic dynamics, linearization, LQR).
- Extensive exploratory/archival datasets and external references.

Engineering quality is high in control depth, but maintainability is reduced by duplication and mixed archive/active content.

## B. Scope of This Report

This report documents:

1. Domain architecture.
2. Methodologies and control strategies.
3. Full technology stack.
4. Folder-level technical intent.
5. Development and validation workflow.
6. Risks and remediation roadmap.

## C. Domain Architecture

### C.1 Physical Plant

- Two-wheeled inverted pendulum robot.
- Coupled translational and rotational dynamics.
- Optional manipulator elements in some task firmware (arm/gripper).

### C.2 Sensing

- IMU tilt/attitude sensing.
- Wheel encoder feedback for velocity/displacement.
- Derived rates from sampled state differences.

### C.3 Actuation

- Differential drive motor PWM control.
- Optional servo channels for manipulation tasks.

### C.4 Compute Layers

- Embedded real-time control (Arduino-class firmware).
- Simulation control (Python + CoppeliaSim API).
- Theory and design layer (MATLAB/Python scientific stack).

## D. Methodologies

### D.1 Cascaded PID Control

Observed pattern:

1. Outer loop computes desired internal target from posture/position error.
2. Inner loop tracks this target via wheel velocity/actuation response.
3. Combined output applied with saturation and anti-windup.

Advantages:

- Handles multi-time-scale dynamics effectively.
- Tuning can be staged (outer then inner).
- Robust against moderate modeling mismatch.

Limitations:

- Manual tuning burden.
- Sensitive to sensor noise and derivative estimation quality.
- Gain portability across simulation and hardware is limited.

### D.2 Symbolic and State-Space Analysis

Methods present in the workspace:

- Nonlinear equation formulation.
- Equilibrium point solving.
- Jacobian linearization around operating points.
- Eigenvalue-based local stability evaluation.

Purpose:

- Provides formal understanding of stability margins.
- Supplies linear model for optimal control synthesis.

### D.3 LQR-Based Control Design

Typical workflow represented:

1. Build linearized system matrices A, B.
2. Select Q, R cost matrices.
3. Solve Riccati equation for optimal gain K.
4. Simulate closed-loop dynamics and tune weighting.

Strength:

- Principled trade-off between state regulation and control effort.

Gap:

- Hardware transfer requires robust state estimation and actuator constraints handling.

### D.4 Simulation-First Verification

CoppeliaSim is used for:

- Fast controller prototyping.
- Safety validation before hardware trials.
- Behavioral regression checks across tuning variants.

### D.5 Data-Centric Tuning

- Logs are parsed and visualized for yaw/pitch/roll and references.
- Multiple output snapshots indicate iterative gain sweeps.
- Enables evidence-based tuning instead of blind manual adjustment.

## E. Technology Inventory

### E.1 Languages

- Arduino C/C++
- Python
- MATLAB
- Lua (scene-side simulation scripts)

### E.2 Core Libraries and Platforms

- CoppeliaSim/V-REP simulation assets and APIs.
- zmqRemoteApi and related wrappers.
- numpy/scipy/sympy for modeling and control math.
- pandas/plotly for telemetry analysis.

### E.3 Extended Ecosystem in Imported References

- ROS2 nodes and packages.
- Advanced controllers (MPC, iLQR, adaptive LQR, sliding mode).
- RL policy experiments (external references).

## F. Folder-Level Technical Intent

### F.1 Active Control/Firmware Zones

- `SBRAMAN/`: iterative firmware variants (tuning, logging, merged control logic).
- `SimplePID/`: compact PID/cascade testbed.
- `BB_1545_Task5/`: integrated balancing plus task behavior.

### F.2 Simulation and Assignment Zones

- `Task0A/` to `Task1C/`: staged simulation/control tasks.
- `2A/`, `2B/`, `Task2A_windows/`, `Task2B_windows/`: extended task implementations and outputs.
- `Task 3/`: advanced task segments.

### F.3 Theory and Model Zones

- `MAtlab/`: analytical and simulation scripts.
- `Model/`: model files and simulation assets.

### F.4 References and Archive Zones

- `GIT/`: external repository imports for comparison/reference.
- `TESTTTSSSSSSS/`: sandbox and extracted artifacts, mostly non-canonical.
- `EyrcNotes/`: historical notes and records.

## G. Development Workflow (Observed)

1. Conceptual design (state-space / PID strategy).
2. Simulation implementation and keyboard/scripted tests.
3. Iterative gain tuning via logs and plots.
4. Hardware deployment to microcontroller firmware.
5. Field tuning and merged variant creation.

## H. Build and Runtime Ecosystem

### H.1 Embedded

- Arduino-compatible toolchain for firmware compile/upload.
- Serial/Bluetooth interfaces for command and diagnostics.

### H.2 Simulation

- CoppeliaSim scene loading.
- Python API client execution for control loops.

### H.3 Analytics

- Python scripts for parsing outputs and plotting trajectories/signals.

## I. Reliability and Quality Considerations

### I.1 Positive Practices

- Anti-windup logic appears in control loops.
- Saturation limits are used for actuator safety.
- Separate logs and tuning outputs retained.

### I.2 Risks

- Variant sprawl makes canonical code path unclear.
- Duplicate runtime/API copies can drift and break parity.
- Binary/extracted files obscure source-of-truth.
- Sparse top-level reproducibility documentation.

## J. Repository Maturity Assessment

- Control engineering maturity: High.
- Simulation maturity: Medium-High.
- Embedded integration maturity: Medium-High.
- Documentation and maintainability maturity: Medium-Low.

## K. Remediation Roadmap

### K.1 Immediate (1-2 days)

1. Keep one canonical hardware folder and one simulation folder.
2. Move extracted/binary-heavy content to explicit archive paths.
3. Add subproject READMEs with exact run/build steps.

### K.2 Short Term (1 week)

1. Refactor repeated PID logic into reusable components.
2. Standardize output naming with timestamp and gain profile metadata.
3. Add environment setup docs for Python/CoppeliaSim/Arduino.

### K.3 Medium Term (2-4 weeks)

1. Add tests for controller math and parser integrity.
2. Add CI checks for Python lint/unit tests.
3. Export versioned simulation scripts where possible.

## L. Suggested Canonical Project Layout

```
BalanceBot/
  firmware/
    production/
    experiments/
  simulation/
    scenes/
    python_controllers/
  control_theory/
    models/
    lqr/
  data/
    logs/
    analysis/
  docs/
    architecture/
    tuning/
    deployment/
  archive/
```

## M. Final Conclusion

BalanceBot is technically rich and demonstrates strong robotics-control competence across embedded systems, simulation, and analytical control design. The primary next step is repository consolidation and standardization to convert this into a clean, reproducible, maintainable engineering codebase.
