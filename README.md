# Modern Robotics Capstone Project

A comprehensive kinematic simulator and control system for the youBot mobile manipulator, implementing the complete Modern Robotics capstone project pipeline. This repository provides a full-stack robotics solution including forward kinematics simulation, trajectory generation, and advanced feedback control.

## Overview

This project implements a complete robotics control pipeline for the KUKA youBot mobile manipulator, covering three major milestones:

- **Milestone 1**: Kinematic simulator with SE(3) chassis dynamics and joint integration
- **Milestone 2**: Eight-segment trajectory generator for pick-and-place operations  
- **Milestone 3**: Feed-forward + PI task-space control with comprehensive visualization

## Key Features

### 🤖 **Complete Robot Simulation**
- SE(3)-based chassis kinematics with omnidirectional wheel dynamics
- 5-DOF manipulator arm forward kinematics and Jacobian computation
- Speed limiting and realistic physical constraints

### 📈 **Advanced Trajectory Generation**
- Eight-segment pick-and-place trajectory planning
- Multiple time-scaling methods (cubic, quintic, trapezoid)
- Automatic timing optimization based on velocity and acceleration limits
- CoppeliaSim Scene 8 compatibility

### 🎯 **Sophisticated Control System**
- Feed-forward + PI task-space control implementation
- Mobile manipulator Jacobian with base and arm coupling
- Real-time error tracking and integral action
- Comprehensive visualization and analysis tools

### 📊 **Visualization & Analysis**
- 6-panel control analysis (position/orientation errors, commands, velocities)
- 3D trajectory visualization with automatic 2D fallback
- Robot configuration plotting and gain comparison tools
- Performance analysis for feedforward vs feedback control

## Requirements

- **Python 3.10+** 
- **NumPy** for numerical computations
- **modern_robotics** library for SE(3) operations
- **matplotlib** (optional) for visualization capabilities
- **pytest** for testing

## Quick Start

### Installation
```bash
# Clone the repository
git clone https://github.com/hafnium49/modern_robotics_capstone_project.git
cd modern_robotics_capstone_project

# Install dependencies
pip install -r requirements.txt
```

### Basic Usage
```bash
# Run all tests to verify installation
pytest -q

# Generate trajectory for CoppeliaSim Scene 8
python -m modern_robotics_sim.driver initial_config.csv output.csv \
    --controls 10 10 10 10 0 0 0 0 0 --speed-limit 20

# Test complete control system with visualization
python -m pytest tests/test_milestone3.py -k "visualization" -v
```

### Testing Individual Components
```bash
# Test kinematic simulator (Milestone 1)
pytest tests/test_milestone1.py -v

# Test trajectory generator (Milestone 2)  
pytest tests/test_milestone2.py -v

# Test feedback control system (Milestone 3)
pytest tests/test_milestone3.py -v
```

---

## Milestone 1 Requirements – "NextState" Simulator

### Public API

```python
NextState(config, controls, dt, speed_limit)  # → new_config
```

| Parameter     | Size | Units   | Description                                          |
| ------------- | ---- | ------- | ---------------------------------------------------- |
| `config`      | 12   | rad, m  | Current robot state **\[φ, x, y, θ₁–θ₅, w₁–w₄]**     |
| `controls`    | 9    | rad s⁻¹ | Desired speeds **\[u₁–u₄, θ̇₁–θ̇₅]**                 |
| `dt`          |  1   |  s      | Integration step (0.01 s for all tests)              |
| `speed_limit` |  1   | rad s⁻¹ | Symmetric bound applied to every entry of `controls` |

Return: new 12‑vector in the same order as `config`.

### Hard‑coded geometry constants

| Symbol | Value    | Meaning                 |
| ------ | -------- | ----------------------- |
| `r`    | 0.0475 m | Wheel radius            |
| `l`    | 0.2350 m | Half front–back spacing |
| `w`    | 0.1500 m | Half left–right spacing |
| `h`    | 0.0963 m | Frame {b} height        |

### Chassis kinematics

Body‑twist from wheel speeds:

```
F = (r/4) * [[-1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w)],
             [      1,        1,        1,        1],
             [     -1,        1,       -1,        1]]   # (3×4)

V_b = F · u                # [ω, v_x, v_y]
```

Pose update with the SE(3) exponential:

```
V_b6 = [0, 0, ω, v_x, v_y, 0]            # planar chassis
T_sb_new = T_sb_old · exp(se(3){V_b6} · dt)
```

Extract
`φ = atan2(T[1,0], T[0,0])`,
`x = T[0,3]`, `y = T[1,3]`.

### Wheel & arm integration

```
θ_new  = θ_old  + clip(θ̇_cmd, -ω_max, ω_max) * dt
w_new  = w_old  + clip(u_cmd , -ω_max, ω_max) * dt
```

### CSV writer format

Each output row for Scene 6 must contain **13 values**

```
φ, x, y, θ1, θ2, θ3, θ4, θ5, w1, w2, w3, w4, gripper_state
```

`gripper_state` is fixed at 0 for Milestone 1.

### Required sanity checks

Running `NextState` for 1 s (100 steps, dt = 0.01 s) with the following wheel speeds must yield:

| Wheel speeds (rad/s) | Expected chassis motion |
| -------------------- | ----------------------- |
| ( 10,  10,  10,  10) |  x ≈ +0.475 m           |
| (‑10,  10, ‑10, 10)  |  y ≈ +0.475 m           |
| (‑10,  10,  10,‑10)  |  φ ≈ +1.234 rad         |

Unit tests in `tests/test_milestone1.py` assert these values (±1 mm / 1 mrad).

### Testing the sample controls

**Option 1: Run automated tests**
```bash
pytest tests/test_milestone1.py -v
```

**Option 2: Generate CSV files for CoppeliaSim Scene 6**

Use the driver module to generate trajectory files for visual verification:

```bash
# Test 1: Forward motion (+x̂_b direction)
python -m modern_robotics_sim.driver milestone1/initial_config.csv test1_forward.csv \
    --controls 10 10 10 10 0 0 0 0 0 --steps 100 --dt 0.01

# Test 2: Sideways motion (+ŷ_b direction) 
python -m modern_robotics_sim.driver milestone1/initial_config.csv test2_sideways.csv \
    --controls -10 10 -10 10 0 0 0 0 0 --steps 100 --dt 0.01

# Test 3: Counter-clockwise rotation
python -m modern_robotics_sim.driver milestone1/initial_config.csv test3_rotation.csv \
    --controls -10 10 10 -10 0 0 0 0 0 --steps 100 --dt 0.01

# Test 4: Speed-limited forward motion (half distance)
python -m modern_robotics_sim.driver milestone1/initial_config.csv test4_limited.csv \
    --controls 10 10 10 10 0 0 0 0 0 --steps 100 --dt 0.01 --speed-limit 5
```

Load these CSV files in CoppeliaSim Scene 6 to visually verify the robot motions match the expected behaviors.

---

Satisfying every item above completes Milestone 1 and provides a drop‑in simulator for the remaining milestones.

---

## Milestone 2 – "TrajectoryGenerator" for the eight‑segment pick‑and‑place path

This specification tells you **exactly** what the autograder (and your later milestones) expect from the Milestone 2 code.  Follow it literally and your generator will plug‑and‑play with the rest of the project.

---

### 1  Public API

```text
TrajectoryGenerator(T_se_init,
                    T_sc_init,
                    T_sc_goal,
                    T_ce_grasp,
                    T_ce_standoff,
                    k           = 1,
                    method      = "quintic",
                    v_max       = 0.1,
                    omega_max   = 0.5,
                    gripper_dwell = 0.625)
        → traj_array   (N × 13)
```

| Parameter       | Type    | Unit    | Default   | Meaning                                                |
| --------------- | ------- | ------- | --------- | ------------------------------------------------------ |
| `T_se_init`     | 4 × 4   | —       | —         | End‑effector SE(3) at **start of segment 1**           |
| `T_sc_init`     | 4 × 4   | —       | —         | Cube initial pose (from Scene 6 defaults)              |
| `T_sc_goal`     | 4 × 4   | —       | —         | Cube goal pose                                         |
| `T_ce_grasp`    | 4 × 4   | —       | —         | EE pose **relative to cube** when grasped              |
| `T_ce_standoff` | 4 × 4   | —       | —         | EE pose **relative to cube** for both standoffs        |
| `k`             | int ≥ 1 | —       | 1         | # reference way‑points per 0.01 s (servo subdivision)  |
| `method`        | str     | —       | "quintic" | Time‑scaling: `"cubic"`, `"quintic"`, or `"trapezoid"` |
| `v_max`         | float   | m s⁻¹   | 0.1       | Linear speed cap used to size segment times            |
| `omega_max`     | float   | rad s⁻¹ | 0.5       | Angular speed cap for segment times                    |
| `gripper_dwell` | float   | s       | 0.625     | Time to keep pose constant during open/close           |

Return value `traj_array` contains **N = Σ segments · duration/0.01 · k** rows.
Each row is

```
r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,grip
```

where `grip` is **0 (open)** or **1 (closed)**.
A CSV writer that simply dumps `traj_array` row‑by‑row will satisfy Scene 8 and later milestones.

---

### 2  Fixed numeric constants

These **must** be hard‑coded (but you may expose them as keyword parameters with the same defaults).

| Constant       | Value           | Reason                                                                      |
| -------------- | --------------- | --------------------------------------------------------------------------- |
| `dt_ref`       | **0.01 s**      | Reference frame in which CoppeliaSim samples the CSV                        |
| `open_state`   | **0**           | Scene convention                                                            |
| `closed_state` | **1**           | Scene convention                                                            |
| `k ≥ 1`        | user‑selectable |  1 gives exactly one row per 0.01 s; the autograder accepts any integer ≥ 1 |

---

### 3  Eight canonical segments

| # | From pose       | To pose                     | Gripper bit | Notes on duration                                                            |
| - | --------------- | --------------------------- | ----------- | ---------------------------------------------------------------------------- |
| 1 | `T_se_init`     | `T_sc_init · T_ce_standoff` | 0           | Choose **T₁ = max(dist/v\_max, rot/ω\_max)** then round up to nearest 0.01 s |
| 2 | standoff ↓      | `T_sc_init · T_ce_grasp`    | 0           | Pure Z‑translation: default **1.0 s** if you prefer a fixed time             |
| 3 | grasp pose      | grasp pose                  | **0 → 1**   | `gripper_dwell` seconds (≥ 63·dt\_ref rows)                                  |
| 4 | grasp pose      | standoff above cube         | 1           | Same rules as #2                                                             |
| 5 | standoff @ init | `T_sc_goal · T_ce_standoff` | 1           | Plan like #1                                                                 |
| 6 | standoff ↓      | `T_sc_goal · T_ce_grasp`    | 1           | Same rules as #2                                                             |
| 7 | drop pose       | drop pose                   | **1 → 0**   | `gripper_dwell` seconds                                                      |
| 8 | drop pose       | standoff above goal         | 0           | Same rules as #2                                                             |

> **Time‑scaling inside each moving segment**
> Generate a geometric path (screw interpolation or straight‑line translation + constant rotation) and wrap it with the chosen `method`:
>
> * *quintic* ⇒ `s(t) = 10(τ³) – 15(τ⁴) + 6(τ⁵)`
> * *cubic*  ⇒ `s(t) = 3(τ²) – 2(τ³)`
> * *trapezoid* ⇒ constant‑accel / cruise / constant‑dec‑accel profile with automatically computed `t_accel` so `|ṡ|≤1`.

`τ = t / T_segment`.

---

### 4  Implementation checklist

1. **Helper** `ScrewTrajectory(X_start, X_end, T, N, method_flag)` from the MR library does both interpolation and time‑scaling; pass

   * `method_flag = 3` for cubic, `5` for quintic.
   * `N = (T / dt_ref) · k`.

2. **Concatenate** the eight segment matrices; **skip** the duplicated first row of every *moving* segment (except segment 1) to avoid rest discontinuities.

3. **Gripper rows** are produced by repeating the final pose of the segment with the appropriate constant `grip` bit.

4. **Duration sizing**

   ```python
   def segment_time(X0, X1, v_max, omega_max):
       d_pos = np.linalg.norm(p1 - p0)
       d_ang = np.arccos(0.5*(np.trace(R0.T @ R1) - 1))  # angle between R0,R1
       return np.ceil(max(d_pos/v_max, d_ang/omega_max) / 0.01) * 0.01
   ```

   Use this for the four "big" moves (#1, #5) if you want automatic timing.

5. **Return** *only* `traj_array`.  CSV emission belongs in a short driver script (`--csv` flag) but is not graded in Milestone 2.

---

### 5  Sanity tests (autograder)

1. **Dimensions** – number of rows divisible by `k`, each row length 13.
2. **Endpoint poses** – first row equals `T_se_init`; row where segment 3 starts has `grip=1`; final row is standoff above goal with `grip=0`.
3. **Continuity** – orientation changes smoothly (no jumps > 5° between consecutive reference points when `k=1`).
4. **Timing** – every duplicate gripper segment contains at least `gripper_dwell / dt_ref · k` identical rows.

---

### 6  Testing Milestone 2

To verify your Milestone 2 trajectory generator with CoppeliaSim Scene 8:

#### Step 1: Generate the trajectory CSV

You have two options for generating and verifying trajectories:

**Option 1: Run automated tests**
```bash
pytest tests/test_milestone2.py -v
```

**Option 2: Generate CSV files manually**

Run the driver script to generate your trajectory CSV file:

```bash
python modern_robotics_sim/driver.py --milestone 2 --csv
```

This will create `milestone2/eight_segment_traj.csv` using the **default cube configuration**:
- **Initial cube pose**: x=1.0m, y=0.0m, θ=0 radians
- **Goal cube pose**: x=0.0m, y=-1.0m, θ=-π/2 radians (-1.571 radians)

For a **custom cube configuration**, you can edit the values in `driver.py` or create your own test script. For example:

```python
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator

# Custom configuration
cube_initial = (0.5, 0.5, 0)           # x=0.5m, y=0.5m, θ=0 radians
cube_goal = (-0.5, 0.5, np.pi/4)       # x=-0.5m, y=0.5m, θ=π/4 radians (0.785 radians)

tg = TrajectoryGenerator()
trajectory = tg.TrajectoryGenerator(
    Tse_initial=tg.get_Tse_initial(),
    Tsc_initial=tg.get_Tsc_from_pose(*cube_initial),
    Tsc_final=tg.get_Tsc_from_pose(*cube_goal),
    Tce_grasp=tg.get_Tce_grasp(),
    Tce_standoff=tg.get_Tce_standoff(),
    k=1
)

# Save to CSV
import numpy as np
np.savetxt('milestone2/custom_trajectory.csv', trajectory, delimiter=',')
```

#### Step 2: Load the CSV in CoppeliaSim

1. Open Scene 8 in CoppeliaSim
2. Use the scene's built-in CSV loading mechanism to import your trajectory file
3. The robot should follow the generated trajectory

#### Step 3: Set cube poses in the UI

**Important**: You must manually set the cube poses in the CoppeliaSim UI to match your trajectory's assumptions:

1. **Select the cube object** in the scene hierarchy
2. **Right-click** → "Object/item properties" or press **Ctrl+D**
3. In the **Position** tab, set the cube's initial position to match your `Tsc_initial`:
   - For default config: **X = 1.0, Y = 0.0, Z = 0.025** (cube height)
4. In the **Orientation** tab, set the cube's initial orientation:
   - For default config: **α = 0, β = 0, γ = 0** (no rotation)
5. **Click "Apply"** to confirm the changes

**Coordinate conversion**:
- Your trajectory uses `(x, y, θ)` where θ is rotation about Z-axis in radians
- CoppeliaSim Scene 8 accepts: **x, y coordinates and θ (theta) in radians**
- CoppeliaSim uses `(X, Y, Z)` position and `(α, β, γ)` Euler angles
- Set **Z = 0.025** (cube rests on ground) and **γ = θ** (Z-rotation in radians)

#### Step 4: Verify the motion

1. **Start the simulation** in CoppeliaSim
2. The robot should:
   - Move to the cube's initial position
   - Grasp the cube (gripper closes)
   - Transport it to the goal location
   - Release the cube (gripper opens)
   - Return to standoff position
3. **Check timing**: The gripper should dwell for at least 0.63 seconds during grasp/release operations

---

Implementing exactly these rules will make your Milestone 2 code compatible with the later **FeedbackControl** loop and with the Coursera autograder.

---

## Milestone 3 – **Feed‑Forward + PI Task‑Space Control**

Your submission must contain **one callable function** (you may of course split the work across helpers) that turns the next two poses on the reference trajectory into wheel and joint rates for one control cycle, plus any state you need to carry between cycles.  The Coursera autograder will run it in a loop together with the Milestone 1 simulator and the Milestone 2 trajectory generator.

---

### 1.  Public API the autograder will call

```python
def FeedbackControl(
        X_actual,            # 4×4 SE(3) matrix – current end‑effector pose
        X_desired,           # 4×4 – reference pose at time step i
        X_desired_next,      # 4×4 – reference pose at time step i+1 (Δt later)
        Kp, Ki,              # 6×6 diagonal gain matrices
        dt,                  # scalar time step (seconds)
        integral_error_prev  # 6‑vector carried over from last call
    ):
    """
    Returns
    -------
    V_cmd : 6‑vector  – commanded twist in frame {e}
    controls : 9‑vector – [u1 u2 u3 u4 θ̇1 … θ̇5]
    X_err : 6‑vector  – twist that takes X_actual to X_desired
    integral_error_new : 6‑vector – updated ∫X_err dt
    """
```

*Write the helper that turns `V_cmd` into the 9‑vector exactly as shown in Eq. (13.37) of the book:*

```
[u  θ̇]^T  =  J_e(θ,φ)†  ·  V_cmd
```

---

### 2.  Fixed constants your code **must** use

| Symbol           | Value                                                                                                               | Meaning                                            | Source              |
| ---------------- | ------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- | ------------------- |
| `r`              | **0.0475 m**                                                                                                        | wheel radius                                       | youBot spec         |
| `l`              | **0.235 m**                                                                                                         | half front‑back wheel separation (2 l = 0.47 m)    | fig. "wheel layout" |
| `w`              | **0.150 m**                                                                                                         | half side‑side wheel separation (2 w = 0.30 m)     | same                |
| `Tb0`            | `[[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]]`                                                               | chassis‑to‑arm‑base transform                      | wiki page           |
| `M0e`            | `[[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]]`                                                                | home pose of the gripper in frame {0}              | wiki page           |
| `Blist`          | columns:  $(0,0,1, 0,0.033,0); (0,-1,0, -0.5076,0,0); (0,-1,0, -0.3526,0,0); (0,-1,0, -0.2176,0,0); (0,0,1, 0,0,0)$ | body screw axes of the 5 joints (expressed in {e}) | wiki page           |
| `F6`             | see §2.2 below                                                                                                      | 6 × 4 matrix mapping wheel rates → chassis twist   | derived below       |
| `dt`             | **0.010 s**                                                                                                         | controller sample time (matches csv timing)        | project spec        |
| `speed_limit`    | **12.3 rad s‑1** (apply to each wheel and joint)                                                                    | Milestone 1 text                                   |                     |
| `pinv_tolerance` | **1 × 10‑3** (singular values < tol → 0)                                                                            | guideline in "Singularities" section               |                     |

#### 2.1  Base inverse‑kinematics constants

Number the wheels as shown in the wiki (front‑left = 1 counter‑clockwise).
The **3 × 4** matrix $H^{(0)}$ that converts chassis twist *V<sub>b</sub>* to wheel speeds *u* is

$$
H^{(0)} = \frac{1}{r}
\begin{bmatrix}
 -1 &  1 &  1 & -1 \\
  1 &  1 &  1 &  1 \\
 \tfrac{1}{l+w} & -\tfrac{1}{l+w} & \tfrac{1}{l+w} & -\tfrac{1}{l+w}
\end{bmatrix}
$$

You need its **pseudo‑inverse** once per cycle:

$$
F = (H^{(0)})^{\dagger}      \qquad\text{(3 × 4 → gives planar twist from wheel rates)}
$$

Embed that in 6‑D:

```
F6 = vstack([ [0,0,0,0],    # ωx
              [0,0,0,0],    # ωy
              [0,0,0,0],    # ωz ( already in Vb )
              F ])          # vx vy ωz rows
```

#### 2.2  Mobile‑manipulator Jacobian

Build once per call:

```
Tsb = ChassisToSE3(phi, x, y)           # from Milestone 1
T0e = FKinBody(M0e, Blist, theta)
Tse = Tsb @ Tb0 @ T0e

# Base Jacobian columns (6×4)
J_base = Adjoint(inv(T0e) @ inv(Tb0)) @ F6

# Arm Jacobian columns (6×5)
J_arm  = JacobianBody(Blist, theta)

J_e = hstack([J_base, J_arm])           # 6×9
```

---

### 3.  Control‑law equations you must implement

1. **Feed‑forward twist**

   ```
   Vd = (1/dt) * se3ToVec( MatrixLog6(inv(X_desired) @ X_desired_next) )
   ```

2. **Configuration error (twist)**

   ```
   X_err = se3ToVec( MatrixLog6( inv(X_actual) @ X_desired ) )
   ```

3. **Integral update**

   ```
   integral_error_new = integral_error_prev + X_err * dt
   ```

4. **Commanded twist (body frame)**

   ```
   V_cmd = Adjoint( inv(X_actual) @ X_desired ) @ Vd \
           + Kp @ X_err \
           + Ki @ integral_error_new
   ```

5. **Wheel + joint rates**

   ```
   controls_raw = pinv(J_e, rcond=pinv_tolerance) @ V_cmd
   controls = clip(controls_raw, -speed_limit, speed_limit)
   ```

Return `(V_cmd, controls, X_err, integral_error_new)`.

---

### 4.  Required gain choices for grading

You **must** expose both gains as 6×6 diagonal matrices:

```python
Kp = diag([5,5,5,5,5,5])     # reasonable starting point
Ki = diag([0,0,0,0,0,0])     # set to zero for initial tests
```

The autograder will test with several gain sets, so read `Kp` and `Ki` exactly as passed.

---

### 5.  Other implementation rules

* Use the **ModernRobotics** routines for `Adjoint`, `MatrixLog6`, `se3ToVec`,
  `JacobianBody`, and `FKinBody`.
* Clamp speeds **before** calling `NextState`.
* Do **not** integrate or filter inside `FeedbackControl`—all dynamics are handled
  by your Milestone 1 `NextState`.
* Keep a **module‑level (or class) variable** to store the running integral; the autograder
  will call your function sequentially with the updated value you return.
* Your code must run in plain Python 3 with only **NumPy** and the
  **modern\_robotics.py** helper file supplied in Course 1.

That's every numeric constant, matrix, tolerance, and gain interface the grader expects for Milestone 3.  Stick to this spec and your controller will plug cleanly into the rest of the capstone pipeline.

---

### 6. Testing Feedforward Control for Milestone 3

To verify your Milestone 3 implementation and understand the behavior of feedforward-only control, you should test the controller with **Kp = Ki = 0** (feedforward only) before adding feedback gains.

#### Step 1: Run Feedforward Control Tests

You have two options for testing feedforward control:

**Option 1: Run automated tests**
```bash
# Run all Milestone 3 tests including feedforward tests
python -m pytest tests/test_milestone3.py -v

# Run only the feedforward control tests
python -m pytest tests/test_milestone3.py -k "feedforward" -v

# Run visualization tests (requires matplotlib)
python -m pytest tests/test_milestone3.py -k "visualization" -v
```

This will automatically:
- Run 25 comprehensive tests (18 validation tests + 6 visualization tests + 1 CSV generation test)
- Generate CSV files for different initial error conditions in `milestone3_feedforward_tests/`
- Create comprehensive visualizations showing control behavior
- Verify all functionality and file generation

**Option 2: Execute feedback control module manually with visualization**

Run the feedback control module directly to test feedforward behavior with comprehensive visualizations:

```bash
# Run with automatic visualization
python -m modern_robotics_sim.feedback_control --feedforward-test --visualize

# Run with custom scenarios and plotting
python -m modern_robotics_sim.feedback_control --feedforward-test --initial-error 0.1 0.05 0.02 --plot-results
```

Generate and visualize control analysis using the test framework:

```python
# Generate comprehensive control analysis with visualizations
python -c "
import sys
sys.path.append('tests')
from test_milestone3 import *
import numpy as np

# Test with visualization
trajectory = create_simple_trajectory()

# Compare feedforward vs feedback control
results_ff = simulate_control_loop(
    trajectory,
    Kp=np.diag([0, 0, 0, 0, 0, 0]),      # Feedforward only
    Ki=np.diag([0, 0, 0, 0, 0, 0]),
    duration_seconds=2.0,
    initial_error=[0.1, 0.05, 0.02]
)

results_fb = simulate_control_loop(
    trajectory,
    Kp=np.diag([5, 5, 5, 5, 5, 5]),      # With feedback
    Ki=np.diag([1, 1, 1, 1, 1, 1]),
    duration_seconds=2.0,
    initial_error=[0.1, 0.05, 0.02]
)

# Generate comprehensive visualizations
plot_control_analysis(results_ff, 'Feedforward Control Analysis')
plot_control_analysis(results_fb, 'Feedback Control Analysis')
plot_gain_comparison([results_ff, results_fb], ['Feedforward', 'Feedback'])
plot_trajectory_comparison([results_ff, results_fb], ['Feedforward', 'Feedback'])
plot_3d_trajectory([results_ff, results_fb], ['Feedforward', 'Feedback'])
"
```

**Available visualization functions:**
- `plot_robot_configuration()` - Robot pose and configuration visualization
- `plot_control_analysis()` - 6-panel control analysis (errors, commands, speeds)
- `plot_gain_comparison()` - Performance comparison between different gains
- `plot_trajectory_comparison()` - End-effector trajectory comparison
- `plot_3d_trajectory()` - 3D trajectory visualization with fallback to 2D

Both options create CSV files for different initial error conditions:
- `feedforward_perfect_initial.csv` - Perfect initial end-effector position
- `feedforward_small_error.csv` - Small initial error (5cm translation)
- `feedforward_medium_error.csv` - Medium initial error (10cm translation)  
- `feedforward_large_error.csv` - Large initial error (20cm translation)
- `feedforward_test_report.txt` - Comprehensive testing instructions

The feedforward tests verify:
- **Perfect initial conditions**: Feedforward control with no initial error
- **Initial end-effector errors**: How feedforward control handles various initial position errors
- **Trajectory following**: Feedforward control's ability to follow reference trajectories
- **Comparison with feedback**: Behavior differences between feedforward-only and feedforward+feedback control
- **Visual analysis**: Comprehensive plotting and visualization of control behavior

#### Step 2: Expected Feedforward Control Behavior

**Key observations you should make:**

1. **Error Persistence**: With feedforward-only control (Kp=Ki=0), initial end-effector errors **persist** throughout the trajectory. The robot cannot correct for initial positioning errors.

2. **Trajectory Following**: The robot can follow the desired trajectory motion (feedforward component) but cannot compensate for deviations from the reference path.

3. **No Steady-State Correction**: Unlike feedback control, feedforward control cannot drive steady-state errors to zero.

**Performance with different initial errors:**
- **Perfect initial**: Should follow trajectory closely and complete pick-and-place task
- **Small error (5cm)**: Small deviation but may still grasp cube successfully  
- **Medium error (10cm)**: Larger deviation, may not grasp cube perfectly
- **Large error (20cm)**: Significant deviation, likely to miss cube entirely

#### Step 3: Add Feedback Control Gains

After verifying feedforward behavior, test with non-zero feedback gains:

```python
# Test with proportional control
Kp = np.diag([5, 5, 5, 5, 5, 5])    # Add proportional feedback
Ki = np.diag([0, 0, 0, 0, 0, 0])    # No integral action yet

# Test with PI control  
Kp = np.diag([5, 5, 5, 5, 5, 5])    # Proportional feedback
Ki = np.diag([1, 1, 1, 1, 1, 1])    # Add integral action
```

**Expected improvements with feedback:**
- **Error correction**: Initial errors should decrease over time
- **Steady-state accuracy**: Better final positioning accuracy
- **Disturbance rejection**: Ability to handle unexpected perturbations

#### Step 4: Analyze Control Performance with Visualization

Use the comprehensive plotting and analysis utilities to understand control behavior:

```python
# Generate and analyze simulation results with visualization
python -c "
import sys
sys.path.append('tests')
from test_milestone3 import *
import numpy as np

# Create test trajectory
trajectory = create_simple_trajectory()

# Test feedforward-only control
results_ff = simulate_control_loop(
    trajectory,
    Kp=np.diag([0, 0, 0, 0, 0, 0]),      # No feedback
    Ki=np.diag([0, 0, 0, 0, 0, 0]),
    duration_seconds=1.0,
    initial_error=[0.1, 0.05, 0.02]       # 10cm initial error
)

# Test feedback control
results_fb = simulate_control_loop(
    trajectory,
    Kp=np.diag([5, 5, 5, 5, 5, 5]),      # With feedback
    Ki=np.diag([1, 1, 1, 1, 1, 1]),
    duration_seconds=1.0,
    initial_error=[0.1, 0.05, 0.02]       # Same initial error
)

# Generate comprehensive visualizations
try:
    # Control analysis plots (6-panel analysis)
    plot_control_analysis(results_ff, 'Feedforward Control Analysis')
    plot_control_analysis(results_fb, 'Feedback Control Analysis')
    
    # Performance comparison plots
    plot_gain_comparison([results_ff, results_fb], ['Feedforward', 'Feedback'])
    plot_trajectory_comparison([results_ff, results_fb], ['Feedforward', 'Feedback'])
    
    # 3D trajectory visualization
    plot_3d_trajectory([results_ff, results_fb], ['Feedforward', 'Feedback'])
    
    # Robot configuration visualization
    plot_robot_configuration(results_fb['configs'][-1], 'Final Robot Configuration')
    
    print('All visualizations generated successfully')
except ImportError:
    print('Matplotlib not available - visualizations skipped')
    print('Install matplotlib with: pip install matplotlib')
"
```

**Available visualization analysis:**

1. **Control Analysis** (`plot_control_analysis`): 6-panel analysis showing:
   - Position errors (X, Y, Z translation)
   - Orientation errors (Roll, Pitch, Yaw rotation)  
   - Control commands over time
   - Joint velocities and wheel speeds
   - Error evolution and convergence

2. **Gain Comparison** (`plot_gain_comparison`): Performance comparison between different control gains showing error reduction effectiveness

3. **Trajectory Comparison** (`plot_trajectory_comparison`): End-effector path comparison between feedforward and feedback control

4. **3D Trajectory Visualization** (`plot_3d_trajectory`): 3D visualization of robot end-effector trajectory with automatic fallback to 2D plotting

5. **Robot Configuration** (`plot_robot_configuration`): Current robot pose and joint configuration visualization

This feedforward testing methodology helps you understand the fundamental differences between feedforward and feedback control, which is essential for tuning your complete Milestone 3 controller.

---

## Milestone 3 Implementation Summary

### Files and Structure

The Milestone 3 implementation provides a complete **Feed-Forward + PI Task-Space Control** system for the youBot mobile manipulator:

#### Core Implementation: `modern_robotics_sim/feedback_control.py`
**Primary module containing:**
- `FeedbackControl()` function - Main control function matching exact API specification
- `FeedbackController` class - Stateful wrapper for easier use in simulation loops
- All required helper functions (Jacobian computation, chassis kinematics, etc.)
- Fixed constants exactly as specified in requirements

**Key Features:**
- **Exact API compliance**: Returns `(V_cmd, controls, X_err, integral_error_new)` as specified
- **Speed limiting**: 12.3 rad/s per specification with proper pseudo-inverse handling
- **Constants compliance**: All hardcoded constants match specification exactly

#### Comprehensive Testing: `tests/test_milestone3.py`
**Test suite covering:**
- API compliance tests and gain matrix behavior verification
- Integral error accumulation and speed limiting validation
- Integration with Milestone 1 (NextState) and Milestone 2 (TrajectoryGenerator)
- **Feedforward control testing** with multiple scenarios
- **Comprehensive visualization capabilities** with matplotlib integration
- Complete milestone integration testing

**Visualization Features:**
- `plot_robot_configuration()` - Robot pose and joint configuration plotting
- `plot_control_analysis()` - 6-panel control analysis (errors, commands, speeds)
- `plot_gain_comparison()` - Performance comparison between different control gains
- `plot_trajectory_comparison()` - End-effector trajectory comparison
- `plot_3d_trajectory()` - 3D trajectory visualization with 2D fallback
- Graceful degradation when matplotlib is not available

### Control Law Implementation

The implementation follows the exact control equations specified:

✅ **Feed-forward twist:** `Vd = (1/dt) * se3ToVec(MatrixLog6(inv(X_desired) @ X_desired_next))`
✅ **Configuration error:** `X_err = se3ToVec(MatrixLog6(inv(X_actual) @ X_desired))`  
✅ **Integral update:** `integral_error_new = integral_error_prev + X_err * dt`
✅ **Commanded twist:** `V_cmd = Adjoint(inv(X_actual) @ X_desired) @ Vd + Kp @ X_err + Ki @ integral_error_new`
✅ **Wheel + joint rates:** `controls = clip(pinv(J_e) @ V_cmd, -speed_limit, speed_limit)`

### Usage Examples

#### Basic Function Call
```python
from modern_robotics_sim.feedback_control import FeedbackControl
import numpy as np

# Set up poses and gains
X_actual = np.eye(4)
X_desired = np.eye(4); X_desired[0,3] = 0.1  # 10cm forward
X_desired_next = X_desired
Kp = np.diag([5,5,5,5,5,5])
Ki = np.diag([0,0,0,0,0,0])

# Compute control
V_cmd, controls, X_err, integral_error = FeedbackControl(
    X_actual, X_desired, X_desired_next, Kp, Ki, 0.01, np.zeros(6)
)
```

#### Stateful Controller
```python
from modern_robotics_sim.feedback_control import FeedbackController

controller = FeedbackController()  # Uses default gains
config = np.zeros(12)  # Robot configuration

V_cmd, controls, X_err = controller.control(
    X_actual, X_desired, X_desired_next, config
)
```

#### Integration with Other Milestones
```python
from modern_robotics_sim.next_state import NextState
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator

# Generate trajectory (Milestone 2)
trajectory = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff)

# Control loop
controller = FeedbackController()
config = np.zeros(12)

for i in range(len(trajectory)-1):
    # Extract desired poses from trajectory
    X_desired = extract_pose_from_trajectory(trajectory[i])
    X_desired_next = extract_pose_from_trajectory(trajectory[i+1])
    X_actual = compute_current_ee_pose(config)  # From forward kinematics
    
    # Compute control (Milestone 3)
    V_cmd, controls, X_err = controller.control(X_actual, X_desired, X_desired_next, config)
    
    # Update robot state (Milestone 1)
    config = NextState(config, controls, 0.01, 12.3)
```

### Feedforward Control Testing Details

The implementation includes specialized **feedforward control testing** as required:

#### Feedforward Test Functions in `test_milestone3.py`:

1. **`test_feedforward_only_perfect_initial()`** - Tests feedforward control with perfect initial configuration (Kp=Ki=0)
2. **`test_feedforward_with_initial_error()`** - Tests feedforward control with initial end-effector errors
3. **`test_feedforward_trajectory_following()`** - Tests trajectory following ability with different speed limits
4. **`test_feedforward_vs_feedback_comparison()`** - Compares feedforward-only vs feedforward+feedback control

#### Visualization Test Functions in `test_milestone3.py`:

5. **`test_visualization_robot_configuration()`** - Tests robot configuration plotting functionality
6. **`test_visualization_control_analysis()`** - Tests 6-panel control analysis visualization
7. **`test_visualization_gain_comparison()`** - Tests gain comparison plotting
8. **`test_visualization_trajectory_comparison()`** - Tests trajectory comparison visualization
9. **`test_visualization_3d_trajectory()`** - Tests 3D trajectory plotting with fallback
10. **`test_visualization_comprehensive()`** - Tests all visualization functions together

#### Key Findings from Feedforward Testing:

- **Perfect Initial Conditions**: Average pose error ~0.0016, 0% control saturation
- **Initial Error Behavior**: Errors persist under feedforward-only control as expected
  - Small error (5.5cm) → persistent ~0.5cm error
  - Medium error (11.4cm) → persistent ~1.1cm error  
  - Large error (22.9cm) → persistent ~2.3cm error
- **Speed Limit Effects**: All tested speeds (5.0, 12.3, 20.0 rad/s) work without saturation
- **Control Comparison**: Higher feedback gains produce larger control responses as expected

#### CSV Generation for CoppeliaSim Testing

The test suite includes utility functions for generating CSV files compatible with CoppeliaSim Scene 8:

```python
# Generate multiple feedforward test scenarios
from test_milestone3 import generate_comparison_csvs
results = generate_comparison_csvs('feedforward_outputs')
```

**Generated test files:**
- `feedforward_perfect_initial.csv` - Perfect initial conditions
- `feedforward_small_error.csv` - 5cm initial error
- `feedforward_medium_error.csv` - 10cm initial error
- `feedforward_large_error.csv` - 20cm initial error

### Verification and Compliance

- **All tests pass**: 25/25 comprehensive tests including feedforward scenarios and visualization features
- **API compliance**: Exact function signature and return value match
- **Constants compliance**: All physical constants match specification exactly
- **Integration verified**: Works correctly with Milestone 1 and 2 implementations
- **Visualization capabilities**: Comprehensive plotting and analysis tools with matplotlib integration
- **Autograder ready**: Designed for full Coursera autograder compatibility

The implementation demonstrates the fundamental differences between feedforward and feedback control, showing how feedforward control cannot correct initial errors while feedback control provides error correction and disturbance rejection.

---

## Final Milestone (Milestone 4) — **Software‐integration requirements**

Below is a concise checklist of **what your codebase must expose, how the pieces talk to each other, and the fixed numerical values that the autograder (and peer reviewers) will assume**.  Follow it literally to avoid "wrong signature / wrong units" errors.

| # | Element                                                      | Mandatory public signature\*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Key assumptions & fixed parameters                                                                                                                                                                                                                                                                                                                                                                                                                 |
| - | ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1 | **NextState** – one‐step kinematic simulator                 | `conf_next = NextState(conf, controls, dt=0.01, speed_limit=5.0)` <br> \* `conf` – 12‑vector ⟨ϕ, x, y, θ₁…θ₅, W₁…W₄⟩ <br> \* `controls` – 9‑vector ⟨u₁…u₄, θ̇₁…θ̇₅⟩ (rad s⁻¹) <br> \* `dt` – fixed at **0.01 s** <br> \* `speed_limit` – clamp magnitude of every wheel & joint rate                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | • Forward‑Euler integration. <br> • Wheel → body–twist map uses constants: <br>   r = 0.0475 m, l = 0.235 m (half of 0.47 m), w = 0.15 m (half of 0.30 m). <br> • Body‑twist → SE (3) pose update via matrix exponential.                                                                                                                                                                                                                          |
| 2 | **TrajectoryGenerator** – eight‑segment nominal path for {e} | `traj, csv_path = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1)` <br> returns an N × 13 matrix (r₁₁…r₃₃, pₓ, p\_y, p\_z, gripper) and writes the same rows to a CSV file                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Fixed transforms (SI units, radians): <br> \* `Tce_grasp` = diag⟨1,1,1⟩, p = \[0, 0, 0.02] m (cube halfway into fingers) <br> \* `Tce_standoff` = `Tce_grasp` shifted +\[0,0,0.10] m in z₍c₎ <br> \* Gripper open / close dwell = 0.625 s = 63 rows. <br> \* Time scaling – quintic (`QuinticTimeScaling`) unless otherwise stated. <br> \* Segment durations (recommended): up/down 1 s; long chassis moves ≥2 s (tune to stay within 5 rad s⁻¹). |
| 3 | **FeedbackControl** – task‑space feed‑forward + PI           | `V, X_err, int_err_next = FeedbackControl(X, Xd, Xd_next, Ki, Kp, dt, int_err)` <br> \* Frames expressed in **{e}**. <br> \* Returns 6‑vector body twist `V`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | • Control law (textbook Eq. 11.16):  $Ad(X⁻¹ Xd) V_d + K_p X_err + K_i ∫X_err dt$. <br> • Use `V_d = (1/dt) log(Xd⁻¹ Xd_next)`.                                                                                                                                                                                                                                                                                                                    |
| 4 | **youBotConst** – one source of truth for kinematic data     | Expose (or import) the constants: <br> `Blist  # 6×5 screw axes in {e}` <br> `M0e   # 4×4 home of {e} in {0}` <br> `Tb0   # fixed transform {b}->{0}`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Values from course page: <br> **B₁** = (0,0,1, 0, 0.033,0) <br> **B₂** = (0,−1,0, −0.5076,0,0) <br> **B₃** = (0,−1,0, −0.3526,0,0) <br> **B₄** = (0,−1,0, −0.2176,0,0) <br> **B₅** = (0,0,1, 0,0,0). <br> `M0e = [[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]]` <br> `Tb0 = [[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]]`                                                                                                              |
| 5 | **Main driver script** (e.g. `run_capstone.py`)              | No fixed signature; **must**: <br> 1. Hard‑code default cube poses: <br>   `Tsc_init = diag(1,1,1), p=[1,0,0.025]` m <br>   `Tsc_goal = [[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]` <br> 2. Choose an initial robot 13‑vector with ≥0.20 m position error **and** ≥30 deg orientation error from the first row of the reference trajectory. <br> 3. Generate nominal path → loop: <br>   • call **FeedbackControl** <br>   • map twist → wheel + joint rates with damped pseudoinverse **J\_e†** (`numpy.linalg.pinv`, tol ≥ 1e‑3) <br>   • clamp to 5 rad s⁻¹ <br>   • call **NextState** <br>   • log every step (or every *k*‑th step) to build the 13‑column csv <br>   • log 6‑vector `X_err` for plotting. <br> 4. Write two files into `results/best/` (and other sub‑tasks): <br>   `youBot_output.csv`, `Xerr_log.csv`. | • Loop runs `N-1` iterations, where `N` is trajectory rows. <br> • Use **dt = 0.01 s** throughout. <br> • Wheel + joint speed limit **u, θ̇ ≤ 5 rad s⁻¹** (modify if you use a different constant `speed_limit` but keep it consistent). <br> • Suggested gains: <br>   `Kp = diag([5,5,5,5,5,5])` <br>   `Ki = diag([0.5,0.5,0.5,0.5,0.5,0.5])` (tune as needed). |

\* **Language:** Python, MATLAB, or Mathematica are all accepted.  Signatures shown in Python style; use the closest analogue in your language.
\* **Return order:** stick to the order shown; the autograder imports by position, not keyword.

---

#### Additional coding constraints

* **Numerical safeguards**

  * Pseudoinverse tolerance ≥ 1 × 10⁻³; smaller singular values are treated as zero to suppress huge rates near singularities.
  * Clip after, not before, the pseudoinverse multiplication so that direction is preserved.

* **Logging**

  * Print at least one status line per run, e.g.

    ```
    Generating reference trajectory … done (2313 points)
    Simulating 2312 control steps … done
    CSV written to results/best/youBot_output.csv
    ```
  * Plot of all six components of `X_err` vs time saved as `Xerr_plot.pdf` (no proprietary format).

* **Directory layout to submit**

```
capstone_submission.zip
 ├── README.pdf
 ├── code/
 │    ├── youBotConst.py
 │    ├── NextState.py
 │    ├── TrajectoryGenerator.py
 │    ├── FeedbackControl.py
 │    └── run_capstone.py
 └── results/
      ├── best/
      │     ├── youBot_output.csv
      │     ├── Xerr_log.csv
      │     └── Xerr_plot.pdf
      ├── overshoot/          # similar files with deliberately lower gains
      └── newTask/            # user‑chosen cube poses
```

Follow these interface contracts and parameter values exactly; if the autograder can import and run the five public functions with the defaults above, and the generated csv animates successfully in Scene 6, you will satisfy Milestone 4.

---

## Testing Milestone 4 - Complete Integration

### Final Step: Completing the Project and Testing Your Submission

Now that feedforward control is working, you are ready to complete your project and test the full integrated system. The final milestone brings together all three previous milestones into a complete pick-and-place control system.

### Required Configuration for Testing

Use the **default initial configurations** for the cube in the capstone CoppeliaSim scene:
- **Initial cube configuration**: `(x, y, θ) = (1, 1 m, 0 m, 0 rad)`
- **Final cube configuration**: `(x, y, θ) = (0 m, −1 m, −π/2 rad)`

**Initial end-effector reference trajectory pose**:
```
T_se = [[0,  0,  1,  0  ],
        [0,  1,  0,  0  ],
        [-1, 0,  0,  0.5],
        [0,  0,  0,  1  ]]
```

**Critical requirement**: Choose an initial configuration of the youBot so that the end-effector has:
- **≥30 degrees of orientation error** from the reference trajectory
- **≥0.2 m of position error** from the reference trajectory

### Testing Options

#### **Option 1: Run Automated Tests**

```bash
# Run all Milestone 4 integration tests
pytest tests/test_milestone4.py -v

# Run specific test categories
pytest tests/test_milestone4.py -k "integration" -v
pytest tests/test_milestone4.py -k "control" -v
pytest tests/test_milestone4.py -k "full_system" -v
```

The automated tests verify:
- **Complete system integration** (Milestones 1, 2, 3 working together)
- **Initial error requirements** (≥30° orientation, ≥0.2m position error)
- **Control scenarios** (feedforward, proportional, PI, feedforward+PI)
- **File generation** (`youBot_output.csv`, `Xerr_log.csv`, `Xerr_plot.pdf`)
- **Error convergence** and trajectory following performance

#### **Option 2: Manual Execution and Progressive Controller Testing**

Follow the progressive testing methodology to understand controller behavior:

##### Step 1: Test Feedforward-Only Control

```bash
# Generate feedforward-only results (Kp = Ki = 0)
python main.py feedforward
```

**Expected behavior**:
- Robot follows trajectory motion but **cannot correct initial errors**
- Initial position/orientation errors **persist** throughout execution
- May fail to grasp cube if initial error is too large

##### Step 2: Add Proportional Control

```bash
# Test with small proportional gains first
python main.py proportional

# Test with the well-tuned "best" scenario
python main.py best
```

**Progressive gain tuning approach**:
1. **Start small**: `Kp = diag([1,1,1,1,1,1])`, `Ki = 0`
2. **Increase gradually**: `Kp = diag([3,3,3,3,3,3])`, `Ki = 0`  
3. **Final tuning**: `Kp = diag([5,5,5,5,5,5])`, `Ki = 0`

**Expected improvements**:
- **Error correction**: Initial errors should decrease over time
- **Better tracking**: Improved trajectory following accuracy
- **Grasp success**: Higher probability of successful cube manipulation

##### Step 3: Test Overshoot Behavior

```bash
# Test deliberately poor gains to observe overshoot
python main.py overshoot
```

**Purpose**: Demonstrate the effects of poorly tuned gains and the importance of proper controller design.

##### Step 4: Add Integral Control

```bash
# Test feedforward + PI control
python main.py feedforward_pi

# Test all control modes for comparison
python main.py all
```

### Verification Steps

#### 1. File Generation Verification

After each test, verify the required files are generated in `results/` directories:

```bash
# Check results structure
ls -la results/best/
# Should contain:
# - youBot_output.csv  (13-column robot trajectory)
# - Xerr_log.csv       (6-DOF error data)  
# - Xerr_plot.pdf      (Error convergence plot)
# - README.txt         (Scenario documentation)
# - program_log.txt    (Execution log)
```

#### 2. CoppeliaSim Animation Testing

1. **Load CoppeliaSim Scene 8**
2. **Set cube poses** to match your scenario:
   - Default: Initial at (1,0,0.025), Goal at (0,-1,0.025)
   - Custom: Set according to your newTask configuration
3. **Load CSV**: Import `youBot_output.csv` using scene's CSV import mechanism
4. **Run simulation** and verify:
   - Robot approaches cube at initial position
   - Gripper closes (grasp operation)
   - Robot transports cube to goal position  
   - Gripper opens (release operation)
   - Robot returns to standoff position

#### 3. Performance Analysis

**Key metrics to evaluate**:

```python
# Analyze error convergence
import numpy as np
error_data = np.loadtxt('results/best/Xerr_log.csv', delimiter=',')

# Check initial error requirements
initial_pos_error = np.linalg.norm(error_data[0, :3])  # Should be ≥ 0.2m
initial_rot_error = np.linalg.norm(error_data[0, 3:])  # Should be ≥ 30° = 0.524 rad

# Check final convergence
final_pos_error = np.linalg.norm(error_data[-1, :3])   # Should be near 0
final_rot_error = np.linalg.norm(error_data[-1, 3:])   # Should be near 0

print(f"Initial position error: {initial_pos_error:.3f} m (requirement: ≥0.2 m)")
print(f"Initial orientation error: {initial_rot_error:.3f} rad (requirement: ≥0.524 rad)")
print(f"Final position error: {final_pos_error:.6f} m")
print(f"Final orientation error: {final_rot_error:.6f} rad")
```

### Success Criteria

Your Milestone 4 implementation should demonstrate:

✅ **Error Requirements**: Initial configuration with ≥30° orientation error and ≥0.2m position error
✅ **Error Convergence**: Essentially all initial error driven to zero by end of first trajectory segment
✅ **Successful Grasp**: Robot successfully picks up and places the cube
✅ **File Generation**: All required output files created with correct format
✅ **CoppeliaSim Compatibility**: Generated CSV files animate correctly in Scene 8
✅ **Control Variants**: Demonstrable differences between best/overshoot performance

---

## Advanced Features ("Other Things to Try")

This implementation includes several advanced features inspired by the "Other Things to Try" section of the capstone requirements:

### 🔧 **Stationary Base Control**
Keep the mobile base stationary during manipulation segments (2, 4, 6, 8) while allowing movement during transit segments (1, 3, 5, 7).

```bash
python main.py stationary_base
```

Features:
- Enhanced manipulation precision
- Reduced base disturbances during grasping
- Optimized for delicate pick-and-place operations

### ⚖️ **Motion Preference Control**
Use weighted pseudoinverse to prefer wheel motions over joint motions or vice versa.

```bash
python main.py motion_preference
```

Features:
- Weighted pseudoinverse Jacobian computation
- Separate scenarios for wheel vs joint preference
- Demonstrates redundancy resolution strategies

### ⚠️ **Joint Limit Enforcement**
Enforce realistic joint limits with safety margins during trajectory execution.

```bash
python main.py joint_limits
```

Features:
- youBot arm joint limit enforcement
- 5-degree safety margins
- Graceful handling of constrained motions

### 🎯 **Singularity Avoidance**
Robust control behavior near singular arm configurations.

```bash
python main.py singularity_avoidance
```

Features:
- Damped least squares inverse near singularities
- Real-time manipulability monitoring
- Graceful degradation in ill-conditioned poses

### 🏀 **Block Throwing**
Plan and execute ballistic trajectories to throw the block to a desired landing point.

```bash
python main.py block_throwing
```

Features:
- Ballistic physics calculations
- Target landing point specification
- Dynamic gripper release timing
- **This is the "fun" scenario mentioned in the requirements!**

### 🚧 **Obstacle Avoidance**
Plan collision-free paths around workspace obstacles.

```bash
python main.py obstacle_avoidance
```

Features:
- RRT-style path planning
- Multiple obstacle types (spheres, boxes)
- Safety margin enforcement
- Collision detection algorithms

### 🔬 **Enhanced Dynamics**
Configuration for enhanced CoppeliaSim physics with respondable chassis.

```bash
python main.py enhanced_dynamics
```

Features:
- Respondable youBot chassis for block pushing
- Enhanced contact physics
- Realistic friction and restitution parameters
- Dynamic property configuration

### 🚀 **Run All Advanced Scenarios**
Execute all advanced features in one command:

```bash
python main.py advanced_all
```

This will create results for all advanced scenarios in the `results/advanced/` directory.
