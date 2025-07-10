# modern_robotics_capstone_project

This repository contains a simple kinematic simulator for the Modern Robotics
capstone project.  The main function `NextState` propagates the robot
configuration forward one time step given wheel and joint commands.

The simulator requires Python 3.10+ and the `modern_robotics` library.  
Install all dependencies with

```bash
pip install -r requirements.txt
```

The `driver` module can generate CSV files compatible with the CoppeliaSim
scenes.  Example usage for the first sanity‑check scenario is

```bash
python -m modern_robotics_sim.driver initial_config.csv output.csv \
    --controls 10 10 10 10 0 0 0 0 0 --speed-limit 20
```

The package also exposes a `TrajectoryGenerator` utility for the
pick‑and‑place task in later milestones. It returns an **N × 13** NumPy
array that can be written directly to a CSV file for Scene 6.

Unit tests verify both `NextState` and the trajectory generator.  Run:

```bash
pytest -q
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
- **Initial cube pose**: `(1, 0, 0)` (x=1m, y=0m, θ=0°)
- **Goal cube pose**: `(0, -1, -π/2)` (x=0m, y=-1m, θ=-90°)

For a **custom cube configuration**, you can edit the values in `driver.py` or create your own test script. For example:

```python
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator

# Custom configuration
cube_initial = (0.5, 0.5, 0)      # x=0.5m, y=0.5m, θ=0°
cube_goal = (-0.5, 0.5, np.pi/4)  # x=-0.5m, y=0.5m, θ=45°

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
- Your trajectory uses `(x, y, θ)` where θ is rotation about Z-axis
- CoppeliaSim uses `(X, Y, Z)` position and `(α, β, γ)` Euler angles
- Set **Z = 0.025** (cube rests on ground) and **γ = θ** (Z-rotation)

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
F = (H^{(0)})^{\#}      \qquad\text{(3 × 4 → gives planar twist from wheel rates)}
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
