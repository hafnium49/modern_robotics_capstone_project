import numpy as np
from modern_robotics import MatrixLog6, MatrixExp6, TransInv

# Fixed constants
DT_REF = 0.01
OPEN_STATE = 0
CLOSED_STATE = 1


def segment_time(X0, X1, v_max, omega_max):
    """Compute the duration of a move between SE(3) poses."""
    p0 = X0[:3, 3]
    p1 = X1[:3, 3]
    R0 = X0[:3, :3]
    R1 = X1[:3, :3]
    d_pos = np.linalg.norm(p1 - p0)
    cos_angle = 0.5 * (np.trace(R0.T @ R1) - 1)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    d_ang = np.arccos(cos_angle)
    T = max(d_pos / v_max, d_ang / omega_max)
    return np.ceil(T / DT_REF) * DT_REF


def _time_scaling(method, T, N):
    """Return array of s values using the specified time-scaling method."""
    t = np.linspace(0, T, N)
    tau = t / T
    if method == "quintic":
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    elif method == "cubic":
        s = 3 * tau**2 - 2 * tau**3
    elif method == "trapezoid":
        if T >= 2:
            t_b = 1.0
            if t_b > T / 2:
                t_b = T / 2
        elif T >= 1:
            t_b = T - 1
        else:
            t_b = T / 2
        a = 1.0 / (t_b * (T - t_b)) if t_b > 0 else 1.0 / T
        s = np.empty_like(t)
        for i, ti in enumerate(t):
            if ti < t_b:
                s[i] = 0.5 * a * ti**2
            elif ti <= T - t_b:
                s[i] = 0.5 * a * t_b**2 + (ti - t_b) / (T - t_b)
            else:
                s[i] = 1 - 0.5 * a * (T - ti) ** 2
    else:
        raise ValueError(f"unknown method {method}")
    return s


def _screw_interp(X0, X1, s_values):
    """Interpolate along a screw path with given s(t) samples."""
    X0_inv_X1 = TransInv(X0) @ X1
    se3mat = MatrixLog6(X0_inv_X1)
    traj = []
    for s in s_values:
        traj.append(X0 @ MatrixExp6(se3mat * s))
    return traj


def _pose_to_row(T, grip):
    return [T[0,0], T[0,1], T[0,2],
            T[1,0], T[1,1], T[1,2],
            T[2,0], T[2,1], T[2,2],
            T[0,3], T[1,3], T[2,3],
            grip]


def _append_segment(rows, T0, T1, duration, grip, method, k, skip_first):
    N = int(duration / DT_REF * k) + 1
    s_vals = _time_scaling(method, duration, N)
    poses = _screw_interp(T0, T1, s_vals)
    for i, pose in enumerate(poses):
        if skip_first and i == 0:
            continue
        rows.append(_pose_to_row(pose, grip))
    return poses[-1]


def _append_hold(rows, Tpose, duration, grip, k):
    N = int(np.ceil(duration / DT_REF * k))
    row = _pose_to_row(Tpose, grip)
    rows.extend([row] * N)
    return Tpose


def TrajectoryGenerator(T_se_init,
                        T_sc_init,
                        T_sc_goal,
                        T_ce_grasp,
                        T_ce_standoff,
                        k=1,
                        method="quintic",
                        v_max=0.1,
                        omega_max=0.5,
                        gripper_dwell=0.625):
    """Generate the 8-segment pick-and-place trajectory."""
    method = method.lower()
    T_se_init = np.asarray(T_se_init, dtype=float)
    T_sc_init = np.asarray(T_sc_init, dtype=float)
    T_sc_goal = np.asarray(T_sc_goal, dtype=float)
    T_ce_grasp = np.asarray(T_ce_grasp, dtype=float)
    T_ce_standoff = np.asarray(T_ce_standoff, dtype=float)

    traj_rows = []

    T_standoff_init = T_sc_init @ T_ce_standoff
    T_grasp_init = T_sc_init @ T_ce_grasp
    T_standoff_goal = T_sc_goal @ T_ce_standoff
    T_grasp_goal = T_sc_goal @ T_ce_grasp

    # Segment 1
    T1 = segment_time(T_se_init, T_standoff_init, v_max, omega_max)
    last = _append_segment(traj_rows, T_se_init, T_standoff_init, T1,
                           OPEN_STATE, method, k, skip_first=False)

    # Segment 2
    T2 = segment_time(T_standoff_init, T_grasp_init, v_max, omega_max)
    last = _append_segment(traj_rows, last, T_grasp_init, T2,
                           OPEN_STATE, method, k, skip_first=True)

    # Segment 3
    last = _append_hold(traj_rows, T_grasp_init, gripper_dwell, CLOSED_STATE, k)

    # Segment 4
    T4 = segment_time(T_grasp_init, T_standoff_init, v_max, omega_max)
    last = _append_segment(traj_rows, last, T_standoff_init, T4,
                           CLOSED_STATE, method, k, skip_first=True)

    # Segment 5
    T5 = segment_time(T_standoff_init, T_standoff_goal, v_max, omega_max)
    last = _append_segment(traj_rows, last, T_standoff_goal, T5,
                           CLOSED_STATE, method, k, skip_first=True)

    # Segment 6
    T6 = segment_time(T_standoff_goal, T_grasp_goal, v_max, omega_max)
    last = _append_segment(traj_rows, last, T_grasp_goal, T6,
                           CLOSED_STATE, method, k, skip_first=True)

    # Segment 7
    last = _append_hold(traj_rows, T_grasp_goal, gripper_dwell, OPEN_STATE, k)

    # Segment 8
    T8 = segment_time(T_grasp_goal, T_standoff_goal, v_max, omega_max)
    _append_segment(traj_rows, last, T_standoff_goal, T8,
                    OPEN_STATE, method, k, skip_first=True)

    return np.array(traj_rows)
