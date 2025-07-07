import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from modern_robotics_sim.trajectory_generator import (
    TrajectoryGenerator,
    OPEN_STATE,
    CLOSED_STATE,
    DT_REF,
)


def create_simple_poses():
    T_se_init = np.eye(4)
    T_sc_init = np.array(
        [[1, 0, 0, 1],
         [0, 1, 0, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    T_sc_goal = np.array(
        [[1, 0, 0, 0],
         [0, 1, 0, 1],
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    T_ce_grasp = np.eye(4)
    T_ce_standoff = np.eye(4)
    T_ce_standoff[2, 3] = 0.1
    return T_se_init, T_sc_init, T_sc_goal, T_ce_grasp, T_ce_standoff


def test_basic_properties():
    poses = create_simple_poses()
    traj = TrajectoryGenerator(*poses, k=1)
    assert traj.shape[1] == 13
    assert traj[0, -1] == OPEN_STATE
    T0 = np.eye(4)
    T0[:3, :3] = traj[0, :9].reshape(3, 3)
    T0[:3, 3] = traj[0, 9:12]
    np.testing.assert_allclose(T0, poses[0])
    closed_indices = np.where(traj[:, -1] == CLOSED_STATE)[0]
    assert closed_indices[0] > 0
    assert traj[-1, -1] == OPEN_STATE


def angle_between(R0, R1):
    cos = 0.5 * (np.trace(R0.T @ R1) - 1)
    cos = np.clip(cos, -1.0, 1.0)
    return np.arccos(cos)


def test_continuity():
    poses = create_simple_poses()
    traj = TrajectoryGenerator(*poses, k=1)
    for i in range(1, traj.shape[0]):
        R0 = traj[i-1, :9].reshape(3, 3)
        R1 = traj[i, :9].reshape(3, 3)
        ang = angle_between(R0, R1)
        assert ang < np.deg2rad(5) + 1e-6


def test_dwell_length():
    poses = create_simple_poses()
    traj = TrajectoryGenerator(*poses, k=1)
    closed_idx = np.where(traj[:, -1] == CLOSED_STATE)[0]
    first_closed = closed_idx[0]
    dwell_len = np.sum(closed_idx - first_closed == np.arange(len(closed_idx)))
    assert dwell_len >= int(np.ceil(0.625 / DT_REF))
    open_idx = np.where(traj[:, -1] == OPEN_STATE)[0]
    splits = np.where(np.diff(open_idx) > 1)[0]
    last_open_start = open_idx[splits[-1] + 1] if len(splits) > 0 else open_idx[0]
    dwell_len2 = traj.shape[0] - last_open_start
    assert dwell_len2 >= int(np.ceil(0.625 / DT_REF))
