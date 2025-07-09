import numpy as np
import os
import sys
import csv

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
    
    # Save trajectory to CSV
    save_trajectory_to_csv(traj, 'test_basic_properties_trajectory.csv')
    
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
    
    # Save trajectory to CSV
    save_trajectory_to_csv(traj, 'test_continuity_trajectory.csv')
    
    for i in range(1, traj.shape[0]):
        R0 = traj[i-1, :9].reshape(3, 3)
        R1 = traj[i, :9].reshape(3, 3)
        ang = angle_between(R0, R1)
        assert ang < np.deg2rad(5) + 1e-6


def test_dwell_length():
    poses = create_simple_poses()
    traj = TrajectoryGenerator(*poses, k=1)
    
    # Save trajectory to CSV
    save_trajectory_to_csv(traj, 'test_dwell_length_trajectory.csv')
    
    closed_idx = np.where(traj[:, -1] == CLOSED_STATE)[0]
    first_closed = closed_idx[0]
    dwell_len = np.sum(closed_idx - first_closed == np.arange(len(closed_idx)))
    assert dwell_len >= int(np.ceil(0.625 / DT_REF))
    open_idx = np.where(traj[:, -1] == OPEN_STATE)[0]
    splits = np.where(np.diff(open_idx) > 1)[0]
    last_open_start = open_idx[splits[-1] + 1] if len(splits) > 0 else open_idx[0]
    dwell_len2 = traj.shape[0] - last_open_start
    assert dwell_len2 >= int(np.ceil(0.625 / DT_REF))


def save_trajectory_to_csv(trajectory, filename):
    """Save trajectory to CSV file with appropriate headers."""
    # Create the milestone2 directory if it doesn't exist
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'milestone2')
    os.makedirs(output_dir, exist_ok=True)
    
    # Define column headers
    headers = [
        'r11', 'r12', 'r13',
        'r21', 'r22', 'r23', 
        'r31', 'r32', 'r33',
        'px', 'py', 'pz',
        'gripper_state'
    ]
    
    # Full path to the CSV file
    filepath = os.path.join(output_dir, filename)
    
    # Write to CSV file
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        writer.writerows(trajectory)
    
    print(f"Trajectory saved to: {filepath}")
    return filepath


def test_comprehensive_trajectory():
    """Test with different parameters and save a comprehensive trajectory."""
    poses = create_simple_poses()
    
    # Generate trajectory with different parameters
    traj = TrajectoryGenerator(
        *poses, 
        k=1, 
        method="quintic", 
        v_max=0.1, 
        omega_max=0.5, 
        gripper_dwell=0.625
    )
    
    # Save the comprehensive trajectory
    save_trajectory_to_csv(traj, 'comprehensive_trajectory.csv')
    
    # Verify basic properties
    assert traj.shape[1] == 13
    assert len(traj) > 0
    
    # Check that we have both open and closed states
    unique_states = np.unique(traj[:, -1])
    assert OPEN_STATE in unique_states
    assert CLOSED_STATE in unique_states
    
    print(f"Generated trajectory with {len(traj)} points")
    print(f"Trajectory duration: {len(traj) * DT_REF:.2f} seconds")
    print(f"Gripper states: {unique_states}")


def test_k_divisible():
    poses = create_simple_poses()
    k = 2
    traj = TrajectoryGenerator(*poses, k=k)
    assert traj.shape[1] == 13
    assert len(traj) % k == 0
    assert set(np.unique(traj[:, -1])).issubset({OPEN_STATE, CLOSED_STATE})


def test_traj_shape_and_flags():
    """Automated sanity check from the TrajectoryGenerator cookbook."""
    # Start the trajectory already at the final standoff so that the
    # first and last poses should match exactly.
    poses = list(create_simple_poses())
    poses[0] = poses[2] @ poses[4]
    traj = TrajectoryGenerator(*poses, k=1)

    # basic shape
    assert traj.ndim == 2 and traj.shape[1] == 13

    # gripper flag only 0/1
    assert set(np.unique(traj[:, -1])).issubset({0, 1})

    # first and last pose equality (final standoff)
    np.testing.assert_allclose(
        traj[0, :12],
        traj[-1, :12],
        atol=1e-6,
        err_msg="trajectory should end at final standoff",
    )

    # continuity guard: no big orientation jumps
    def angle(R):
        return np.arccos(0.5 * (np.trace(R.reshape(3, 3)) - 1))

    for i in range(len(traj) - 1):
        R_i = traj[i, :9].reshape(3, 3)
        R_j = traj[i + 1, :9].reshape(3, 3)
        assert angle(R_i.T @ R_j) < 0.09
