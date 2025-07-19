import numpy as np
import os
import sys
import csv

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from code.trajectory_generator import (
    TrajectoryGenerator,
    OPEN_STATE,
    CLOSED_STATE,
    DT_REF,
)


def extract_pose_2d(T):
    """Extract (x, y, θ) from a 4x4 SE(3) transformation matrix.
    
    Args:
        T: 4x4 numpy array representing SE(3) transformation
        
    Returns:
        tuple: (x, y, theta) where theta is in radians
    """
    x = T[0, 3]
    y = T[1, 3]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return x, y, theta


def print_cube_configurations(T_sc_init, T_sc_goal, test_name=""):
    """Print cube configurations in (x, y, θ) format."""
    x_init, y_init, theta_init = extract_pose_2d(T_sc_init)
    x_goal, y_goal, theta_goal = extract_pose_2d(T_sc_goal)
    
    if test_name:
        print(f"\n{test_name} - Cube configurations:")
    else:
        print(f"\nCube configurations:")
    print(f"  Initial: (x={x_init:.3f}, y={y_init:.3f}, θ={theta_init:.3f} rad)")
    print(f"  Goal:    (x={x_goal:.3f}, y={y_goal:.3f}, θ={theta_goal:.3f} rad)")


def create_simple_poses():
    """Create poses for basic testing."""
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
    
    # Print configurations for simple test
    print_cube_configurations(T_sc_init, T_sc_goal, "Simple Test")
    
    return T_se_init, T_sc_init, T_sc_goal, T_ce_grasp, T_ce_standoff


def create_scene8_poses():
    """Create poses for CoppeliaSim Scene 8 testing with default configurations.
    
    Initial cube: (x=1, y=0, θ=0)
    Goal cube: (x=0, y=-1, θ=-π/2)
    """
    # End-effector initial pose (arbitrary start position)
    T_se_init = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0], 
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ])
    
    # Initial cube pose: (1, 0, 0) with no rotation
    T_sc_init = np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0.025],  # cube height/2
        [0, 0, 0, 1]
    ])
    
    # Goal cube pose: (0, -1, -π/2) 
    T_sc_goal = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, -1],
        [0, 0, 1, 0.025],  # cube height/2
        [0, 0, 0, 1]
    ])
    
    # Print configurations for Scene 8 default
    print_cube_configurations(T_sc_init, T_sc_goal, "Scene 8 Default")
    
    # Grasp pose relative to cube (approach from above)
    T_ce_grasp = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0],  # directly above cube center
        [0, 0, 0, 1]
    ])
    
    # Standoff pose relative to cube (10cm above grasp)
    T_ce_standoff = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0.1],  # 10cm above grasp pose
        [0, 0, 0, 1]
    ])
    
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


def test_scene8_default_configuration():
    """Test Step 1: Generate eight_segment_traj.csv for Scene 8 with default cube configurations.
    
    Default configurations:
    - Initial cube: (x=1, y=0, θ=0) 
    - Goal cube: (x=0, y=-1, θ=-π/2)
    """
    poses = create_scene8_poses()
    T_se_init, T_sc_init, T_sc_goal, T_ce_grasp, T_ce_standoff = poses
    
    # Generate trajectory with k=1 (standard timing)
    traj = TrajectoryGenerator(T_se_init,
                              T_sc_init, 
                              T_sc_goal,
                              T_ce_grasp,
                              T_ce_standoff,
                              k=1)
    
    # Save as eight_segment_traj.csv for Scene 8
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'milestone2')
    os.makedirs(output_dir, exist_ok=True)
    filepath = os.path.join(output_dir, 'eight_segment_traj.csv')
    
    # Save without headers for Scene 8 compatibility
    np.savetxt(filepath, traj, delimiter=",")
    
    print(f"Scene 8 trajectory saved to: {filepath}")
    print(f"Total rows: {len(traj)}")
    print(f"Trajectory duration: {len(traj) * DT_REF:.2f} seconds")
    
    # Print cube configurations using the helper function
    print_cube_configurations(T_sc_init, T_sc_goal, "Scene 8 Default")
    
    # Verify basic properties
    assert traj.shape[1] == 13
    assert len(traj) > 0
    assert traj[0, -1] == OPEN_STATE  # starts with gripper open
    assert traj[-1, -1] == OPEN_STATE  # ends with gripper open
    
    # Verify we have both gripper states
    unique_states = np.unique(traj[:, -1])
    assert OPEN_STATE in unique_states
    assert CLOSED_STATE in unique_states


def test_scene8_custom_configuration():
    """Test Step 3: Generate trajectory with custom cube configurations.
    
    Custom configurations:
    - Initial cube: (x=0.5, y=0.5, θ=π/4)
    - Goal cube: (x=-0.5, y=-0.5, θ=-π/4)
    """
    T_se_init, _, _, T_ce_grasp, T_ce_standoff = create_scene8_poses()
    
    # Custom initial cube pose: (0.5, 0.5, π/4)
    cos_pi4 = np.cos(np.pi/4)
    sin_pi4 = np.sin(np.pi/4)
    T_sc_init = np.array([
        [cos_pi4, -sin_pi4, 0, 0.5],
        [sin_pi4,  cos_pi4, 0, 0.5],
        [0,        0,       1, 0.025],
        [0,        0,       0, 1]
    ])
    
    # Custom goal cube pose: (-0.5, -0.5, -π/4)  
    cos_neg_pi4 = np.cos(-np.pi/4)
    sin_neg_pi4 = np.sin(-np.pi/4)
    T_sc_goal = np.array([
        [cos_neg_pi4, -sin_neg_pi4, 0, -0.5],
        [sin_neg_pi4,  cos_neg_pi4, 0, -0.5],
        [0,            0,           1, 0.025],
        [0,            0,           0, 1]
    ])
    
    # Generate trajectory 
    traj = TrajectoryGenerator(T_se_init,
                              T_sc_init,
                              T_sc_goal, 
                              T_ce_grasp,
                              T_ce_standoff,
                              k=1)
    
    # Save custom configuration trajectory
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'milestone2')
    os.makedirs(output_dir, exist_ok=True)
    filepath = os.path.join(output_dir, 'eight_segment_traj_custom.csv')
    
    np.savetxt(filepath, traj, delimiter=",")
    
    print(f"Custom Scene 8 trajectory saved to: {filepath}")
    print(f"Total rows: {len(traj)}")
    
    # Print cube configurations using the helper function
    print_cube_configurations(T_sc_init, T_sc_goal, "Scene 8 Custom")
    
    # Verify trajectory properties
    assert traj.shape[1] == 13
    assert len(traj) > 0
    assert traj[0, -1] == OPEN_STATE
    assert traj[-1, -1] == OPEN_STATE


def test_scene8_timing_verification():
    """Verify timing requirements for Scene 8 compatibility."""
    poses = create_scene8_poses()
    traj = TrajectoryGenerator(*poses, k=1, gripper_dwell=0.625)
    
    # Check gripper dwell times (segments 3 and 7)
    closed_indices = np.where(traj[:, -1] == CLOSED_STATE)[0]
    open_indices = np.where(traj[:, -1] == OPEN_STATE)[0]
    
    min_dwell_rows = int(np.ceil(0.625 / DT_REF))  # >= 63 rows
    
    if len(closed_indices) > 0:
        # Find continuous segments of closed gripper
        closed_segments = []
        current_segment = [closed_indices[0]]
        
        for i in range(1, len(closed_indices)):
            if closed_indices[i] == closed_indices[i-1] + 1:
                current_segment.append(closed_indices[i])
            else:
                closed_segments.append(current_segment)
                current_segment = [closed_indices[i]]
        closed_segments.append(current_segment)
        
        print(f"Found {len(closed_segments)} closed gripper segments")
        for i, segment in enumerate(closed_segments):
            segment_length = len(segment)
            print(f"  Segment {i+1}: {segment_length} rows (min required: {min_dwell_rows})")
            assert segment_length >= min_dwell_rows, f"Dwell segment {i+1} too short: {segment_length} < {min_dwell_rows}"
    
    # Check row count divisibility by k
    assert len(traj) % 1 == 0  # k=1, so should be divisible by 1
    
    # Verify first and last poses
    assert traj[0, -1] == OPEN_STATE, "First pose should have gripper open"
    assert traj[-1, -1] == OPEN_STATE, "Last pose should have gripper open"
    
    print(f"Timing verification passed. Total duration: {len(traj) * DT_REF:.2f} seconds")
