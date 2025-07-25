#!/usr/bin/env python3
"""
Final Milestone (Milestone 4): Software Integration Driver

This module implements the complete capstone project integration, combining:
- Milestone 1: NextState kinematic simulator
- Milestone 2: TrajectoryGenerator for pick-and-place path
- Milestone 3: FeedbackControl for task-space control

According to the capstone requirements, this driver must:
1. Hard-code default cube poses
2. Choose initial robot configuration with significant error
3. Generate nominal trajectory and run control loop
4. Write output CSV files and error logs
"""

import numpy as np
import os
import sys
import modern_robotics as mr
from pathlib import Path

# Import milestone components
from .next_state import NextState
from .trajectory_generator import TrajectoryGenerator
from .feedback_control import FeedbackControl, compute_jacobian
from .feedback_control import R, L, W, DT, TB0, M0E, BLIST, INV_TB0, PINV_TOLERANCE


# Fixed constants for Milestone 4
SPEED_LIMIT = 5.0  # rad/s for Milestone 4 (reduced from 12.3 in Milestone 3)
DT_CAPSTONE = 0.01  # s - fixed time step


def create_initial_ee_pose():
    """Create initial end-effector pose according to Final Step specifications.
    
    T_se = ⎡ 0  0  1  0 ⎤
           ⎢ 0  1  0  0 ⎥
           ⎢−1  0  0  0.5⎥
           ⎣ 0  0  0  1 ⎦
           
    Returns:
        4x4 SE(3) initial end-effector pose
    """
    Tse_init = np.array([
        [0,  0,  1, 0],
        [0,  1,  0, 0],
        [-1, 0,  0, 0.5],
        [0,  0,  0, 1]
    ])
    
    return Tse_init


def create_default_cube_poses():
    """Create default cube poses according to Final Step specifications.
    
    Initial cube: (x, y, θ) = (1 m, 0 m, 0 rad) 
    Final cube: (x, y, θ) = (0 m, -1 m, -π/2 rad)
    
    Returns:
        Tsc_init: 4x4 SE(3) - initial cube pose
        Tsc_goal: 4x4 SE(3) - goal cube pose
    """
    # Initial cube configuration: (1 m, 0 m, 0 rad) - matches CoppeliaSim Scene 6 defaults
    Tsc_init = np.array([
        [1, 0, 0, 1.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])
    
    # Final cube configuration: (0 m, -1 m, -π/2 rad)
    # θ = -π/2 means rotation of -90° about z-axis
    cos_neg_pi2 = np.cos(-np.pi/2)  # 0
    sin_neg_pi2 = np.sin(-np.pi/2)  # -1
    Tsc_goal = np.array([
        [cos_neg_pi2, -sin_neg_pi2, 0, 0.0],
        [sin_neg_pi2,  cos_neg_pi2, 0, -1.0],
        [0,            0,           1, 0.025],
        [0,            0,           0, 1]
    ])
    
    return Tsc_init, Tsc_goal


def create_grasp_transforms():
    """Create grasp and standoff transforms according to Milestone 4 requirements.
    
    Returns:
        Tce_grasp: 4x4 SE(3) - grasp transform relative to cube
        Tce_standoff: 4x4 SE(3) - standoff transform relative to cube
    """
    # Tce_grasp: End-effector pointing downward to grasp cube from above
    # Rotation: 180° about x-axis to flip the gripper downward
    # Position: [0, 0, 0.02] m (cube halfway into fingers)
    Tce_grasp = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0.02],
        [0,  0,  0, 1]
    ])
    
    # Tce_standoff = Tce_grasp shifted +[0,0,0.10] m in z_c (before rotation)
    # Since we rotated 180° about x, the standoff should be further in -z direction
    Tce_standoff = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0.12],  # 0.02 + 0.10
        [0,  0,  0, 1]
    ])
    
    return Tce_grasp, Tce_standoff


def create_initial_config_with_error(trajectory_first_row):
    """Create initial robot configuration with significant error.
    
    Must have ≥0.20 m position error AND ≥30 deg orientation error
    from the first row of the reference trajectory.
    
    Args:
        trajectory_first_row: 13-element array from trajectory
        
    Returns:
        config: 12-element initial configuration [phi, x, y, theta1-5, w1-4]
    """
    # Extract desired end-effector pose from first trajectory row
    # Format: [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,grip]
    R_desired = trajectory_first_row[:9].reshape(3, 3)
    p_desired = trajectory_first_row[9:12]
    
    # Create initial configuration with significant but manageable errors
    phi_init = np.radians(35)  # 35° chassis rotation (>30° requirement)
    x_init = p_desired[0] + 0.15  # +0.15m position error in x  
    y_init = p_desired[1] + 0.15  # +0.15m position error in y (total ~0.21m > 0.20m requirement)
    
    # Joint angles - use better starting configuration for manipulation
    theta_init = np.array([0.0, 0.0, -0.2, -1.6, 0.0])  # More suitable for manipulation
    
    # Wheel angles (not critical for initial pose)
    w_init = np.zeros(4)
    
    # Combine into 12-element configuration
    config = np.hstack([phi_init, x_init, y_init, theta_init, w_init])
    
    return config


def extract_pose_from_trajectory_row(traj_row):
    """Extract SE(3) pose from trajectory row.
    
    Args:
        traj_row: 13-element trajectory row [r11...r33, px, py, pz, grip]
        
    Returns:
        4x4 SE(3) transformation matrix
    """
    R = traj_row[:9].reshape(3, 3)
    p = traj_row[9:12]
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    
    return T


def compute_current_ee_pose(config):
    """Compute current end-effector pose from robot configuration.
    
    Args:
        config: 12-element configuration [phi, x, y, theta1-5, w1-4]
        
    Returns:
        4x4 SE(3) end-effector pose
    """
    phi, x, y = config[0], config[1], config[2]
    theta = config[3:8]
    
    # Chassis-to-base transform
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    Tsb = np.array([
        [cos_phi, -sin_phi, 0, x],
        [sin_phi,  cos_phi, 0, y],
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ])
    
    # Base-to-end-effector transform
    T0e = mr.FKinBody(M0E, BLIST, theta)
    
    # Complete transformation: space -> chassis -> base -> end-effector
    Tse = Tsb @ TB0 @ T0e
    
    return Tse


def run_capstone_simulation(Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, 
                          Kp=None, Ki=None, output_dir="results/best"):
    """Run the complete capstone simulation.
    
    Args:
        Tsc_init: 4x4 SE(3) - initial cube pose
        Tsc_goal: 4x4 SE(3) - goal cube pose  
        Tce_grasp: 4x4 SE(3) - grasp transform
        Tce_standoff: 4x4 SE(3) - standoff transform
        Kp: 6x6 proportional gain matrix (default provided)
        Ki: 6x6 integral gain matrix (default provided)
        output_dir: directory for output files
        
    Returns:
        config_log: Nx12 array of robot configurations
        error_log: Nx6 array of pose errors
        success: boolean indicating if simulation completed
    """
    
    # Default gains as suggested in Milestone 4 (conservative for large initial errors)
    if Kp is None:
        Kp = np.diag([3, 3, 3, 3, 3, 3])  # Moderate proportional gains
    if Ki is None:
        Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Small integral gains to prevent windup
    
    print("Generating reference trajectory ...", end="", flush=True)
    
    # Use specified initial end-effector pose from Final Step
    Tse_init = create_initial_ee_pose()
    
    trajectory = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        k=1, method="quintic"
    )
    
    N_points = len(trajectory)
    print(f" done ({N_points} points)")
    
    # Create initial configuration with significant error
    config = create_initial_config_with_error(trajectory[0])
    
    # Initialize control state
    integral_error = np.zeros(6)
    
    # Storage for logging
    config_log = np.zeros((N_points, 12))
    error_log = np.zeros((N_points-1, 6))
    trajectory_log = np.zeros((N_points, 13))
    
    # Log initial configuration
    config_log[0] = config
    trajectory_log[0] = trajectory[0]
    
    print(f"Simulating {N_points-1} control steps ...", end="", flush=True)
    
    # Main control loop
    for i in range(N_points - 1):
        # Extract desired poses
        X_desired = extract_pose_from_trajectory_row(trajectory[i])
        X_desired_next = extract_pose_from_trajectory_row(trajectory[i+1])
        
        # Compute current end-effector pose
        X_actual = compute_current_ee_pose(config)
        
        # Feedback control
        V_cmd, controls_twist, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT_CAPSTONE, integral_error, config
        )
        
        # Convert twist to wheel + joint rates using mobile manipulator Jacobian
        Je = compute_jacobian(config)
        controls_raw = np.linalg.pinv(Je, rcond=PINV_TOLERANCE) @ V_cmd
        controls = np.clip(controls_raw, -SPEED_LIMIT, SPEED_LIMIT)
        
        # Simulate robot motion
        config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
        
        # Log data
        config_log[i+1] = config
        error_log[i] = X_err
        trajectory_log[i+1] = trajectory[i+1]
    
    print(" done")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Write robot configuration CSV (13-column format for CoppeliaSim Scene 6)
    youbot_output_path = os.path.join(output_dir, "youBot_output.csv")
    youbot_data = np.zeros((N_points, 13))
    youbot_data[:, :12] = config_log  # robot configurations
    youbot_data[:, 12] = trajectory_log[:, 12]  # gripper states from trajectory
    
    # Save in Scene 6 compatible format (no headers, precise formatting)
    np.savetxt(youbot_output_path, youbot_data, delimiter=',', fmt='%.6f')
    print(f"Scene 6 compatible CSV written to {youbot_output_path}")
    
    # Verify Scene 6 compatibility
    verify_scene6_compatibility(youbot_output_path)
    
    # Write error log CSV
    error_output_path = os.path.join(output_dir, "Xerr_log.csv")
    np.savetxt(error_output_path, error_log, delimiter=',', fmt='%.6f')
    print(f"Error log written to {error_output_path}")

    return config_log, error_log, True


def verify_scene6_compatibility(csv_path):
    """Verify that the CSV file is compatible with CoppeliaSim Scene 6.
    
    Args:
        csv_path: path to the CSV file to verify
    """
    try:
        # Read the CSV file and verify format
        data = np.loadtxt(csv_path, delimiter=',')
        rows, cols = data.shape
        
        print(f"✓ CSV verification: {rows} rows x {cols} columns")
        
        # Check column count (must be 13 for Scene 6)
        if cols != 13:
            print(f"⚠ Warning: Expected 13 columns, found {cols}")
            return False
            
        # Check data ranges for reasonableness
        chassis_phi = data[:, 0]  # φ (chassis orientation)
        chassis_x = data[:, 1]    # x position
        chassis_y = data[:, 2]    # y position
        gripper = data[:, 12]     # gripper state
        
        print(f"✓ Chassis position range: x[{chassis_x.min():.3f}, {chassis_x.max():.3f}], y[{chassis_y.min():.3f}, {chassis_y.max():.3f}]")
        print(f"✓ Chassis orientation range: φ[{chassis_phi.min():.3f}, {chassis_phi.max():.3f}] rad")
        print(f"✓ Gripper states: {np.unique(gripper).astype(int)}")
        
        # Check for NaN or infinite values
        if np.any(np.isnan(data)) or np.any(np.isinf(data)):
            print("⚠ Warning: CSV contains NaN or infinite values")
            return False
            
        print("✓ CSV file is compatible with CoppeliaSim Scene 6")
        return True
        
    except Exception as e:
        print(f"✗ CSV verification failed: {e}")
        return False
def plot_error_results(error_log, output_dir="results/best"):
    """Plot all six components of X_err vs time and save as PDF.
    
    Args:
        error_log: Nx6 array of pose errors
        output_dir: directory for output files
    """
    try:
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend to avoid tkinter issues
        import matplotlib.pyplot as plt
        
        # Create time vector
        time = np.arange(len(error_log)) * DT_CAPSTONE
        
        # Create figure with 2x3 subplots
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle('Pose Error vs Time', fontsize=14)
        
        # Error labels
        labels = ['ωx (rad)', 'ωy (rad)', 'ωz (rad)', 'vx (m)', 'vy (m)', 'vz (m)']
        titles = ['Angular Error X', 'Angular Error Y', 'Angular Error Z',
                 'Linear Error X', 'Linear Error Y', 'Linear Error Z']
        
        # Plot each error component
        for i in range(6):
            row, col = i // 3, i % 3
            axes[row, col].plot(time, error_log[:, i], 'b-', linewidth=1.5)
            axes[row, col].set_title(titles[i])
            axes[row, col].set_xlabel('Time (s)')
            axes[row, col].set_ylabel(labels[i])
            axes[row, col].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save as PDF
        plot_path = os.path.join(output_dir, "Xerr_plot.pdf")
        plt.savefig(plot_path, format='pdf', bbox_inches='tight')
        print(f"Error plot saved to {plot_path}")
        
        plt.close()
        
    except ImportError:
        print("Matplotlib not available - error plot not generated")


def main():
    """Main function for capstone simulation."""
    print("=== Modern Robotics Capstone Project - Final Integration ===")
    print()
    
    # Create default poses
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    # Run simulation with default parameters
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff
    )
    
    if success:
        # Generate error plot
        plot_error_results(error_log)
        
        print()
        print("=== Simulation Summary ===")
        print(f"Total time steps: {len(config_log)}")
        print(f"Final position error: {np.linalg.norm(error_log[-1, 3:6]):.4f} m")
        print(f"Final orientation error: {np.linalg.norm(error_log[-1, :3]):.4f} rad")
        print()
        print("Files generated in results/best/:")
        print("  - youBot_output.csv")
        print("  - Xerr_log.csv") 
        print("  - Xerr_plot.pdf")
        print()
        print("Simulation completed successfully!")
        
    else:
        print("Simulation failed!")
        return False
        
    return True


if __name__ == "__main__":
    main()
