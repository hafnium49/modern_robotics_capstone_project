#!/usr/bin/env python3
"""
Check end-effector poses when gripper closes in both simulations
"""
import numpy as np
import modern_robotics as mr

# youBot transformation matrices
TB0 = np.array([
    [1, 0, 0, 0.1662],
    [0, 1, 0, 0],
    [0, 0, 1, 0.0026],
    [0, 0, 0, 1]
])

M0E = np.array([
    [1, 0, 0, 0.033],
    [0, 1, 0, 0],
    [0, 0, 1, 0.6546],
    [0, 0, 0, 1]
])

BLIST = np.array([
    [0, 0, 1, 0, 0.033, 0],
    [0, -1, 0, -0.5076, 0, 0],
    [0, -1, 0, -0.3526, 0, 0],
    [0, -1, 0, -0.2176, 0, 0],
    [0, 0, 1, 0, 0, 0]
]).T

H = 0.0963  # chassis height

def compute_current_ee_pose(config):
    """Compute current end-effector pose from robot configuration."""
    phi, x, y = config[0], config[1], config[2]
    theta = config[3:8]
    
    # Chassis-to-base transform
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    Tsb = np.array([
        [cos_phi, -sin_phi, 0, x],
        [sin_phi,  cos_phi, 0, y],
        [0,        0,       1, H],
        [0,        0,       0, 1]
    ])
    
    # Base-to-end-effector transform
    T0e = mr.FKinBody(M0E, BLIST, theta)
    
    # Complete transformation: space -> chassis -> base -> end-effector
    Tse = Tsb @ TB0 @ T0e
    
    return Tse

# Successful simulation configuration when gripper closes
config_success = np.array([0.000000,0.394365,-0.000000,0.000000,-2.435029,0.001693,0.862540,-0.000000,8.302430,8.302430,8.302430,8.302430])

# Failed simulation configuration when gripper closes  
config_failed = np.array([0.325585,0.302346,-0.053312,-0.325554,-1.566932,-0.201013,0.174752,-0.000003,8.579820,-6.711539,3.955082,-2.086800])

# Compute end-effector poses
T_success = compute_current_ee_pose(config_success)
T_failed = compute_current_ee_pose(config_failed)

print('=== Successful Simulation (Perfect Initial) ===')
print(f'Chassis: phi={config_success[0]:.3f}, x={config_success[1]:.3f}, y={config_success[2]:.3f}')
print(f'Joints: [{config_success[3]:.3f}, {config_success[4]:.3f}, {config_success[5]:.3f}, {config_success[6]:.3f}, {config_success[7]:.3f}]')
print(f'End-effector position: [{T_success[0,3]:.3f}, {T_success[1,3]:.3f}, {T_success[2,3]:.3f}]')

print('\n=== Failed Simulation (With Initial Error) ===')
print(f'Chassis: phi={config_failed[0]:.3f}, x={config_failed[1]:.3f}, y={config_failed[2]:.3f}')
print(f'Joints: [{config_failed[3]:.3f}, {config_failed[4]:.3f}, {config_failed[5]:.3f}, {config_failed[6]:.3f}, {config_failed[7]:.3f}]')
print(f'End-effector position: [{T_failed[0,3]:.3f}, {T_failed[1,3]:.3f}, {T_failed[2,3]:.3f}]')

# Cube position (should be at [1.0, 0.0, 0.025])
cube_pos = np.array([1.0, 0.0, 0.025])
success_distance = np.linalg.norm(T_success[:3,3] - cube_pos)
failed_distance = np.linalg.norm(T_failed[:3,3] - cube_pos)

print(f'\n=== Distance to Cube ===')
print(f'Cube position: {cube_pos}')
print(f'Successful sim distance to cube: {success_distance:.3f} m')
print(f'Failed sim distance to cube: {failed_distance:.3f} m')

# Check gripper orientation
print(f'\n=== Gripper Orientation ===')
print('Successful simulation gripper Z-axis (should point down):')
print(f'[{T_success[0,2]:.3f}, {T_success[1,2]:.3f}, {T_success[2,2]:.3f}]')
print('Failed simulation gripper Z-axis (should point down):')
print(f'[{T_failed[0,2]:.3f}, {T_failed[1,2]:.3f}, {T_failed[2,2]:.3f}]')

print(f'\n=== Checking Target Poses When Gripper Closes ===')

# Let's regenerate both trajectories and see what they command at pickup
import sys
sys.path.append('code')
from trajectory_generator import TrajectoryGenerator

# Default cube poses
Tsc_init = np.array([
    [1, 0, 0, 1.0],
    [0, 1, 0, 0.0], 
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]
])

Tsc_goal = np.array([
    [0, -1, 0, 0.0],
    [1, 0, 0, -1.0],
    [0, 0, 1, 0.025], 
    [0, 0, 0, 1]
])

# Grasp transforms from current run_capstone.py (after revert)
Tce_grasp = np.array([
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.0],
    [0, 0, 0, 1]
])

Tce_standoff = np.array([
    [0, 0, 1, 0],
    [0, 1, 0, 0], 
    [-1, 0, 0, 0.10],
    [0, 0, 0, 1]
])

# Generate trajectory from perfect initial pose
perfect_initial_ee = T_success
trajectory_perfect = TrajectoryGenerator(
    perfect_initial_ee, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
)

# Generate trajectory from error initial pose  
error_initial_ee = T_failed.copy()
# Adjust to reasonable initial pose
error_initial_ee[:3,3] = [0.0, 0.0, 0.5]  # Use spec initial position
trajectory_error = TrajectoryGenerator(
    error_initial_ee, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
)

# Find where gripper closes (transitions from 0 to 1)
def find_gripper_close_index(trajectory):
    for i in range(len(trajectory)-1):
        if trajectory[i,12] == 0.0 and trajectory[i+1,12] == 1.0:
            return i+1
    return -1

pickup_idx_perfect = find_gripper_close_index(trajectory_perfect)
pickup_idx_error = find_gripper_close_index(trajectory_error)

if pickup_idx_perfect >= 0:
    target_perfect = trajectory_perfect[pickup_idx_perfect]
    target_pos_perfect = target_perfect[9:12]
    print(f'Perfect trajectory target at pickup (step {pickup_idx_perfect}): [{target_pos_perfect[0]:.3f}, {target_pos_perfect[1]:.3f}, {target_pos_perfect[2]:.3f}]')
else:
    print('Could not find pickup point in perfect trajectory')

if pickup_idx_error >= 0:  
    target_error = trajectory_error[pickup_idx_error]
    target_pos_error = target_error[9:12]
    print(f'Error trajectory target at pickup (step {pickup_idx_error}): [{target_pos_error[0]:.3f}, {target_pos_error[1]:.3f}, {target_pos_error[2]:.3f}]')
else:
    print('Could not find pickup point in error trajectory')

# Check if target poses are the same
if pickup_idx_perfect >= 0 and pickup_idx_error >= 0:
    pos_diff = np.linalg.norm(target_pos_perfect - target_pos_error)
    print(f'Target position difference: {pos_diff:.6f} m')
    if pos_diff < 1e-6:
        print('✓ Target poses are identical')
    else:
        print('✗ Target poses are different!')
