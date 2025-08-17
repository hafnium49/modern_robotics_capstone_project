#!/usr/bin/env python3
"""
Check if higher gains improved tracking performance
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

# Configuration when gripper closes with higher gains
config_higher_gains = np.array([0.325585,0.302346,-0.053312,-0.325554,-1.566932,-0.201013,0.174752,-0.000003,8.579820,-6.711539,3.955082,-2.086800])

# Compute end-effector pose
T_higher_gains = compute_current_ee_pose(config_higher_gains)

print('=== Higher Gains Simulation Results ===')
print(f'Chassis: phi={config_higher_gains[0]:.3f}, x={config_higher_gains[1]:.3f}, y={config_higher_gains[2]:.3f}')
print(f'Joints: [{config_higher_gains[3]:.3f}, {config_higher_gains[4]:.3f}, {config_higher_gains[5]:.3f}, {config_higher_gains[6]:.3f}, {config_higher_gains[7]:.3f}]')
print(f'End-effector position: [{T_higher_gains[0,3]:.3f}, {T_higher_gains[1,3]:.3f}, {T_higher_gains[2,3]:.3f}]')

# Cube position (should be at [1.0, 0.0, 0.025])
cube_pos = np.array([1.0, 0.0, 0.025])
distance_to_cube = np.linalg.norm(T_higher_gains[:3,3] - cube_pos)

print(f'\n=== Cube Pickup Analysis ===')
print(f'Cube position: {cube_pos}')
print(f'Distance to cube: {distance_to_cube:.3f} m')
print(f'Gripper orientation Z-axis: [{T_higher_gains[0,2]:.3f}, {T_higher_gains[1,2]:.3f}, {T_higher_gains[2,2]:.3f}]')

if distance_to_cube < 0.05:  # Within 5cm
    print('✅ SUCCESS: Gripper is close enough to pick up the cube!')
else:
    print('❌ FAILED: Gripper is still too far from the cube')
    print(f'   Height error: {T_higher_gains[2,3] - cube_pos[2]:.3f} m')
    print(f'   Horizontal error: {np.linalg.norm(T_higher_gains[:2,3] - cube_pos[:2]):.3f} m')

# Compare with original results
print(f'\n=== Comparison with Original Lower Gains ===')
print(f'Original distance to cube: 0.190 m (19cm too high)')
print(f'New distance to cube: {distance_to_cube:.3f} m')
improvement = 0.190 - distance_to_cube
print(f'Improvement: {improvement:.3f} m ({improvement*100:.0f}cm closer)')

if improvement > 0:
    print('✅ Higher gains DID improve tracking performance!')
else:
    print('❌ Higher gains did NOT improve performance')
