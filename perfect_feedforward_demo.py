#!/usr/bin/env python3
"""
Perfect feedforward demonstration - show the concept without relative imports
"""

import sys
import os
import numpy as np

# Add the modern_robotics environment path
sys.path.insert(0, '/home/hafnium/anaconda3/envs/modern_robotics/lib/python3.10/site-packages')

try:
    import modern_robotics as mr
    print("✓ Modern Robotics imported successfully")
except ImportError:
    print("✗ Failed to import modern_robotics")
    sys.exit(1)

# Robot parameters (from feedback_control.py)
R = 0.033  # wheel radius
L = 0.47   # chassis length 
W = 0.3    # chassis width
DT = 0.01  # time step

# youBot parameters
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
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ])
    
    # Base-to-end-effector transform
    T0e = mr.FKinBody(M0E, BLIST, theta)
    
    # Complete transformation: space -> chassis -> base -> end-effector
    Tse = Tsb @ TB0 @ T0e
    
    return Tse


def create_perfect_initial_config():
    """Create initial robot configuration for perfect feedforward tests."""
    phi_init = 0.0  # No chassis rotation
    x_init = 0.15   # Close to expected position
    y_init = 0.15   # Close to expected position
    
    # Joint angles - designed to achieve the desired end-effector pose
    theta_init = np.array([0.0, 0.0, -0.3, 0.2, 0.0])
    
    # Wheel angles
    w_init = np.zeros(4)
    
    # Combine into 12-element configuration
    config = np.hstack([phi_init, x_init, y_init, theta_init, w_init])
    return config


def create_initial_ee_pose():
    """Create initial end-effector pose according to Final Step specifications."""
    Tse_init = np.array([
        [0,  0,  1, 0],
        [0,  1,  0, 0],
        [-1, 0,  0, 0.5],
        [0,  0,  0, 1]
    ])
    return Tse_init


def main():
    print("=== Perfect Feedforward Concept Demonstration ===")
    print()
    
    # Step 1: Show the original approach (trajectory first, config second)
    print("ORIGINAL APPROACH:")
    print("1. Fixed initial end-effector pose (from specification):")
    Tse_spec = create_initial_ee_pose()
    print(f"   Position: {Tse_spec[:3, 3]}")
    print(f"   Rotation: {Tse_spec[:3, :3]}")
    
    print("\n2. 'Perfect' initial robot configuration:")
    config_perfect = create_perfect_initial_config()
    print(f"   Chassis: phi={config_perfect[0]:.3f}, x={config_perfect[1]:.3f}, y={config_perfect[2]:.3f}")
    print(f"   Joints: {config_perfect[3:8]}")
    
    print("\n3. Actual end-effector pose achieved by robot:")
    Tse_actual = compute_current_ee_pose(config_perfect)
    print(f"   Position: {Tse_actual[:3, 3]}")
    print(f"   Rotation diagonal: [{Tse_actual[0,0]:.3f}, {Tse_actual[1,1]:.3f}, {Tse_actual[2,2]:.3f}]")
    
    print("\n4. MISMATCH ANALYSIS:")
    position_error = np.linalg.norm(Tse_spec[:3, 3] - Tse_actual[:3, 3])
    rotation_error = np.linalg.norm(mr.MatrixLog3(Tse_spec[:3, :3] @ Tse_actual[:3, :3].T))
    print(f"   Position error: {position_error:.6f} m")
    print(f"   Rotation error: {rotation_error:.6f} rad") 
    print(f"   This mismatch explains why feedforward-only control has large final errors!")
    
    print("\n" + "="*60)
    print("REVISED APPROACH:")
    print("1. Set robot configuration first")
    print("2. Compute ACTUAL end-effector pose via forward kinematics")
    print("3. Generate trajectory starting from ACTUAL pose")
    print()
    print("Benefits:")
    print("   ✓ Perfect initial match (zero trajectory error at t=0)")
    print("   ✓ Feedforward control starts from correct state")
    print("   ✓ Much better final accuracy expected")
    print()
    print("This is exactly what your suggested revision accomplishes!")
    

if __name__ == "__main__":
    main()
