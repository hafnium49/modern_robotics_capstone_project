#!/usr/bin/env python3
"""
Utility script to extract (x, y, θ) configurations from SE(3) transformation matrices
in the Milestone 2 tests.
"""

import numpy as np
import sys
import os

# Add the project root to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from tests.test_milestone2 import create_scene8_poses, create_simple_poses


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


def print_pose_info(T, name):
    """Print pose information in a readable format."""
    x, y, theta = extract_pose_2d(T)
    print(f"{name}:")
    print(f"  Position: x={x:.3f}, y={y:.3f}")
    print(f"  Orientation: θ={theta:.3f} rad ({np.degrees(theta):.1f}°)")
    print(f"  SE(3) matrix:")
    print(f"    {T}")
    print()


def main():
    print("=== Milestone 2 Cube Configurations ===")
    print()
    
    # Scene 8 default configurations
    print("Scene 8 Default Configuration:")
    print("-" * 40)
    T_se_init, T_sc_init, T_sc_goal, T_ce_grasp, T_ce_standoff = create_scene8_poses()
    
    print_pose_info(T_sc_init, "Initial Cube Pose")
    print_pose_info(T_sc_goal, "Goal Cube Pose")
    
    # Simple test configurations  
    print("Simple Test Configuration:")
    print("-" * 40)
    T_se_init_simple, T_sc_init_simple, T_sc_goal_simple, T_ce_grasp_simple, T_ce_standoff_simple = create_simple_poses()
    
    print_pose_info(T_sc_init_simple, "Initial Cube Pose (Simple)")
    print_pose_info(T_sc_goal_simple, "Goal Cube Pose (Simple)")
    
    # Custom Scene 8 configuration (as defined in test)
    print("Scene 8 Custom Configuration:")
    print("-" * 40)
    
    # Custom initial cube pose: (0.5, 0.5, π/4)
    cos_pi4 = np.cos(np.pi/4)
    sin_pi4 = np.sin(np.pi/4)
    T_sc_init_custom = np.array([
        [cos_pi4, -sin_pi4, 0, 0.5],
        [sin_pi4,  cos_pi4, 0, 0.5],
        [0,        0,       1, 0.025],
        [0,        0,       0, 1]
    ])
    
    # Custom goal cube pose: (-0.5, -0.5, -π/4)  
    cos_neg_pi4 = np.cos(-np.pi/4)
    sin_neg_pi4 = np.sin(-np.pi/4)
    T_sc_goal_custom = np.array([
        [cos_neg_pi4, -sin_neg_pi4, 0, -0.5],
        [sin_neg_pi4,  cos_neg_pi4, 0, -0.5],
        [0,            0,           1, 0.025],
        [0,            0,           0, 1]
    ])
    
    print_pose_info(T_sc_init_custom, "Initial Cube Pose (Custom)")
    print_pose_info(T_sc_goal_custom, "Goal Cube Pose (Custom)")
    
    print("=== Summary ===")
    print("Default Scene 8:  Initial (1.0, 0.0, 0.0°) → Goal (0.0, -1.0, -90.0°)")
    print("Simple Test:      Initial (1.0, 0.0, 0.0°) → Goal (0.0, 1.0, 0.0°)")  
    print("Custom Scene 8:   Initial (0.5, 0.5, 45.0°) → Goal (-0.5, -0.5, -45.0°)")


if __name__ == "__main__":
    main()
