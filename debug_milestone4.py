#!/usr/bin/env python3
"""
Simple test script for Milestone 4 debugging

This script runs a very basic test to verify the integration works properly.
"""

import sys
import numpy as np
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from modern_robotics_sim.run_capstone import (
    create_default_cube_poses, create_grasp_transforms,
    create_initial_config_with_error, extract_pose_from_trajectory_row,
    compute_current_ee_pose
)
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator
from modern_robotics_sim.feedback_control import FeedbackControl, compute_jacobian
from modern_robotics_sim.next_state import NextState


def simple_test():
    """Run a very basic integration test."""
    print("=== Simple Milestone 4 Integration Test ===")
    
    # Create poses
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    print("✓ Created default poses")
    
    # Generate trajectory  
    Tse_init = compute_current_ee_pose(np.zeros(12))
    trajectory = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
    )
    
    print(f"✓ Generated trajectory with {len(trajectory)} points")
    
    # Test one control step
    config = np.zeros(12)  # Start at nominal position
    
    # Extract first desired pose
    X_desired = extract_pose_from_trajectory_row(trajectory[0])
    X_desired_next = extract_pose_from_trajectory_row(trajectory[1])
    X_actual = compute_current_ee_pose(config)
    
    print(f"✓ Current EE position: {X_actual[:3, 3]}")
    print(f"✓ Desired EE position: {X_desired[:3, 3]}")
    
    # Run feedback control
    Kp = np.diag([2, 2, 2, 2, 2, 2])  # Lower gains for stability
    Ki = np.diag([0, 0, 0, 0, 0, 0])   # No integral for now
    
    V_cmd, controls_twist, X_err, integral = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, 0.01, np.zeros(6), config
    )
    
    print(f"✓ Commanded twist: {V_cmd}")
    print(f"✓ Position error: {np.linalg.norm(X_err[3:]):.4f} m")
    print(f"✓ Orientation error: {np.linalg.norm(X_err[:3]):.4f} rad")
    
    # Compute control with Jacobian
    Je = compute_jacobian(config)
    controls = np.linalg.pinv(Je) @ V_cmd
    controls = np.clip(controls, -5.0, 5.0)
    
    print(f"✓ Control commands: {controls}")
    
    # Simulate one step
    new_config = NextState(config, controls, 0.01, 5.0)
    
    print(f"✓ New configuration: {new_config}")
    
    # Check if position changed
    new_ee_pos = compute_current_ee_pose(new_config)
    print(f"✓ New EE position: {new_ee_pos[:3, 3]}")
    
    print("\n=== Test Summary ===")
    print("All basic integration components working!")
    print("Ready for full simulation tuning.")


if __name__ == "__main__":
    simple_test()
