#!/usr/bin/env python3
"""
Simple test script for perfect feedforward simulation
"""

import sys
import os
sys.path.append('code')

try:
    print("Starting perfect feedforward test...")
    
    import numpy as np
    
    # Import needed functions
    from run_capstone import (
        create_default_cube_poses, create_grasp_transforms, 
        run_perfect_feedforward_simulation
    )
    
    print("Imports successful")
    
    # Get default poses
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    print("Poses created")
    
    # Run perfect feedforward simulation
    output_dir = "milestone3_feedforward_tests/perfect_initial_test"
    config_log, error_log, success = run_perfect_feedforward_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        output_dir=output_dir
    )
    
    print(f"Generated trajectory: {len(config_log)} timesteps")
    print(f"Final position error: {np.linalg.norm(error_log[-1, 3:6]):.6f} m")  
    print(f"Final orientation error: {np.linalg.norm(error_log[-1, :3]):.6f} rad")
    print("SUCCESS: Perfect feedforward test completed!")
    
    # Compare with previous result
    print(f"\nComparison with previous 'perfect' test:")
    print(f"Previous result: 0.276430 m position, 1.384748 rad orientation")
    print(f"New result:      {np.linalg.norm(error_log[-1, 3:6]):.6f} m position, {np.linalg.norm(error_log[-1, :3]):.6f} rad orientation")
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
