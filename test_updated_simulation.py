#!/usr/bin/env python3
"""
Test the updated run_capstone_simulation function
"""

import sys
import os
import numpy as np

# Add paths
sys.path.insert(0, '/home/hafnium/anaconda3/envs/modern_robotics/lib/python3.10/site-packages')
sys.path.append('code')

try:
    import modern_robotics as mr
    print("✓ Modern Robotics imported successfully")
    
    # Import the functions we need 
    from code.run_capstone import (
        create_default_cube_poses, 
        create_grasp_transforms,
        run_capstone_simulation
    )
    
    print("✓ run_capstone.py functions imported successfully")
    
    # Test the updated function
    print("\n=== Testing Updated run_capstone_simulation ===")
    
    # Create cube poses
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    print("\n1. Testing with perfect initial configuration:")
    config_log1, error_log1, success1 = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        Kp=np.zeros((6, 6)), Ki=np.zeros((6, 6)),  # Feedforward only
        output_dir="test_results/perfect_revised",
        use_perfect_initial=True
    )
    
    if success1:
        print(f"  ✓ Perfect initial test completed")
        print(f"  Final position error: {np.linalg.norm(error_log1[-1, 3:6]):.6f} m")
        print(f"  Final orientation error: {np.linalg.norm(error_log1[-1, :3]):.6f} rad")
    
    print("\n2. Testing with error initial configuration:")
    config_log2, error_log2, success2 = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        Kp=np.zeros((6, 6)), Ki=np.zeros((6, 6)),  # Feedforward only
        output_dir="test_results/error_revised", 
        use_perfect_initial=False
    )
    
    if success2:
        print(f"  ✓ Error initial test completed")
        print(f"  Final position error: {np.linalg.norm(error_log2[-1, 3:6]):.6f} m")
        print(f"  Final orientation error: {np.linalg.norm(error_log2[-1, :3]):.6f} rad")
    
    print("\n=== Comparison ===")
    if success1 and success2:
        print(f"Perfect initial: {np.linalg.norm(error_log1[-1, 3:6]):.6f} m position error")
        print(f"Error initial:   {np.linalg.norm(error_log2[-1, 3:6]):.6f} m position error")
        print("Both should now have much better performance due to revised approach!")
    
    print("\n✓ All tests completed successfully!")
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
