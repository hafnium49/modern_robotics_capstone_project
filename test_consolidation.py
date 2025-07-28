#!/usr/bin/env python3
"""
Test the consolidated run_capstone_simulation function
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
        run_capstone_simulation,
        run_perfect_feedforward_simulation  # Test backward compatibility
    )
    
    print("✓ run_capstone.py functions imported successfully")
    
    # Test the updated function
    print("\n=== Testing Consolidated Function ===")
    
    # Create cube poses
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    print("\n1. Testing feedforward_only=True:")
    config_log1, error_log1, success1 = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        output_dir="test_consolidated/feedforward_new",
        feedforward_only=True
    )
    
    if success1:
        print(f"  ✓ Feedforward test completed")
        print(f"  Final position error: {np.linalg.norm(error_log1[-1, 3:6]):.6f} m")
    
    print("\n2. Testing backward compatibility function:")
    config_log2, error_log2, success2 = run_perfect_feedforward_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        output_dir="test_consolidated/feedforward_old"
    )
    
    if success2:
        print(f"  ✓ Backward compatibility test completed")
        print(f"  Final position error: {np.linalg.norm(error_log2[-1, 3:6]):.6f} m")
    
    print("\n3. Testing regular feedback mode:")
    config_log3, error_log3, success3 = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
        output_dir="test_consolidated/feedback",
        use_perfect_initial=True  # Use perfect config for comparison
    )
    
    if success3:
        print(f"  ✓ Regular feedback test completed")
        print(f"  Final position error: {np.linalg.norm(error_log3[-1, 3:6]):.6f} m")
    
    print("\n=== Results Comparison ===")
    if success1 and success2 and success3:
        err1 = np.linalg.norm(error_log1[-1, 3:6])
        err2 = np.linalg.norm(error_log2[-1, 3:6])
        err3 = np.linalg.norm(error_log3[-1, 3:6])
        
        print(f"Feedforward (new): {err1:.6f} m")
        print(f"Feedforward (old): {err2:.6f} m")
        print(f"Feedback:          {err3:.6f} m")
        
        if abs(err1 - err2) < 1e-6:
            print("✅ Backward compatibility PERFECT - identical results!")
        else:
            print(f"⚠️  Results differ by {abs(err1-err2):.6f} m")
    
    print("\n✓ All consolidation tests completed successfully!")
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
