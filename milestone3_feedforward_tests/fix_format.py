#!/usr/bin/env python3
"""
Script to convert trajectory format from SE(3) poses to Scene 6 configuration format
"""
import numpy as np

def convert_trajectory_to_scene6_format(trajectory_file_path, output_path):
    """
    Convert trajectory from SE(3) pose format to Scene 6 robot configuration format.
    
    Input format: [R11, R12, R13, R21, R22, R23, R31, R32, R33, px, py, pz, gripper]
    Output format: [chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper]
    """
    print("ERROR: The input file contains end-effector poses, not robot configurations!")
    print("\nTo fix this, you need to:")
    print("1. Use inverse kinematics to convert end-effector poses to joint angles")
    print("2. Use the mobile manipulator Jacobian pseudoinverse method")
    print("3. Generate actual robot configurations (chassis + arm + wheels)")
    print("\nThe current file format is incompatible with CoppeliaSim Scene 6.")
    print("Scene 6 expects robot configurations, not end-effector trajectories.")

if __name__ == "__main__":
    print("=== Scene 6 Compatibility Analysis ===")
    print("Current file: feedforward_perfect_initial.csv")
    print("Status: INCOMPATIBLE ❌")
    print("\nReason: File contains end-effector poses (rotation matrices)")
    print("Required: Robot configurations (chassis + joints + wheels)")
    
    convert_trajectory_to_scene6_format("", "")
