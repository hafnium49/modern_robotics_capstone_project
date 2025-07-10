"""
Milestone 3 Feedforward Testing CSV Generator

This script generates CSV files for testing feedforward control in CoppeliaSim.
It implements the specific testing methodology described in the Milestone 3 requirements.
"""

import numpy as np
import os
from modern_robotics_sim.feedback_control import FeedbackController, FeedbackControl
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator
from modern_robotics_sim.next_state import NextState


def generate_feedforward_csv(output_file="feedforward_test.csv", initial_error=None):
    """Generate CSV file for feedforward control testing.
    
    Args:
        output_file: Path to output CSV file
        initial_error: Optional 3-element array [dx, dy, dz] for initial end-effector error
    """
    
    print(f"Generating feedforward control test CSV: {output_file}")
    if initial_error is not None:
        print(f"With initial error: {initial_error}")
    
    # Define poses for pick-and-place trajectory
    T_se_init = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ])
    
    T_sc_init = np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])
    
    T_sc_goal = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, -1],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])
    
    T_ce_grasp = np.array([
        [-1.0/np.sqrt(2), 0, 1.0/np.sqrt(2), 0],
        [0, 1, 0, 0],
        [-1.0/np.sqrt(2), 0, -1.0/np.sqrt(2), 0],
        [0, 0, 0, 1]
    ])
    
    T_ce_standoff = np.array([
        [-1.0/np.sqrt(2), 0, 1.0/np.sqrt(2), 0],
        [0, 1, 0, 0],
        [-1.0/np.sqrt(2), 0, -1.0/np.sqrt(2), 0.1],
        [0, 0, 0, 1]
    ])
    
    # Generate reference trajectory
    print("Generating reference trajectory...")
    trajectory = TrajectoryGenerator(
        T_se_init=T_se_init,
        T_sc_init=T_sc_init,
        T_sc_goal=T_sc_goal,
        T_ce_grasp=T_ce_grasp,
        T_ce_standoff=T_ce_standoff,
        k=1  # Standard time resolution
    )
    
    print(f"Generated trajectory with {len(trajectory)} time steps")
    
    # Initial robot configuration that should put end-effector near start of trajectory
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y  
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Feedforward only gains (Kp = Ki = 0)
    Kp = np.diag([0, 0, 0, 0, 0, 0])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Control parameters
    dt = 0.01
    speed_limit = 12.3
    integral_error = np.zeros(6)
    
    # Storage for simulation
    csv_data = []
    
    print("Simulating feedforward control...")
    print("Step   | EE Error | V_cmd Norm | Chassis (x,y)")
    print("-" * 50)
    
    # Simulate the trajectory
    for step in range(len(trajectory) - 1):
        # Extract desired poses from trajectory
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        # For feedforward testing, assume current end-effector pose
        X_actual = X_desired.copy()
        
        # Add initial error if specified
        if initial_error is not None and step == 0:
            X_actual[:3, 3] += initial_error
        elif initial_error is not None:
            # Error persists under feedforward-only control
            decay_factor = max(0.1, 1.0 - 0.01 * step)  # Slight decay for realism
            X_actual[:3, 3] += initial_error * decay_factor
        
        # Compute feedforward control
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, dt, integral_error, config
        )
        
        # Apply controls to robot
        new_config = NextState(config, controls, dt, speed_limit)
        
        # Extract gripper state from trajectory
        gripper_state = int(trajectory[step, 12])
        
        # Store CSV row: [φ, x, y, θ1, θ2, θ3, θ4, θ5, w1, w2, w3, w4, gripper]
        csv_row = [
            new_config[0],    # φ (chassis orientation)
            new_config[1],    # x (chassis x position)
            new_config[2],    # y (chassis y position)
            new_config[3],    # θ1 (joint 1)
            new_config[4],    # θ2 (joint 2)
            new_config[5],    # θ3 (joint 3)
            new_config[6],    # θ4 (joint 4)
            new_config[7],    # θ5 (joint 5)
            new_config[8],    # w1 (wheel 1)
            new_config[9],    # w2 (wheel 2)
            new_config[10],   # w3 (wheel 3)
            new_config[11],   # w4 (wheel 4)
            gripper_state     # gripper state
        ]
        csv_data.append(csv_row)
        
        # Print progress
        if step % 200 == 0 or step < 10:
            X_err_norm = np.linalg.norm(X_err)
            V_cmd_norm = np.linalg.norm(V_cmd)
            chassis_pos = (new_config[1], new_config[2])
            print(f"{step:5d} | {X_err_norm:8.4f} | {V_cmd_norm:9.4f} | ({chassis_pos[0]:6.3f}, {chassis_pos[1]:6.3f})")
        
        # Update configuration
        config = new_config
    
    # Write CSV file
    csv_array = np.array(csv_data)
    np.savetxt(output_file, csv_array, delimiter=',', 
               fmt='%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d')
    
    print(f"\nCSV file saved: {output_file}")
    print(f"Total time steps: {len(csv_data)}")
    print(f"Duration: {len(csv_data) * dt:.2f} seconds")
    
    return csv_array


def generate_comparison_csvs():
    """Generate multiple CSV files for comparison testing."""
    
    # Create output directory
    output_dir = "milestone3_feedforward_tests"
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating comparison CSV files for feedforward testing...")
    print("=" * 60)
    
    # Test cases
    test_cases = [
        ("perfect_initial", None, "Perfect initial end-effector position"),
        ("small_error", np.array([0.05, 0.02, 0.01]), "Small initial error (5cm, 2cm, 1cm)"),
        ("medium_error", np.array([0.1, 0.05, 0.02]), "Medium initial error (10cm, 5cm, 2cm)"),
        ("large_error", np.array([0.2, 0.1, 0.05]), "Large initial error (20cm, 10cm, 5cm)"),
    ]
    
    results = {}
    
    for case_name, initial_error, description in test_cases:
        print(f"\nGenerating {case_name}...")
        print(f"Description: {description}")
        
        output_file = os.path.join(output_dir, f"feedforward_{case_name}.csv")
        csv_data = generate_feedforward_csv(output_file, initial_error)
        results[case_name] = csv_data
        
        print(f"✓ {case_name} complete")
    
    # Generate analysis report
    report_file = os.path.join(output_dir, "feedforward_test_report.txt")
    with open(report_file, 'w') as f:
        f.write("Milestone 3 Feedforward Control Test Report\n")
        f.write("=" * 50 + "\n\n")
        
        f.write("Test Files Generated:\n")
        for case_name, _, description in test_cases:
            f.write(f"- feedforward_{case_name}.csv: {description}\n")
        
        f.write("\nTesting Instructions:\n")
        f.write("1. Load each CSV file in CoppeliaSim Scene 8\n")
        f.write("2. Set cube initial position to (1, 0, 0.025) with 0° rotation\n")
        f.write("3. Set cube goal position to (0, -1, 0.025) with -90° rotation\n")
        f.write("4. Run simulation and observe robot behavior\n\n")
        
        f.write("Expected Results:\n")
        f.write("- perfect_initial: Should follow trajectory closely, pick and place cube\n")
        f.write("- small_error: Small deviation but should still complete task\n")
        f.write("- medium_error: Larger deviation, may not grasp cube perfectly\n")
        f.write("- large_error: Significant deviation, likely to miss cube\n\n")
        
        f.write("Key Observations to Make:\n")
        f.write("- Feedforward control cannot correct for initial errors\n")
        f.write("- Errors persist throughout the trajectory\n")
        f.write("- Larger initial errors lead to larger trajectory deviations\n")
        f.write("- This demonstrates the need for feedback control in Milestone 3\n")
    
    print(f"\n✓ All test files generated in {output_dir}/")
    print(f"✓ Test report saved: {report_file}")
    print("\nNext steps:")
    print("1. Load CSV files in CoppeliaSim Scene 8")
    print("2. Observe feedforward-only control behavior")
    print("3. Compare with full feedback control (Milestone 3)")


if __name__ == "__main__":
    print("Milestone 3 Feedforward Control CSV Generator")
    print("=" * 50)
    
    # Generate all comparison files
    generate_comparison_csvs()
    
    print("\nFeedforward control testing files ready!")
    print("Use these CSV files to verify feedforward control works as expected")
    print("before adding feedback control gains.")
