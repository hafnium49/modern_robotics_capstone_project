import os
import sys
import numpy as np
import csv

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from next_state import NextState


def run_for_one_second(controls, speed_limit=20.0):
    config = np.zeros(12)
    dt = 0.01
    for _ in range(100):
        config = NextState(config, controls, dt, speed_limit)
    return config


def generate_csv_trajectory(controls, speed_limit=20.0, steps=100, dt=0.01, initial_config=None):
    """
    Generate a complete CSV trajectory compatible with CoppeliaSim Scene 6.
    
    Returns trajectory data where each row contains:
    chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
    """
    if initial_config is None:
        config = np.zeros(12)
    else:
        config = np.array(initial_config, dtype=float)
    
    trajectory = []
    gripper_state = 0  # 0 = open, 1 = closed
    
    # Add initial configuration as first row
    row = list(config) + [gripper_state]
    trajectory.append(row)
    
    # Generate trajectory for specified number of steps
    for _ in range(steps):
        config = NextState(config, controls, dt, speed_limit)
        row = list(config) + [gripper_state]
        trajectory.append(row)
    
    return trajectory


def test_forward_x():
    """Test 1: u = (10, 10, 10, 10) - robot drives forward in +x̂_b direction by 0.475m"""
    controls = [10, 10, 10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[1], 0.475, atol=1e-3)  # x displacement
    assert np.allclose(cfg[8], 10.0, atol=1e-6)   # wheel angle updated


def test_forward_y():
    """Test 2: u = (-10, 10, -10, 10) - robot slides sideways in +ŷ_b direction by 0.475m"""
    controls = [-10, 10, -10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[2], 0.475, atol=1e-3)  # y displacement


def test_rotation():
    """Test 3: u = (-10, 10, 10, -10) - robot spins counter-clockwise by 1.234 radians"""
    controls = [-10, 10, 10, -10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[0], 1.234, atol=1e-3)  # φ rotation


def test_speed_limit():
    """Test 4: Speed limit test - with limit=5, robot should move half distance (0.2375m)"""
    controls = [10, 10, 10, 10] + [0]*5
    cfg = run_for_one_second(controls, speed_limit=5.0)
    assert np.allclose(cfg[1], 0.2375, atol=1e-3)  # half of 0.475m


def save_test_configuration(config, filename, description):
    """Save a robot configuration to CSV file in milestone1 directory."""
    # Create milestone1 directory in project root
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    os.makedirs(output_dir, exist_ok=True)
    
    filepath = os.path.join(output_dir, filename)
    
    # Save configuration with description
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f'# {description}'])
        writer.writerow(['phi', 'x', 'y', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 
                        'wheel1', 'wheel2', 'wheel3', 'wheel4'])
        writer.writerow([f'{val:.6f}' for val in config])
    
    print(f"Test configuration saved to: {filepath}")
    return filepath


def save_csv_trajectory(trajectory, filename, description):
    """Save a complete trajectory to CSV file compatible with CoppeliaSim Scene 6."""
    # Create milestone1 directory in project root
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    os.makedirs(output_dir, exist_ok=True)
    
    filepath = os.path.join(output_dir, filename)
    
    # Save trajectory with description header
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f'# {description}'])
        writer.writerow(['phi', 'x', 'y', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 
                        'wheel1', 'wheel2', 'wheel3', 'wheel4', 'gripper'])
        
        # Write trajectory data
        for row in trajectory:
            formatted_row = [f'{val:.6f}' if isinstance(val, (int, float, np.number)) else str(val) for val in row]
            writer.writerow(formatted_row)
    
    print(f"CSV trajectory saved to: {filepath} ({len(trajectory)} rows)")
    return filepath


def save_csv_trajectory_scene6_format(trajectory, filename, description):
    """Save trajectory to CSV file in exact Scene6 format (no headers, just data)."""
    # Create milestone1 directory in project root
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    os.makedirs(output_dir, exist_ok=True)
    
    filepath = os.path.join(output_dir, filename)
    
    # Save trajectory in pure Scene6 format - no headers, just data
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write trajectory data without any headers (exactly like Scene6_example.csv)
        for row in trajectory:
            # Format numbers to match Scene6 precision (5 decimal places)
            formatted_row = []
            for val in row:
                if isinstance(val, (int, float, np.number)):
                    formatted_row.append(f"{val:.5f}")
                else:
                    formatted_row.append(str(val))
            writer.writerow(formatted_row)
    
    print(f"Scene6 format CSV saved to: {filepath} ({len(trajectory)} rows)")
    return filepath


def test_generate_milestone1_outputs():
    """Generate test output files for milestone1 verification."""
    print("\nGenerating Milestone 1 test outputs...")
    
    # Test case 1: Forward motion trajectory
    print("Generating forward motion trajectory...")
    controls = [10, 10, 10, 10] + [0]*5
    trajectory1 = generate_csv_trajectory(controls, speed_limit=20.0, steps=100)
    save_csv_trajectory(trajectory1, 'test_case1.csv', 
                       'Forward motion: u=(10,10,10,10), expect x=0.475m after 1s')
    save_csv_trajectory_scene6_format(trajectory1, 'test_case1_scene6.csv',
                                     'Forward motion for Scene6')
    
    # Test case 2: Sideways motion trajectory
    print("Generating sideways motion trajectory...")
    controls = [-10, 10, -10, 10] + [0]*5
    trajectory2 = generate_csv_trajectory(controls, speed_limit=20.0, steps=100)
    save_csv_trajectory(trajectory2, 'test_case2.csv',
                       'Sideways motion: u=(-10,10,-10,10), expect y=0.475m after 1s')
    save_csv_trajectory_scene6_format(trajectory2, 'test_case2_scene6.csv',
                                     'Sideways motion for Scene6')
    
    # Test case 3: Rotation trajectory
    print("Generating rotation trajectory...")
    controls = [-10, 10, 10, -10] + [0]*5
    trajectory3 = generate_csv_trajectory(controls, speed_limit=20.0, steps=100)
    save_csv_trajectory(trajectory3, 'test_case3.csv',
                       'Rotation: u=(-10,10,10,-10), expect phi=1.234 rad after 1s')
    save_csv_trajectory_scene6_format(trajectory3, 'test_case3_scene6.csv',
                                     'Rotation for Scene6')
    
    # Initial configuration (single row for reference)
    initial_config = np.zeros(12)
    save_test_configuration(initial_config, 'initial_config.csv',
                           'Initial configuration: all zeros')
    
    # Generate additional test trajectories with different parameters
    
    # Test case 4: Speed-limited forward motion
    print("Generating speed-limited forward motion trajectory...")
    controls = [10, 10, 10, 10] + [0]*5
    trajectory4 = generate_csv_trajectory(controls, speed_limit=5.0, steps=100)
    save_csv_trajectory(trajectory4, 'test_case_speed_limit.csv',
                       'Speed limited forward motion: u=(10,10,10,10) with limit=5, expect x=0.2375m after 1s')
    
    # Test case 5: Complex motion combining translation and rotation
    print("Generating combined motion trajectory...")
    controls = [5, 8, 5, 8] + [0]*5  # Mixed motion
    trajectory5 = generate_csv_trajectory(controls, speed_limit=20.0, steps=200)  # 2 seconds
    save_csv_trajectory(trajectory5, 'test_case_combined.csv',
                       'Combined motion: u=(5,8,5,8) for 2 seconds')
    
    # Test case 6: Arm joint motion while stationary
    print("Generating arm motion trajectory...")
    controls = [0, 0, 0, 0] + [1, -0.5, 0.8, -0.3, 0.2]  # Only arm joints moving
    trajectory6 = generate_csv_trajectory(controls, speed_limit=20.0, steps=150)  # 1.5 seconds
    save_csv_trajectory(trajectory6, 'test_case_arm_motion.csv',
                       'Arm motion only: chassis stationary, joints moving')
    
    print("✅ Successfully generated 10 milestone1 test files")
    
    # Verify files exist
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    expected_files = [
        'test_case1.csv', 'test_case2.csv', 'test_case3.csv', 
        'test_case1_scene6.csv', 'test_case2_scene6.csv', 'test_case3_scene6.csv',
        'initial_config.csv', 'test_case_speed_limit.csv',
        'test_case_combined.csv', 'test_case_arm_motion.csv'
    ]
    
    for filename in expected_files:
        filepath = os.path.join(output_dir, filename)
        assert os.path.exists(filepath), f"Expected file {filepath} was not created"
        assert os.path.getsize(filepath) > 0, f"File {filepath} is empty"


def test_csv_format_compatibility():
    """Test that generated CSV files have the correct format for CoppeliaSim Scene 6."""
    print("\nTesting CSV format compatibility with CoppeliaSim Scene 6...")
    
    # Generate a short test trajectory
    controls = [5, 5, 5, 5] + [0]*5
    trajectory = generate_csv_trajectory(controls, steps=5)
    
    # Test trajectory structure
    assert len(trajectory) == 6, "Trajectory should have initial state + 5 steps = 6 rows"
    
    # Test row format
    for i, row in enumerate(trajectory):
        assert len(row) == 13, f"Row {i} should have 13 values (12 config + 1 gripper)"
        
        # Check data types
        for j, val in enumerate(row):
            if j < 12:  # Configuration values
                assert isinstance(val, (int, float, np.number)), f"Config value at [{i}][{j}] should be numeric"
            else:  # Gripper state
                assert val in [0, 1], f"Gripper state at [{i}][{j}] should be 0 or 1"
    
    print("✅ CSV format compatibility test passed")


def test_scene6_example_compatibility():
    """Test compatibility with the Scene6_example.csv format provided in the course."""
    print("\nTesting compatibility with Scene6 example format...")
    
    # Generate a trajectory similar to the provided example
    controls = [2, 2, 2, 2] + [0.1, -0.1, 0.05, -0.05, 0.02]  # Gentle motion
    trajectory = generate_csv_trajectory(controls, steps=10)
    
    # Save as Scene6-compatible format
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    filepath = os.path.join(output_dir, 'scene6_compatible_example.csv')
    
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write trajectory data without headers (as in Scene6_example.csv)
        for row in trajectory:
            # Format numbers to match Scene6 example precision
            formatted_row = []
            for val in row:
                if isinstance(val, (int, float, np.number)):
                    formatted_row.append(f"{val:.5f}")
                else:
                    formatted_row.append(str(val))
            writer.writerow(formatted_row)
    
    print(f"✅ Scene6-compatible example saved to: {filepath}")
    
    # Verify file was created and has correct structure
    assert os.path.exists(filepath), f"Scene6 compatible file was not created"
    
    with open(filepath, 'r') as csvfile:
        reader = csv.reader(csvfile)
        rows = list(reader)
        
    assert len(rows) == 11, "Should have 11 rows (initial + 10 steps)"
    
    for i, row in enumerate(rows):
        assert len(row) == 13, f"Row {i} should have 13 comma-separated values"
    
    print("✅ Scene6 compatibility test passed")


if __name__ == "__main__":
    # Run all tests when executed directly
    test_forward_x()
    test_forward_y() 
    test_rotation()
    test_speed_limit()
    test_csv_format_compatibility()
    test_generate_milestone1_outputs()
    test_scene6_example_compatibility()
    print("\n🎉 All Milestone 1 tests passed successfully!")
