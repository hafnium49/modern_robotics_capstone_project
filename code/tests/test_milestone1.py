import os
import sys
import numpy as np
import csv

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from code.next_state import NextState


def run_for_one_second(controls, speed_limit=20.0):
    config = np.zeros(12)
    dt = 0.01
    for _ in range(100):
        config = NextState(config, controls, dt, speed_limit)
    return config


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
        writer.writerow(['# ' + description])
        writer.writerow(['phi', 'x', 'y', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 
                        'wheel1', 'wheel2', 'wheel3', 'wheel4'])
        writer.writerow([f'{val:.6f}' for val in config])
    
    print(f"Test configuration saved to: {filepath}")
    return filepath


def test_generate_milestone1_outputs():
    """Generate test output files for milestone1 verification."""
    print("\nGenerating Milestone 1 test outputs...")
    
    # Test case 1: Forward motion
    controls = [10, 10, 10, 10] + [0]*5
    config1 = run_for_one_second(controls)
    save_test_configuration(config1, 'test_case1.csv', 
                           'Forward motion: u=(10,10,10,10), expect x=0.475m after 1s')
    
    # Test case 2: Sideways motion  
    controls = [-10, 10, -10, 10] + [0]*5
    config2 = run_for_one_second(controls)
    save_test_configuration(config2, 'test_case2.csv',
                           'Sideways motion: u=(-10,10,-10,10), expect y=0.475m after 1s')
    
    # Test case 3: Rotation
    controls = [-10, 10, 10, -10] + [0]*5
    config3 = run_for_one_second(controls)  
    save_test_configuration(config3, 'test_case3.csv',
                           'Rotation: u=(-10,10,10,-10), expect phi=1.234 rad after 1s')
    
    # Initial configuration (all zeros)
    initial_config = np.zeros(12)
    save_test_configuration(initial_config, 'initial_config.csv',
                           'Initial configuration: all zeros')
    
    print("✅ Successfully generated 4 milestone1 test files")
    
    # Verify files exist
    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'milestone1')
    expected_files = ['test_case1.csv', 'test_case2.csv', 'test_case3.csv', 'initial_config.csv']
    
    for filename in expected_files:
        filepath = os.path.join(output_dir, filename)
        assert os.path.exists(filepath), f"Expected file {filepath} was not created"
        assert os.path.getsize(filepath) > 0, f"File {filepath} is empty"
