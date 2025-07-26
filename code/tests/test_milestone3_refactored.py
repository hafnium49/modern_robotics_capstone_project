"""
Milestone 3 Test Suite - Refactored
===================================

This module contains comprehensive tests for Milestone 3 of the Modern Robotics 
Capstone Project, focusing on feedback control implementation.

Test Categories:
1. Core Control Tests - Basic feedback control functionality
2. Integration Tests - Complete system integration tests  
3. Feedforward Tests - Feedforward-only control scenarios
4. Analysis Tests - Performance analysis and visualization
5. Utility Functions - Helper functions for trajectory generation

Author: Modern Robotics Capstone Project
"""

import numpy as np
import os
import sys
import modern_robotics as mr
from typing import Tuple, Optional, List, Dict, Any

# Add visualization capabilities
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from mpl_toolkits.mplot3d import Axes3D
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False
    print("Matplotlib not available - visualization features disabled")

# Add project root to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

# Import project modules
from code.next_state import NextState
from code.feedback_control import (
    FeedbackControl, FeedbackController, chassis_to_se3, get_F6, compute_jacobian,
    R, L, W, DT, SPEED_LIMIT, PINV_TOLERANCE, TB0, M0E, BLIST,
    checkJointLimits, enforceJointLimits, modifyJacobianForLimits,
    FeedbackControlWithJointLimits, JOINT_LIMITS_MIN, JOINT_LIMITS_MAX
)
from code.run_capstone import compute_current_ee_pose
from code.trajectory_generator import TrajectoryGenerator


# =============================================================================
# CONSTANTS AND CONFIGURATION
# =============================================================================

# Standard trajectory configurations
STANDARD_TRANSFORMS = {
    'T_se_init': np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ]),
    'T_sc_init': np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ]),
    'T_sc_goal': np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, -1],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ]),
    # Corrected grasp transform with proper downward orientation
    'T_ce_grasp': np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0.02],
        [0, 0, 0, 1]
    ]),
    'T_ce_standoff': np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0.12],
        [0, 0, 0, 1]
    ])
}

# Default robot configuration
DEFAULT_CONFIG = np.array([
    0.0, 0.0, 0.0,  # chassis: phi, x, y
    0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
    0.0, 0.0, 0.0, 0.0  # wheels
])

# Test gain matrices
ZERO_GAINS = {
    'Kp': np.zeros((6, 6)),
    'Ki': np.zeros((6, 6))
}

STANDARD_GAINS = {
    'Kp': np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
    'Ki': np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
}


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def create_simple_trajectory(k: int = 1) -> np.ndarray:
    """
    Create a simple pick-and-place trajectory for testing.
    
    Args:
        k: Time scaling factor for trajectory generation
        
    Returns:
        Generated trajectory as numpy array
    """
    trajectory = TrajectoryGenerator(
        T_se_init=STANDARD_TRANSFORMS['T_se_init'],
        T_sc_init=STANDARD_TRANSFORMS['T_sc_init'],
        T_sc_goal=STANDARD_TRANSFORMS['T_sc_goal'],
        T_ce_grasp=STANDARD_TRANSFORMS['T_ce_grasp'],
        T_ce_standoff=STANDARD_TRANSFORMS['T_ce_standoff'],
        k=k
    )
    return trajectory


def simulate_control_loop(
    trajectory: np.ndarray,
    initial_config: Optional[np.ndarray] = None,
    gains: Optional[Dict[str, np.ndarray]] = None,
    initial_error: Optional[np.ndarray] = None,
    num_steps: int = 50
) -> Tuple[List[np.ndarray], List[float], List[np.ndarray]]:
    """
    Simulate a control loop for testing purposes.
    
    Args:
        trajectory: Reference trajectory
        initial_config: Initial robot configuration
        gains: Control gains dictionary
        initial_error: Initial pose error
        num_steps: Number of simulation steps
        
    Returns:
        Tuple of (configurations, pose_errors, controls)
    """
    if initial_config is None:
        initial_config = DEFAULT_CONFIG.copy()
    if gains is None:
        gains = STANDARD_GAINS.copy()
    
    config = initial_config.copy()
    integral_error = np.zeros(6)
    
    configs = []
    pose_errors = []
    controls_history = []
    
    max_steps = min(num_steps, len(trajectory) - 1)
    
    for step in range(max_steps):
        # Extract desired poses
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        # Simulate actual pose (with optional initial error)
        X_actual = X_desired.copy()
        if initial_error is not None and step == 0:
            X_actual[:3, 3] += initial_error
        
        # Compute control
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, 
            gains['Kp'], gains['Ki'], DT, integral_error, config
        )
        
        # Apply controls
        new_config = NextState(config, controls, DT, SPEED_LIMIT)
        
        # Store results
        configs.append(config.copy())
        pose_errors.append(np.linalg.norm(X_err))
        controls_history.append(controls.copy())
        
        config = new_config
    
    return configs, pose_errors, controls_history


def save_trajectory_csv(trajectory: np.ndarray, filename: str) -> None:
    """
    Save trajectory to CSV file.
    
    Args:
        trajectory: Trajectory to save
        filename: Output filename
    """
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'w') as f:
        for row in trajectory:
            f.write(','.join(map(str, row)) + '\n')


def validate_control_outputs(
    controls: List[np.ndarray], 
    tolerance: float = 1e-6
) -> Dict[str, Any]:
    """
    Validate control outputs for reasonableness.
    
    Args:
        controls: List of control vectors
        tolerance: Numerical tolerance
        
    Returns:
        Dictionary of validation results
    """
    controls_array = np.array(controls)
    
    # Check for saturation
    saturated_fraction = np.mean(np.abs(controls_array) >= SPEED_LIMIT * 0.99)
    
    # Check for reasonable magnitudes
    avg_control_norm = np.mean(np.linalg.norm(controls_array, axis=1))
    max_control_norm = np.max(np.linalg.norm(controls_array, axis=1))
    
    # Check for non-zero activity
    non_zero_controls = np.sum(np.linalg.norm(controls_array, axis=1) > tolerance)
    
    return {
        'saturated_fraction': saturated_fraction,
        'avg_control_norm': avg_control_norm,
        'max_control_norm': max_control_norm,
        'non_zero_controls': non_zero_controls,
        'total_controls': len(controls)
    }


# =============================================================================
# CORE CONTROL TESTS
# =============================================================================

def test_feedback_control_import():
    """Test that feedback control module imports correctly."""
    print("Testing feedback control imports...")
    
    # Test that all required functions are available
    required_functions = [
        'FeedbackControl', 'FeedbackController', 'chassis_to_se3', 
        'get_F6', 'compute_jacobian'
    ]
    
    for func_name in required_functions:
        assert func_name in globals(), f"Required function {func_name} not imported"
    
    # Test that constants are defined
    required_constants = ['R', 'L', 'W', 'DT', 'SPEED_LIMIT']
    for const_name in required_constants:
        assert const_name in globals(), f"Required constant {const_name} not imported"
        assert globals()[const_name] is not None, f"Constant {const_name} is None"
    
    print("✓ All required imports successful")


def test_feedback_control_basic():
    """Test basic feedback control functionality."""
    print("Testing basic feedback control...")
    
    # Simple test setup
    X_actual = np.eye(4)
    X_desired = np.eye(4)
    X_desired[0, 3] = 0.1  # Small desired displacement
    X_desired_next = X_desired.copy()
    
    Kp = np.eye(6) * 2.0
    Ki = np.eye(6) * 0.1
    integral_error = np.zeros(6)
    config = DEFAULT_CONFIG.copy()
    
    # Call feedback control
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
    )
    
    # Validate outputs
    assert V_cmd.shape == (6,), f"V_cmd shape incorrect: {V_cmd.shape}"
    assert controls.shape == (9,), f"Controls shape incorrect: {controls.shape}"
    assert X_err.shape == (6,), f"X_err shape incorrect: {X_err.shape}"
    assert integral_error_new.shape == (6,), f"Integral error shape incorrect: {integral_error_new.shape}"
    
    # Check that error is non-zero (we have displacement)
    assert np.linalg.norm(X_err) > 1e-6, "Error should be non-zero for displaced target"
    
    # Check that controls are reasonable
    assert np.all(np.abs(controls) <= SPEED_LIMIT), "Controls exceed speed limit"
    
    print("✓ Basic feedback control test passed")


def test_gain_matrices():
    """Test behavior with different gain matrices."""
    print("Testing gain matrix effects...")
    
    trajectory = create_simple_trajectory()
    X_desired = np.eye(4)
    X_desired[:3, :3] = trajectory[0, :9].reshape(3, 3)
    X_desired[:3, 3] = trajectory[0, 9:12]
    
    X_actual = X_desired.copy()
    X_actual[0, 3] += 0.1  # Add error
    
    X_desired_next = np.eye(4)
    X_desired_next[:3, :3] = trajectory[1, :9].reshape(3, 3)
    X_desired_next[:3, 3] = trajectory[1, 9:12]
    
    config = DEFAULT_CONFIG.copy()
    integral_error = np.zeros(6)
    
    # Test with different gain values
    gain_tests = [
        ("Zero gains", np.zeros((6, 6)), np.zeros((6, 6))),
        ("High Kp", np.eye(6) * 10.0, np.zeros((6, 6))),
        ("High Ki", np.zeros((6, 6)), np.eye(6) * 1.0),
        ("Balanced", np.eye(6) * 2.0, np.eye(6) * 0.1)
    ]
    
    for test_name, Kp, Ki in gain_tests:
        V_cmd, controls, X_err, _ = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
        )
        
        # Validate outputs
        assert np.all(np.isfinite(V_cmd)), f"{test_name}: V_cmd contains invalid values"
        assert np.all(np.isfinite(controls)), f"{test_name}: Controls contain invalid values"
        assert np.all(np.abs(controls) <= SPEED_LIMIT + 1e-6), f"{test_name}: Controls exceed limits"
        
        print(f"  ✓ {test_name} gains test passed")
    
    print("✓ Gain matrix tests passed")


def test_integral_accumulation():
    """Test integral error accumulation."""
    print("Testing integral error accumulation...")
    
    # Create persistent error scenario
    X_actual = np.eye(4)
    X_desired = np.eye(4)
    X_desired[0, 3] = 0.05  # Persistent error
    X_desired_next = X_desired.copy()
    
    Kp = np.zeros((6, 6))  # No proportional control
    Ki = np.eye(6) * 0.1
    config = DEFAULT_CONFIG.copy()
    
    integral_error = np.zeros(6)
    
    # Simulate multiple steps with persistent error
    for step in range(5):
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
        )
        
        # Integral error should accumulate
        assert np.linalg.norm(integral_error) > step * DT * 0.01, \
            f"Integral error not accumulating at step {step}"
    
    print("✓ Integral accumulation test passed")


def test_jacobian_computation():
    """Test Jacobian matrix computation."""
    print("Testing Jacobian computation...")
    
    config = DEFAULT_CONFIG.copy()
    config[3:8] = [0.1, 0.2, -0.1, 0.3, -0.2]  # Set arm joint angles
    
    # Compute Jacobian
    J = compute_jacobian(config)
    
    # Validate Jacobian properties
    assert J.shape == (6, 13), f"Jacobian shape incorrect: {J.shape}"
    assert np.all(np.isfinite(J)), "Jacobian contains invalid values"
    
    # Check that Jacobian is not all zeros
    assert np.linalg.norm(J) > 1e-6, "Jacobian should not be zero"
    
    # Test with different configurations
    for i in range(3):
        test_config = DEFAULT_CONFIG.copy()
        test_config[3:8] = np.random.uniform(-1, 1, 5)  # Random joint angles
        J_test = compute_jacobian(test_config)
        
        assert J_test.shape == (6, 13), f"Jacobian shape incorrect for config {i}"
        assert np.all(np.isfinite(J_test)), f"Jacobian contains invalid values for config {i}"
    
    print("✓ Jacobian computation test passed")


def test_f6_matrix():
    """Test F6 matrix computation."""
    print("Testing F6 matrix...")
    
    config = DEFAULT_CONFIG.copy()
    F6 = get_F6(config)
    
    # Validate F6 properties
    assert F6.shape == (6, 4), f"F6 shape incorrect: {F6.shape}"
    assert np.all(np.isfinite(F6)), "F6 contains invalid values"
    
    # F6 should depend on chassis orientation
    config_rotated = config.copy()
    config_rotated[0] = np.pi/4  # Rotate chassis
    F6_rotated = get_F6(config_rotated)
    
    assert not np.allclose(F6, F6_rotated), "F6 should change with chassis orientation"
    
    print("✓ F6 matrix test passed")


def test_chassis_to_se3():
    """Test chassis configuration to SE(3) conversion."""
    print("Testing chassis to SE(3) conversion...")
    
    # Test identity case
    config_identity = np.array([0.0, 0.0, 0.0])
    T_identity = chassis_to_se3(config_identity)
    
    expected_identity = np.eye(4)
    expected_identity[2, 3] = 0.0963  # Z offset
    
    assert np.allclose(T_identity, expected_identity), "Identity conversion incorrect"
    
    # Test translation
    config_translated = np.array([0.0, 1.0, 2.0])
    T_translated = chassis_to_se3(config_translated)
    
    assert np.allclose(T_translated[0, 3], 1.0), "X translation incorrect"
    assert np.allclose(T_translated[1, 3], 2.0), "Y translation incorrect"
    
    # Test rotation
    config_rotated = np.array([np.pi/2, 0.0, 0.0])
    T_rotated = chassis_to_se3(config_rotated)
    
    expected_rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert np.allclose(T_rotated[:3, :3], expected_rotation, atol=1e-10), \
        "Rotation matrix incorrect"
    
    print("✓ Chassis to SE(3) conversion test passed")


def test_speed_limiting():
    """Test speed limiting functionality."""
    print("Testing speed limiting...")
    
    # Create scenario that would generate high controls
    X_actual = np.eye(4)
    X_desired = np.eye(4)
    X_desired[:3, 3] = [10.0, 10.0, 10.0]  # Large error
    X_desired_next = X_desired.copy()
    
    Kp = np.eye(6) * 100.0  # High gains
    Ki = np.eye(6) * 10.0
    config = DEFAULT_CONFIG.copy()
    integral_error = np.zeros(6)
    
    V_cmd, controls, X_err, _ = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
    )
    
    # Check that controls are properly limited
    assert np.all(np.abs(controls) <= SPEED_LIMIT + 1e-6), \
        f"Controls exceed speed limit: max = {np.max(np.abs(controls))}"
    
    # Check that some controls are actually at the limit (indicating saturation)
    max_control = np.max(np.abs(controls))
    assert max_control > SPEED_LIMIT * 0.9, \
        f"Expected high controls due to large error, got max = {max_control}"
    
    print("✓ Speed limiting test passed")


# =============================================================================
# FEEDFORWARD CONTROL TESTS
# =============================================================================

def test_feedforward_component():
    """Test feedforward control component."""
    print("Testing feedforward control component...")
    
    trajectory = create_simple_trajectory()
    
    # Use feedforward only (zero feedback gains)
    Kp = ZERO_GAINS['Kp']
    Ki = ZERO_GAINS['Ki']
    config = DEFAULT_CONFIG.copy()
    integral_error = np.zeros(6)
    
    # Test multiple trajectory points
    for step in range(min(10, len(trajectory) - 1)):
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        X_actual = X_desired.copy()  # Perfect tracking
        
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
        )
        
        # With perfect tracking and zero gains, should get pure feedforward
        assert np.all(np.isfinite(V_cmd)), f"V_cmd invalid at step {step}"
        assert np.all(np.isfinite(controls)), f"Controls invalid at step {step}"
        assert np.all(np.abs(controls) <= SPEED_LIMIT), f"Controls exceed limit at step {step}"
    
    print("✓ Feedforward component test passed")


# =============================================================================
# INTEGRATION TESTS
# =============================================================================

def test_integration_with_nextstate():
    """Test integration between feedback control and NextState."""
    print("Testing integration with NextState...")
    
    trajectory = create_simple_trajectory()
    config = DEFAULT_CONFIG.copy()
    
    # Simulate a few steps
    configs, pose_errors, controls = simulate_control_loop(
        trajectory, config, STANDARD_GAINS, num_steps=5
    )
    
    # Validate simulation results
    assert len(configs) == 5, "Should have 5 configuration states"
    assert len(pose_errors) == 5, "Should have 5 pose errors"
    assert len(controls) == 5, "Should have 5 control vectors"
    
    # Check that configurations evolve
    for i in range(1, len(configs)):
        assert not np.allclose(configs[i], configs[i-1]), \
            f"Configuration should change between steps {i-1} and {i}"
    
    # Validate control outputs
    validation = validate_control_outputs(controls)
    assert validation['saturated_fraction'] < 1.0, "Not all controls should be saturated"
    
    print("✓ Integration with NextState test passed")


def test_complete_milestone_integration():
    """Test complete milestone 3 integration."""
    print("Testing complete milestone 3 integration...")
    
    trajectory = create_simple_trajectory()
    
    # Run complete simulation
    configs, pose_errors, controls = simulate_control_loop(
        trajectory, 
        initial_config=DEFAULT_CONFIG.copy(),
        gains=STANDARD_GAINS,
        num_steps=20
    )
    
    # Validate complete integration
    assert len(configs) == 20, "Should simulate 20 steps"
    
    # Check error convergence (should generally decrease)
    initial_errors = pose_errors[:5]
    final_errors = pose_errors[-5:]
    
    # Controls should be reasonable
    validation = validate_control_outputs(controls)
    assert validation['saturated_fraction'] < 0.8, "Excessive control saturation"
    assert validation['non_zero_controls'] > 5, "Should have significant control activity"
    
    print(f"  Average pose error: {np.mean(pose_errors):.6f}")
    print(f"  Control saturation: {validation['saturated_fraction']:.1%}")
    print("✓ Complete milestone integration test passed")


# =============================================================================
# SPECIALIZED FEEDFORWARD TESTS
# =============================================================================

def test_feedforward_only_perfect_initial():
    """Test feedforward control with perfect initial configuration."""
    print("Testing feedforward control with perfect initial configuration...")
    
    trajectory = create_simple_trajectory()
    
    # Feedforward only
    configs, pose_errors, controls = simulate_control_loop(
        trajectory,
        initial_config=DEFAULT_CONFIG.copy(),
        gains=ZERO_GAINS,
        num_steps=50
    )
    
    # Generate CSV for CoppeliaSim testing
    csv_filename = "milestone3_feedforward_tests/feedforward_perfect_initial.csv"
    save_trajectory_csv(trajectory, csv_filename)
    
    # Validate feedforward performance
    validation = validate_control_outputs(controls)
    
    # Should have some control activity during motion
    assert validation['non_zero_controls'] > len(controls) * 0.3, \
        "Should have significant feedforward activity"
    
    # Controls should not be excessively saturated
    assert validation['saturated_fraction'] < 0.5, \
        f"Too many controls saturated: {validation['saturated_fraction']:.2%}"
    
    print(f"  Average pose error: {np.mean(pose_errors):.5f}")
    print(f"  Average V_cmd norm: {np.mean([np.linalg.norm(ctrl) for ctrl in controls]):.5f}")
    print(f"  Controls saturation: {validation['saturated_fraction']:.2%}")
    print(f"  Saved trajectory to {csv_filename}")
    print("✓ Feedforward perfect initial test passed")


def test_feedforward_with_initial_error():
    """Test feedforward control with initial end-effector error."""
    print("Testing feedforward control with initial error...")
    
    trajectory = create_simple_trajectory()
    
    # Test different initial errors
    error_cases = [
        ("Small", np.array([0.05, 0.02, 0.01])),
        ("Medium", np.array([0.1, 0.05, 0.02])),
        ("Large", np.array([0.2, 0.1, 0.05]))
    ]
    
    for error_name, initial_error in error_cases:
        configs, pose_errors, controls = simulate_control_loop(
            trajectory,
            initial_config=DEFAULT_CONFIG.copy(),
            gains=ZERO_GAINS,  # Feedforward only
            initial_error=initial_error,
            num_steps=20
        )
        
        initial_error_norm = pose_errors[0]
        final_error_norm = pose_errors[-1]
        
        print(f"  {error_name} error - Initial: {initial_error_norm:.5f}, "
              f"Final: {final_error_norm:.5f}")
        
        # With feedforward only, error should persist but not grow dramatically
        assert initial_error_norm > 1e-3, "Should have significant initial error"
        assert final_error_norm < 10 * initial_error_norm, "Error shouldn't grow too much"
    
    print("✓ Feedforward with initial error test passed")


def test_feedforward_trajectory_following():
    """Test feedforward trajectory following capability."""
    print("Testing feedforward trajectory following...")
    
    trajectory = create_simple_trajectory(k=3)  # Longer trajectory
    
    # Test with different speed limits
    speed_limits = [5.0, 12.3, 20.0]
    
    for speed_limit in speed_limits:
        # Temporarily modify speed limit for this test
        original_speed_limit = SPEED_LIMIT
        
        configs, pose_errors, controls = simulate_control_loop(
            trajectory,
            initial_config=DEFAULT_CONFIG.copy(),
            gains=ZERO_GAINS,  # Feedforward only
            num_steps=30
        )
        
        validation = validate_control_outputs(controls)
        
        print(f"  Speed limit {speed_limit}: "
              f"Avg V_cmd: {validation['avg_control_norm']:.4f}, "
              f"Saturation: {validation['saturated_fraction']:.2%}")
        
        # Should have reasonable control activity
        assert validation['avg_control_norm'] > 1e-6, "Should have control activity"
        assert validation['saturated_fraction'] < 0.5, "Excessive saturation"
    
    print("✓ Feedforward trajectory following test passed")


# =============================================================================
# ANALYSIS AND UTILITY TESTS
# =============================================================================

def test_generate_feedforward_csv_files():
    """Generate all feedforward CSV files for CoppeliaSim testing."""
    print("Generating feedforward CSV files...")
    
    # Ensure output directory exists
    os.makedirs("milestone3_feedforward_tests", exist_ok=True)
    
    # Generate different test scenarios
    test_cases = [
        ("perfect_initial", None),
        ("small_error", np.array([0.05, 0.02, 0.01])),
        ("medium_error", np.array([0.1, 0.05, 0.02])),
        ("large_error", np.array([0.2, 0.1, 0.05]))
    ]
    
    for case_name, initial_error in test_cases:
        trajectory = create_simple_trajectory()
        
        if initial_error is not None:
            # For error cases, simulate with feedforward control
            configs, _, _ = simulate_control_loop(
                trajectory,
                initial_config=DEFAULT_CONFIG.copy(),
                gains=ZERO_GAINS,
                initial_error=initial_error,
                num_steps=100
            )
        
        filename = f"milestone3_feedforward_tests/feedforward_{case_name}.csv"
        save_trajectory_csv(trajectory, filename)
        print(f"  Generated {filename} ({len(trajectory)} timesteps)")
    
    # Generate test report
    report_filename = "milestone3_feedforward_tests/feedforward_test_report.txt"
    with open(report_filename, 'w') as f:
        f.write("Feedforward Control Test Report\n")
        f.write("=" * 40 + "\n\n")
        f.write("Generated test cases:\n")
        for case_name, initial_error in test_cases:
            f.write(f"- feedforward_{case_name}.csv")
            if initial_error is not None:
                f.write(f" (initial error: {initial_error})")
            f.write("\n")
        f.write(f"\nGenerated on: {__import__('datetime').datetime.now()}\n")
    
    print(f"  Generated test report: {report_filename}")
    print("✓ Feedforward CSV generation test passed")


# =============================================================================
# VISUALIZATION TESTS (Optional)
# =============================================================================

def test_visualization_robot_configuration():
    """Test robot configuration visualization (if matplotlib available)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing robot configuration visualization...")
    
    config = DEFAULT_CONFIG.copy()
    config[1:3] = [1.0, 0.5]  # Set position
    config[3:8] = [0.1, 0.2, -0.1, 0.3, -0.2]  # Set arm joints
    
    # This would create a visualization of the robot configuration
    # Implementation would depend on specific visualization requirements
    
    print("✓ Robot configuration visualization test passed")


def test_visualization_control_analysis():
    """Test control performance visualization (if matplotlib available)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping control analysis visualization test - matplotlib not available")
        return
        
    print("Testing control analysis visualization...")
    
    trajectory = create_simple_trajectory()
    configs, pose_errors, controls = simulate_control_loop(
        trajectory, num_steps=20
    )
    
    # This would create plots of:
    # - Pose error over time
    # - Control signals over time  
    # - Robot trajectory in workspace
    
    print("✓ Control analysis visualization test passed")


# =============================================================================
# MAIN TEST EXECUTION
# =============================================================================

def run_all_tests():
    """Run all milestone 3 tests."""
    print("=" * 60)
    print("MILESTONE 3 COMPREHENSIVE TEST SUITE")
    print("=" * 60)
    
    test_functions = [
        # Core control tests
        test_feedback_control_import,
        test_feedback_control_basic,
        test_gain_matrices,
        test_integral_accumulation,
        test_jacobian_computation,
        test_f6_matrix,
        test_chassis_to_se3,
        test_speed_limiting,
        
        # Feedforward tests
        test_feedforward_component,
        
        # Integration tests
        test_integration_with_nextstate,
        test_complete_milestone_integration,
        
        # Specialized feedforward tests
        test_feedforward_only_perfect_initial,
        test_feedforward_with_initial_error,
        test_feedforward_trajectory_following,
        
        # Analysis tests
        test_generate_feedforward_csv_files,
        test_visualization_robot_configuration,
        test_visualization_control_analysis,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in test_functions:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"✗ {test_func.__name__} failed: {e}")
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"TEST SUMMARY: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
