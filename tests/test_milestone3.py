import numpy as np
import os
import sys
import modern_robotics as mr

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from modern_robotics_sim.next_state import NextState
from modern_robotics_sim.feedback_control import (
    FeedbackControl, FeedbackController, chassis_to_se3, get_F6, compute_jacobian,
    R, L, W, DT, SPEED_LIMIT, PINV_TOLERANCE, TB0, M0E, BLIST
)
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator


# Global variable to store integral error between calls
_integral_error = np.zeros(6)


def reset_integral_error():
    """Reset the global integral error for testing."""
    global _integral_error
    _integral_error = np.zeros(6)


def test_feedback_control_import():
    """Test that FeedbackControl can be imported and used correctly."""
    reset_integral_error()
    
    # Create test poses
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.1], 
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])
    X_desired_next = np.array([
        [1, 0, 0, 0.2],
        [0, 1, 0, 0.2],
        [0, 0, 1, 0.2], 
        [0, 0, 0, 1]
    ])
    
    # Gain matrices
    Kp = np.diag([5, 5, 5, 5, 5, 5])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Call imported FeedbackControl
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, _integral_error
    )
    
    # Verify return shapes
    assert V_cmd.shape == (6,), f"V_cmd shape {V_cmd.shape} != (6,)"
    assert controls.shape == (9,), f"controls shape {controls.shape} != (9,)"
    assert X_err.shape == (6,), f"X_err shape {X_err.shape} != (6,)"
    assert integral_error_new.shape == (6,), f"integral_error shape {integral_error_new.shape} != (6,)"
    
    # Verify controls are within speed limits
    assert np.all(np.abs(controls) <= SPEED_LIMIT), "Controls exceed speed limit"
    
    print("FeedbackControl import test passed")


def test_feedback_control_basic():
    """Test basic FeedbackControl function properties."""
    reset_integral_error()
    
    # Create test poses
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.1], 
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])
    X_desired_next = np.array([
        [1, 0, 0, 0.2],
        [0, 1, 0, 0.2],
        [0, 0, 1, 0.2], 
        [0, 0, 0, 1]
    ])
    
    # Gain matrices
    Kp = np.diag([5, 5, 5, 5, 5, 5])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Call FeedbackControl
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, _integral_error
    )
    
    # Verify return shapes
    assert V_cmd.shape == (6,), f"V_cmd shape {V_cmd.shape} != (6,)"
    assert controls.shape == (9,), f"controls shape {controls.shape} != (9,)"
    assert X_err.shape == (6,), f"X_err shape {X_err.shape} != (6,)"
    assert integral_error_new.shape == (6,), f"integral_error shape {integral_error_new.shape} != (6,)"
    
    # Verify controls are within speed limits
    assert np.all(np.abs(controls) <= SPEED_LIMIT), "Controls exceed speed limit"
    
    print("Basic FeedbackControl test passed")


def test_gain_matrices():
    """Test that different gain matrices produce different results."""
    reset_integral_error()
    
    # Test poses with some error
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired  # No motion
    
    # Test different Kp values
    Kp_low = np.diag([1, 1, 1, 1, 1, 1])
    Kp_high = np.diag([10, 10, 10, 10, 10, 10])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    V_cmd_low, controls_low, _, _ = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp_low, Ki, DT, _integral_error
    )
    
    V_cmd_high, controls_high, _, _ = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp_high, Ki, DT, _integral_error
    )
    
    # Higher gains should produce larger control commands
    assert np.linalg.norm(V_cmd_high) > np.linalg.norm(V_cmd_low), "Higher Kp should produce larger V_cmd"
    
    print("Gain matrix test passed")


def test_integral_accumulation():
    """Test that integral error accumulates over time."""
    reset_integral_error()
    
    # Persistent error scenario
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired
    
    Kp = np.diag([1, 1, 1, 1, 1, 1])
    Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    
    integral_error = np.zeros(6)
    
    # Run multiple control cycles
    for i in range(5):
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error
        )
        
        # Integral should be accumulating
        if i > 0:
            assert np.linalg.norm(integral_error) > 0, "Integral error should accumulate"
    
    print("Integral accumulation test passed")


def test_jacobian_computation():
    """Test Jacobian computation for different configurations."""
    # Test with zero configuration
    config_zero = np.zeros(12)
    Je_zero = compute_jacobian(config_zero)
    
    assert Je_zero.shape == (6, 9), f"Jacobian shape {Je_zero.shape} != (6, 9)"
    
    # Test with non-zero configuration
    config_nonzero = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.4, 0.5, 0, 0, 0, 0])
    Je_nonzero = compute_jacobian(config_nonzero)
    
    assert Je_nonzero.shape == (6, 9), f"Jacobian shape {Je_nonzero.shape} != (6, 9)"
    
    # Jacobians should be different for different configurations
    assert not np.allclose(Je_zero, Je_nonzero), "Jacobians should differ for different configs"
    
    print("Jacobian computation test passed")


def test_f6_matrix():
    """Test F6 matrix computation."""
    F6 = get_F6()
    
    assert F6.shape == (6, 4), f"F6 shape {F6.shape} != (6, 4)"
    
    # Rows 0, 1, 5 should be zero (no wheel contribution to ωx, ωy, vz)
    assert np.allclose(F6[0, :], 0), "Row 0 of F6 should be zero (ωx)"
    assert np.allclose(F6[1, :], 0), "Row 1 of F6 should be zero (ωy)"  
    assert np.allclose(F6[5, :], 0), "Row 5 of F6 should be zero (vz)"
    
    # Rows 2, 3, 4 should be non-zero (wheel contributions to ωz, vx, vy)
    assert not np.allclose(F6[2, :], 0), "Row 2 of F6 should be non-zero (ωz)"
    assert not np.allclose(F6[3, :], 0), "Row 3 of F6 should be non-zero (vx)"
    assert not np.allclose(F6[4, :], 0), "Row 4 of F6 should be non-zero (vy)"
    
    print("F6 matrix test passed")


def test_chassis_to_se3():
    """Test chassis configuration to SE(3) conversion."""
    # Test identity
    T_identity = chassis_to_se3(0, 0, 0)
    assert np.allclose(T_identity, np.eye(4)), "Identity test failed"
    
    # Test translation
    T_trans = chassis_to_se3(0, 1, 2)
    expected_trans = np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 2],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    assert np.allclose(T_trans, expected_trans), "Translation test failed"
    
    # Test rotation
    T_rot = chassis_to_se3(np.pi/2, 0, 0)
    expected_rot = np.array([
        [0, -1, 0, 0],
        [1,  0, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    assert np.allclose(T_rot, expected_rot), "Rotation test failed"
    
    print("Chassis to SE(3) test passed")


def test_speed_limiting():
    """Test that controls are properly speed limited."""
    reset_integral_error()
    
    # Create scenario that would produce large control commands
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 10],  # Large translation
        [0, 1, 0, 10],
        [0, 0, 1, 10],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired
    
    # Very high gains to force large commands
    Kp = np.diag([1000, 1000, 1000, 1000, 1000, 1000])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, _integral_error
    )
    
    # All controls should be within speed limits
    assert np.all(np.abs(controls) <= SPEED_LIMIT), f"Controls {controls} exceed speed limit {SPEED_LIMIT}"
    
    print("Speed limiting test passed")


def test_feedforward_component():
    """Test that feed-forward component works correctly."""
    reset_integral_error()
    
    # Moving reference trajectory
    X_actual = np.eye(4)
    X_desired = np.eye(4)
    X_desired_next = np.array([
        [1, 0, 0, 0.01],  # Small forward motion
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    
    # Zero gains to isolate feed-forward
    Kp = np.diag([0, 0, 0, 0, 0, 0])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, _integral_error
    )
    
    # Should have non-zero command from feed-forward term
    assert np.linalg.norm(V_cmd) > 1e-6, "Feed-forward should produce non-zero command"
    
    print("Feed-forward component test passed")


def test_integration_with_nextstate():
    """Test integration with NextState function from Milestone 1."""
    reset_integral_error()
    
    # Initial configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Test poses
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired
    
    # Control gains
    Kp = np.diag([5, 5, 5, 5, 5, 5])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Get control commands
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, _integral_error
    )
    
    # Apply to NextState
    new_config = NextState(config, controls, DT, SPEED_LIMIT)
    
    # Verify new configuration has correct shape and reasonable values
    assert new_config.shape == (12,), f"NextState output shape {new_config.shape} != (12,)"
    assert not np.allclose(new_config, config), "Configuration should change"
    
    print("Integration with NextState test passed")


def test_gain_specification_compliance():
    """Test that the implementation accepts the specified gain formats."""
    reset_integral_error()
    
    X_actual = np.eye(4)
    X_desired = np.eye(4)
    X_desired_next = np.eye(4)
    
    # Test specified default gains
    Kp_default = np.diag([5, 5, 5, 5, 5, 5])
    Ki_default = np.diag([0, 0, 0, 0, 0, 0])
    
    V_cmd, controls, X_err, integral_error_new = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp_default, Ki_default, DT, _integral_error
    )
    
    # Should work without errors
    assert V_cmd is not None, "Should handle default gains"
    
    # Test with different gains (as autograder would)
    Kp_test = np.diag([1, 2, 3, 4, 5, 6])
    Ki_test = np.diag([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    
    V_cmd2, controls2, X_err2, integral_error_new2 = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp_test, Ki_test, DT, _integral_error
    )
    
    # Should work with different gains
    assert V_cmd2 is not None, "Should handle different gains"
    
    print("Gain specification compliance test passed")


def test_constants_specification():
    """Test that all required constants match the specification."""
    
    # Verify physical constants
    assert R == 0.0475, f"Wheel radius {R} != 0.0475"
    assert L == 0.235, f"Half front-back separation {L} != 0.235"  
    assert W == 0.150, f"Half side-side separation {W} != 0.150"
    assert DT == 0.01, f"Sample time {DT} != 0.01"
    assert SPEED_LIMIT == 12.3, f"Speed limit {SPEED_LIMIT} != 12.3"
    assert PINV_TOLERANCE == 1e-3, f"Pinv tolerance {PINV_TOLERANCE} != 1e-3"
    
    # Verify transformation matrices
    expected_Tb0 = np.array([
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])
    assert np.allclose(TB0, expected_Tb0), "Tb0 matrix incorrect"
    
    expected_M0e = np.array([
        [1, 0, 0, 0.033],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])
    assert np.allclose(M0E, expected_M0e), "M0e matrix incorrect"
    
    # Verify Blist dimensions
    assert BLIST.shape == (6, 5), f"Blist shape {BLIST.shape} != (6, 5)"
    
    print("Constants specification test passed")


def test_stateful_controller():
    """Test the stateful FeedbackController class."""
    # Test poses with some error
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired
    config = np.zeros(12)
    
    # Create controller
    controller = FeedbackController()
    
    # Run multiple control cycles  
    for i in range(3):
        V_cmd, controls, X_err = controller.control(
            X_actual, X_desired, X_desired_next, config
        )
        
        # Verify return shapes
        assert V_cmd.shape == (6,), f"V_cmd shape {V_cmd.shape} != (6,)"
        assert controls.shape == (9,), f"controls shape {controls.shape} != (9,)"
        assert X_err.shape == (6,), f"X_err shape {X_err.shape} != (6,)"
        
        # Verify controls are within speed limits
        assert np.all(np.abs(controls) <= SPEED_LIMIT), "Controls exceed speed limit"
    
    # Test reset functionality
    controller.reset_integral()
    assert np.allclose(controller.integral_error, 0), "Integral should reset to zero"
    
    print("Stateful controller test passed")


def test_complete_milestone_integration():
    """Test complete integration of all three milestones."""
    # This test simulates a simple control loop with NextState, TrajectoryGenerator, and FeedbackControl
    
    # Initial robot configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Define simple poses for testing
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
    trajectory = TrajectoryGenerator(
        T_se_init=T_se_init,
        T_sc_init=T_sc_init,
        T_sc_goal=T_sc_goal,
        T_ce_grasp=T_ce_grasp,
        T_ce_standoff=T_ce_standoff,
        k=1
    )
    
    # Verify trajectory was generated
    assert trajectory.shape[1] == 13, f"Trajectory has {trajectory.shape[1]} columns, expected 13"
    assert trajectory.shape[0] > 10, "Trajectory should have multiple time steps"
    
    # Create controller
    controller = FeedbackController()
    
    # Simulate a few control steps
    for i in range(min(5, len(trajectory)-1)):
        # Extract poses from trajectory (convert from matrix form)
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[i, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[i, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[i+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[i+1, 9:12]
        
        # Current end-effector pose (simplified)
        X_actual = X_desired  # Assume perfect tracking for this test
        
        # Compute control commands
        V_cmd, controls, X_err = controller.control(
            X_actual, X_desired, X_desired_next, config
        )
        
        # Apply controls to NextState
        new_config = NextState(config, controls, DT, SPEED_LIMIT)
        
        # Update configuration
        config = new_config
        
        # Verify reasonable evolution
        assert not np.any(np.isnan(config)), "Configuration contains NaN"
        assert not np.any(np.isinf(config)), "Configuration contains Inf"
    
    print("Complete milestone integration test passed")


if __name__ == "__main__":
    print("Running Milestone 3 tests...")
    print("=" * 50)
    
    test_constants_specification()
    test_chassis_to_se3()
    test_f6_matrix()
    test_jacobian_computation()
    test_feedback_control_import()
    test_feedback_control_basic()
    test_gain_matrices()
    test_integral_accumulation()
    test_speed_limiting()
    test_feedforward_component()
    test_gain_specification_compliance()
    test_integration_with_nextstate()
    test_stateful_controller()
    test_complete_milestone_integration()
    
    print("=" * 50)
    print("All Milestone 3 tests passed! ✅")
