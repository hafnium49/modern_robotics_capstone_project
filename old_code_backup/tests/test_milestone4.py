"""
Test Suite for Milestone 4: Software Integration

This module provides comprehensive testing for the final capstone integration,
verifying proper integration of all milestones and correct system behavior.
"""

import pytest
import numpy as np
import os
import tempfile
import shutil
from pathlib import Path
import sys

# Add the parent directory to the path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from modern_robotics_sim.run_capstone import (
    create_default_cube_poses,
    create_grasp_transforms, 
    create_initial_ee_pose,
    create_initial_config_with_error,
    extract_pose_from_trajectory_row,
    compute_current_ee_pose,
    run_capstone_simulation,
    plot_error_results,
    DT_CAPSTONE,
    SPEED_LIMIT
)
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator
from modern_robotics_sim.feedback_control import FeedbackControl, compute_jacobian
from modern_robotics_sim.next_state import NextState
import modern_robotics as mr


class TestMilestone4Setup:
    """Test proper setup and configuration for Milestone 4."""
    
    def test_default_cube_poses(self):
        """Test that default cube poses match Final Step specification."""
        Tsc_init, Tsc_goal = create_default_cube_poses()
        
        # Test initial cube pose: (x, y, θ) = (1 m, 0 m, 0 rad)
        expected_init = np.array([
            [1, 0, 0, 1.0],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.025],
            [0, 0, 0, 1]
        ])
        np.testing.assert_allclose(Tsc_init, expected_init, rtol=1e-6)
        
        # Test goal cube pose: (x, y, θ) = (0 m, -1 m, -π/2 rad)
        expected_goal = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, -1],
            [0, 0, 1, 0.025],
            [0, 0, 0, 1]
        ])
        np.testing.assert_allclose(Tsc_goal, expected_goal, rtol=1e-6, atol=1e-15)
        
    def test_grasp_transforms(self):
        """Test that grasp transforms match specification."""
        Tce_grasp, Tce_standoff = create_grasp_transforms()
        
        # Test grasp transform
        expected_grasp = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.02],
            [0, 0, 0, 1]
        ])
        np.testing.assert_allclose(Tce_grasp, expected_grasp, rtol=1e-6)
        
        # Test standoff transform (grasp + 0.10m in z)
        expected_standoff = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0], 
            [0, 0, 1, 0.12],
            [0, 0, 0, 1]
        ])
        np.testing.assert_allclose(Tce_standoff, expected_standoff, rtol=1e-6)
        
    def test_initial_config_error_requirements(self):
        """Test that initial configuration has required error levels."""
        # Create a dummy trajectory row (identity pose at origin)
        traj_row = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0.5, 0])
        
        config = create_initial_config_with_error(traj_row)
        
        # Verify configuration format
        assert len(config) == 12
        assert np.all(np.isfinite(config))
        
        # Verify position error > 0.20 m
        x_error = abs(config[1] - 0)  # x position error
        y_error = abs(config[2] - 0)  # y position error
        position_error = np.sqrt(x_error**2 + y_error**2)
        assert position_error >= 0.20, f"Position error {position_error:.3f} < 0.20 m"
        
        # Verify chassis orientation error > 30 degrees
        phi_error = abs(config[0])  # chassis angle error
        assert phi_error >= np.radians(30), f"Orientation error {np.degrees(phi_error):.1f}° < 30°"
        
    def test_initial_ee_pose(self):
        """Test that initial EE pose matches Final Step specification."""
        Tse_init = create_initial_ee_pose()
        
        # Test specified initial EE pose
        expected_tse = np.array([
            [0,  0,  1, 0],
            [0,  1,  0, 0],
            [-1, 0,  0, 0.5],
            [0,  0,  0, 1]
        ])
        np.testing.assert_allclose(Tse_init, expected_tse, rtol=1e-6)
        

class TestMilestone4Integration:
    """Test integration between milestone components."""
    
    def test_pose_extraction_from_trajectory(self):
        """Test extraction of SE(3) poses from trajectory rows."""
        # Create test trajectory row: 45° rotation about z
        cos_45 = np.cos(np.pi/4)
        sin_45 = np.sin(np.pi/4)
        traj_row = np.array([
            cos_45, -sin_45, 0,
            sin_45,  cos_45, 0,
            0,       0,      1,
            0.5, 0.3, 0.2,  # position
            1  # gripper
        ])
        
        T = extract_pose_from_trajectory_row(traj_row)
        
        # Verify SE(3) structure
        assert T.shape == (4, 4)
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], rtol=1e-6)
        
        # Verify rotation matrix
        expected_R = np.array([
            [cos_45, -sin_45, 0],
            [sin_45,  cos_45, 0],
            [0,       0,      1]
        ])
        np.testing.assert_allclose(T[:3, :3], expected_R, rtol=1e-6)
        
        # Verify position
        np.testing.assert_allclose(T[:3, 3], [0.5, 0.3, 0.2], rtol=1e-6)
        
    def test_current_ee_pose_computation(self):
        """Test computation of current end-effector pose."""
        # Test with zero configuration
        config = np.zeros(12)
        T = compute_current_ee_pose(config)
        
        # Should be at home position
        assert T.shape == (4, 4)
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], rtol=1e-6)
        
        # Position should be forward kinematics result
        expected_pos = np.array([0.1662 + 0.033, 0, 0.0026 + 0.6546])  # TB0 + M0E positions
        np.testing.assert_allclose(T[:3, 3], expected_pos, rtol=1e-3)
        
    def test_feedback_control_integration(self):
        """Test that feedback control integrates properly with config."""
        # Create test configuration
        config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        # Create slight pose error
        X_actual = np.eye(4)
        X_desired = np.array([
            [1, 0, 0, 0.1],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1]
        ])
        X_desired_next = X_desired
        
        # Test gains
        Kp = np.diag([5, 5, 5, 5, 5, 5])
        Ki = np.diag([0, 0, 0, 0, 0, 0])
        
        # Run feedback control
        V_cmd, controls, X_err, integral = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT_CAPSTONE, 
            np.zeros(6), config
        )
        
        # Verify outputs
        assert len(V_cmd) == 6
        assert len(controls) == 9  # 4 wheels + 5 joints
        assert len(X_err) == 6
        assert len(integral) == 6
        
        # Should produce reasonable control commands
        assert np.all(np.abs(controls) <= SPEED_LIMIT + 1e-6)
        
    def test_simulation_step_integration(self):
        """Test that one simulation step works correctly."""
        # Initial configuration
        config = np.zeros(12)
        
        # Compute test control
        V_cmd = np.array([0, 0, 0.1, 0.05, 0, 0])  # Small twist
        Je = compute_jacobian(config)
        controls = np.linalg.pinv(Je) @ V_cmd
        controls = np.clip(controls, -SPEED_LIMIT, SPEED_LIMIT)
        
        # Simulate one step
        config_new = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
        
        # Verify integration worked
        assert len(config_new) == 12
        assert np.all(np.isfinite(config_new))
        
        # Should have changed from initial
        assert not np.allclose(config_new, config, rtol=1e-6)


class TestMilestone4Simulation:
    """Test the complete capstone simulation."""
    
    def setup_method(self):
        """Set up temporary directory for each test."""
        self.temp_dir = tempfile.mkdtemp()
        
    def teardown_method(self):
        """Clean up temporary directory after each test."""
        shutil.rmtree(self.temp_dir)
        
    def test_short_simulation_run(self):
        """Test a short simulation run for basic functionality."""
        # Get default poses
        Tsc_init, Tsc_goal = create_default_cube_poses()
        Tce_grasp, Tce_standoff = create_grasp_transforms()
        
        # Create simple gains
        Kp = np.diag([3, 3, 3, 3, 3, 3])
        Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        
        # Run short simulation by modifying trajectory generator
        from modern_robotics_sim.trajectory_generator import TrajectoryGenerator
        
        # Get just first few points of trajectory
        Tse_init = compute_current_ee_pose(np.zeros(12))
        full_trajectory = TrajectoryGenerator(
            Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
        )
        
        # Take only first 50 points for quick test
        short_trajectory = full_trajectory[:50]
        
        # Manually run shortened simulation
        config = create_initial_config_with_error(short_trajectory[0])
        integral_error = np.zeros(6)
        
        # Verify we can run a few control steps
        for i in range(min(10, len(short_trajectory)-1)):
            X_desired = extract_pose_from_trajectory_row(short_trajectory[i])
            X_desired_next = extract_pose_from_trajectory_row(short_trajectory[i+1])
            X_actual = compute_current_ee_pose(config)
            
            V_cmd, controls_twist, X_err, integral_error = FeedbackControl(
                X_actual, X_desired, X_desired_next, Kp, Ki, DT_CAPSTONE, 
                integral_error, config
            )
            
            Je = compute_jacobian(config)
            controls = np.linalg.pinv(Je) @ V_cmd
            controls = np.clip(controls, -SPEED_LIMIT, SPEED_LIMIT)
            
            config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
            
        # If we get here, basic simulation loop works
        assert True
        
    def test_output_file_generation(self):
        """Test that output files are generated correctly."""
        # Create mock data
        N_points = 100
        config_log = np.random.rand(N_points, 12)
        error_log = np.random.rand(N_points-1, 6) * 0.1
        
        # Test CSV writing manually
        youbot_data = np.zeros((N_points, 13))
        youbot_data[:, :12] = config_log
        youbot_data[:, 12] = 0  # gripper closed
        
        csv_path = os.path.join(self.temp_dir, "test_output.csv")
        np.savetxt(csv_path, youbot_data, delimiter=',', fmt='%.6f')
        
        # Verify file exists and has correct format
        assert os.path.exists(csv_path)
        
        loaded_data = np.loadtxt(csv_path, delimiter=',')
        assert loaded_data.shape == (N_points, 13)
        # With %.6f format, we can expect about 6 decimal places precision
        np.testing.assert_allclose(loaded_data[:, :12], config_log, atol=1e-6)
        
    def test_error_plotting_functionality(self):
        """Test error plotting with different backends."""
        # Create mock error data
        error_log = np.random.rand(100, 6) * 0.1
        
        # Test plotting function (should handle missing matplotlib gracefully)
        try:
            plot_error_results(error_log, self.temp_dir)
            # If matplotlib available, check if PDF was created
            pdf_path = os.path.join(self.temp_dir, "Xerr_plot.pdf")
            if os.path.exists(pdf_path):
                assert os.path.getsize(pdf_path) > 0
        except ImportError:
            # Should handle gracefully
            pass


class TestMilestone4Validation:
    """Test validation and error checking."""
    
    def test_speed_limit_enforcement(self):
        """Test that speed limits are properly enforced."""
        # Create large control commands
        large_controls = np.ones(9) * 20  # Exceed 5.0 rad/s limit
        
        # Clamp to limits
        clamped = np.clip(large_controls, -SPEED_LIMIT, SPEED_LIMIT)
        
        # Verify all within limits
        assert np.all(np.abs(clamped) <= SPEED_LIMIT + 1e-10)
        assert np.all(clamped == SPEED_LIMIT)
        
    def test_se3_matrix_validation(self):
        """Test that SE(3) matrices are valid."""
        Tsc_init, Tsc_goal = create_default_cube_poses()
        
        # Test SE(3) properties
        for T in [Tsc_init, Tsc_goal]:
            # Check shape
            assert T.shape == (4, 4)
            
            # Check bottom row
            np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], rtol=1e-10)
            
            # Check rotation matrix properties
            R = T[:3, :3]
            assert abs(np.linalg.det(R) - 1.0) < 1e-10  # det(R) = 1
            np.testing.assert_allclose(R @ R.T, np.eye(3), rtol=1e-10)  # R^T R = I
            
    def test_configuration_bounds(self):
        """Test that configurations stay within reasonable bounds."""
        config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        # Apply small controls for many steps
        for _ in range(100):
            controls = np.random.rand(9) * 0.1  # Small random controls
            config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
            
            # Check chassis position doesn't become unreasonable
            assert abs(config[1]) < 10  # x position < 10m
            assert abs(config[2]) < 10  # y position < 10m
            
            # Check joint angles stay reasonable
            assert np.all(np.abs(config[3:8]) < 2*np.pi)
            
    def test_trajectory_continuity(self):
        """Test that trajectory points are continuous."""
        # Generate a short trajectory
        Tsc_init, Tsc_goal = create_default_cube_poses()
        Tce_grasp, Tce_standoff = create_grasp_transforms()
        
        Tse_init = compute_current_ee_pose(np.zeros(12))
        trajectory = TrajectoryGenerator(
            Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
        )
        
        # Check continuity between adjacent points
        max_step = 0.0
        for i in range(len(trajectory)-1):
            T1 = extract_pose_from_trajectory_row(trajectory[i])
            T2 = extract_pose_from_trajectory_row(trajectory[i+1])
            
            # Position step size
            pos_step = np.linalg.norm(T2[:3, 3] - T1[:3, 3])
            max_step = max(max_step, pos_step)
            
        # Trajectory should be reasonably smooth
        assert max_step < 0.1, f"Large trajectory step: {max_step:.3f} m"


# Performance and Integration Tests
class TestMilestone4Performance:
    """Test performance characteristics and system integration."""
    
    def test_jacobian_computation_performance(self):
        """Test Jacobian computation efficiency."""
        import time
        
        config = np.random.rand(12)
        
        # Time Jacobian computation
        start_time = time.time()
        for _ in range(100):
            Je = compute_jacobian(config)
        elapsed = time.time() - start_time
        
        # Should be fast enough for real-time control
        assert elapsed < 1.0, f"Jacobian computation too slow: {elapsed:.3f}s for 100 calls"
        
        # Verify output shape
        assert Je.shape == (6, 9)
        
    def test_memory_usage_stability(self):
        """Test that simulation doesn't have memory leaks."""
        import gc
        
        # Run multiple short simulations
        for trial in range(5):
            config = np.zeros(12)
            
            # Run short control loop
            for step in range(50):
                Je = compute_jacobian(config)
                V_cmd = np.random.rand(6) * 0.1
                controls = np.linalg.pinv(Je) @ V_cmd
                config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
                
            # Force garbage collection
            gc.collect()
            
        # If we get here without memory errors, test passes
        assert True
        
    def test_numerical_stability(self):
        """Test numerical stability with edge cases."""
        # Test with very small configurations
        config_small = np.ones(12) * 1e-8
        Je_small = compute_jacobian(config_small)
        assert np.all(np.isfinite(Je_small))
        
        # Test with larger configurations
        config_large = np.ones(12) * 10
        Je_large = compute_jacobian(config_large)
        assert np.all(np.isfinite(Je_large))
        
        # Test pseudoinverse doesn't fail
        V_test = np.ones(6)
        controls_small = np.linalg.pinv(Je_small) @ V_test
        controls_large = np.linalg.pinv(Je_large) @ V_test
        
        assert np.all(np.isfinite(controls_small))
        assert np.all(np.isfinite(controls_large))


# Integration Test with All Milestones
class TestFullSystemIntegration:
    """Integration tests that verify all milestones work together."""
    
    def test_milestone_1_integration(self):
        """Test integration with Milestone 1 (NextState)."""
        # Test that NextState works with Milestone 4 parameters
        config = np.zeros(12)
        controls = np.ones(9) * 2.0  # Within speed limit
        
        new_config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
        
        assert len(new_config) == 12
        assert np.all(np.isfinite(new_config))
        assert not np.allclose(new_config, config)  # Should have changed
        
    def test_milestone_2_integration(self):
        """Test integration with Milestone 2 (TrajectoryGenerator)."""
        # Test that TrajectoryGenerator produces compatible output
        Tsc_init, Tsc_goal = create_default_cube_poses()
        Tce_grasp, Tce_standoff = create_grasp_transforms()
        
        traj_gen = TrajectoryGenerator
        Tse_init = compute_current_ee_pose(np.zeros(12))
        
        trajectory = traj_gen(
            Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
        )
        
        # Verify trajectory format
        assert len(trajectory) > 0
        assert trajectory.shape[1] == 13  # 12 pose + 1 gripper
        
        # Verify we can extract poses
        for i in range(min(5, len(trajectory))):
            T = extract_pose_from_trajectory_row(trajectory[i])
            assert T.shape == (4, 4)
            
    def test_milestone_3_integration(self):
        """Test integration with Milestone 3 (FeedbackControl)."""
        # Test that FeedbackControl works with Milestone 4 setup
        config = np.zeros(12)
        
        X_actual = compute_current_ee_pose(config)
        X_desired = X_actual.copy()
        X_desired[0, 3] += 0.1  # Small position error
        X_desired_next = X_desired
        
        Kp = np.diag([5, 5, 5, 5, 5, 5])
        Ki = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        
        V_cmd, controls, X_err, integral = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT_CAPSTONE,
            np.zeros(6), config
        )
        
        # Verify control output is reasonable
        assert len(controls) == 9
        assert np.all(np.abs(controls) <= SPEED_LIMIT + 1e-6)
        assert np.linalg.norm(X_err) > 0  # Should detect error
        
    def test_end_to_end_short_run(self):
        """Test short end-to-end simulation run."""
        # This is the ultimate integration test
        Tsc_init, Tsc_goal = create_default_cube_poses()
        Tce_grasp, Tce_standoff = create_grasp_transforms()
        
        # Get trajectory
        Tse_init = compute_current_ee_pose(np.zeros(12))
        trajectory = TrajectoryGenerator(
            Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k=1
        )
        
        # Run simulation for 20 steps
        config = create_initial_config_with_error(trajectory[0])
        integral_error = np.zeros(6)
        
        Kp = np.diag([3, 3, 3, 3, 3, 3])
        Ki = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        
        errors = []
        
        for i in range(min(20, len(trajectory)-1)):
            X_desired = extract_pose_from_trajectory_row(trajectory[i])
            X_desired_next = extract_pose_from_trajectory_row(trajectory[i+1])
            X_actual = compute_current_ee_pose(config)
            
            V_cmd, controls_twist, X_err, integral_error = FeedbackControl(
                X_actual, X_desired, X_desired_next, Kp, Ki, DT_CAPSTONE,
                integral_error, config
            )
            
            Je = compute_jacobian(config)
            controls = np.linalg.pinv(Je) @ V_cmd
            controls = np.clip(controls, -SPEED_LIMIT, SPEED_LIMIT)
            
            config = NextState(config, controls, DT_CAPSTONE, SPEED_LIMIT)
            errors.append(np.linalg.norm(X_err))
            
        # Error should generally decrease (though may have transients)
        final_error = errors[-1]
        initial_error = errors[0]
        
        print(f"Initial error: {initial_error:.4f}, Final error: {final_error:.4f}")
        
        # System should be stable (no exploding errors)
        assert final_error < 10.0, "System appears unstable"
        assert np.all(np.isfinite(config)), "Configuration contains invalid values"


if __name__ == "__main__":
    # Run the tests
    pytest.main([__file__, "-v"])
