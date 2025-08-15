"""
Milestone 4 Integration Utilities

This module provides convenient utilities for running the capstone simulation
with different parameters, scenarios, and analysis tools.
"""

import numpy as np
import os
from .run_capstone import (
    create_default_cube_poses, create_grasp_transforms,
    run_capstone_simulation, plot_error_results
)
from .trajectory_generator import TrajectoryGenerator


def run_feedforward_only_scenario(output_dir="results/feedforward"):
    """Run simulation with feedforward-only control (Kp = Ki = 0).
    
    Args:
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print("=== Running Feedforward-Only Scenario ===")
    
    # Feedforward only: no feedback gains
    Kp = np.diag([0, 0, 0, 0, 0, 0])  # No proportional
    Ki = np.diag([0, 0, 0, 0, 0, 0])  # No integral
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"Feedforward-only scenario completed - check {output_dir}/")
        
    return success


def run_proportional_only_scenario(Kp_gain=1.0, output_dir="results/proportional"):
    """Run simulation with P-control only (Ki = 0).
    
    Args:
        Kp_gain: proportional gain value
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print(f"=== Running P-Control Scenario (Kp={Kp_gain}) ===")
    
    # P-control only
    Kp = np.diag([Kp_gain, Kp_gain, Kp_gain, Kp_gain, Kp_gain, Kp_gain])
    Ki = np.diag([0, 0, 0, 0, 0, 0])  # No integral
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"P-control scenario completed - check {output_dir}/")
        
    return success


def run_pi_only_scenario(Ki_gain=0.1, output_dir="results/pi_only"):
    """Run simulation with PI-control only (no feedforward).
    
    Args:
        Ki_gain: integral gain value
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print(f"=== Running PI-Control Only Scenario (Ki={Ki_gain}) ===")
    
    # PI control only (implemented by modifying feedforward in control loop)
    Kp = np.diag([2, 2, 2, 2, 2, 2])  # Moderate proportional
    Ki = np.diag([Ki_gain, Ki_gain, Ki_gain, Ki_gain, Ki_gain, Ki_gain])
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"PI-only scenario completed - check {output_dir}/")
        
    return success


def run_feedforward_plus_pi_scenario(output_dir="results/feedforward_plus_pi"):
    """Run simulation with feedforward + PI control.
    
    Args:
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print("=== Running Feedforward + PI Scenario ===")
    
    # Feedforward + PI control (well-tuned)
    Kp = np.diag([2, 2, 2, 2, 2, 2])
    Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"Feedforward + PI scenario completed - check {output_dir}/")
        
    return success


def run_overshoot_scenario(output_dir="results/overshoot"):
    """Run simulation with high gains that cause overshoot.
    
    Args:
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print("=== Running Overshoot Scenario ===")
    
    # High proportional gains to cause overshoot
    Kp = np.diag([20, 20, 20, 20, 20, 20])  # High gains
    Ki = np.diag([0, 0, 0, 0, 0, 0])        # No integral
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"Overshoot scenario completed - check {output_dir}/")
        
    return success


def run_newTask_scenario(output_dir="results/newTask"):
    """Run simulation with custom cube poses (new task).
    
    Args:
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print("=== Running New Task Scenario ===")
    
    # Custom cube poses for new task
    Tsc_init = np.array([
        [1, 0, 0, 2.0],
        [0, 1, 0, 1.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])
    
    Tsc_goal = np.array([
        [0, -1, 0, -1.0],
        [1, 0, 0, 2.0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])
    
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    # Well-tuned gains
    Kp = np.diag([8, 8, 8, 8, 8, 8])
    Ki = np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"New task scenario completed - check {output_dir}/")
        
    return success


def run_best_scenario(output_dir="results/best"):
    """Run simulation with well-tuned gains for best performance.
    
    Args:
        output_dir: directory for output files
        
    Returns:
        success: boolean indicating if simulation completed
    """
    print("=== Running Best Performance Scenario ===")
    
    # Well-tuned gains for good performance with large initial errors
    Kp = np.diag([3, 3, 3, 3, 3, 3])  # Moderate proportional gains
    Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Small integral gains
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    config_log, error_log, success = run_capstone_simulation(
        Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, Kp, Ki, output_dir
    )
    
    if success:
        plot_error_results(error_log, output_dir)
        print(f"Best performance scenario completed - check {output_dir}/")
        
    return success


def analyze_simulation_results(result_dir):
    """Analyze simulation results and print performance metrics.
    
    Args:
        result_dir: directory containing Xerr_log.csv
    """
    import numpy as np
    
    error_file = os.path.join(result_dir, "Xerr_log.csv")
    if not os.path.exists(error_file):
        print(f"Error file not found: {error_file}")
        return
        
    error_log = np.loadtxt(error_file, delimiter=',')
    
    print(f"\n=== Analysis of {result_dir} ===")
    print(f"Simulation duration: {len(error_log) * 0.01:.2f} seconds")
    print(f"Total time steps: {len(error_log)}")
    
    # Error statistics
    angular_errors = error_log[:, :3]  # ωx, ωy, ωz
    linear_errors = error_log[:, 3:]   # vx, vy, vz
    
    angular_magnitude = np.linalg.norm(angular_errors, axis=1)
    linear_magnitude = np.linalg.norm(linear_errors, axis=1)
    
    print(f"\nAngular Error Statistics:")
    print(f"  Initial: {angular_magnitude[0]:.4f} rad")
    print(f"  Final:   {angular_magnitude[-1]:.4f} rad")
    print(f"  Max:     {np.max(angular_magnitude):.4f} rad")
    print(f"  RMS:     {np.sqrt(np.mean(angular_magnitude**2)):.4f} rad")
    
    print(f"\nLinear Error Statistics:")
    print(f"  Initial: {linear_magnitude[0]:.4f} m")
    print(f"  Final:   {linear_magnitude[-1]:.4f} m")
    print(f"  Max:     {np.max(linear_magnitude):.4f} m")
    print(f"  RMS:     {np.sqrt(np.mean(linear_magnitude**2)):.4f} m")
    
    # Convergence analysis
    final_10_percent = int(len(error_log) * 0.9)
    steady_state_angular = np.mean(angular_magnitude[final_10_percent:])
    steady_state_linear = np.mean(linear_magnitude[final_10_percent:])
    
    print(f"\nSteady-State Performance (final 10%):")
    print(f"  Angular error: {steady_state_angular:.4f} rad")
    print(f"  Linear error:  {steady_state_linear:.4f} m")
    
    # Performance rating
    if steady_state_linear < 0.01 and steady_state_angular < 0.05:
        rating = "Excellent"
    elif steady_state_linear < 0.05 and steady_state_angular < 0.1:
        rating = "Good"
    elif steady_state_linear < 0.1 and steady_state_angular < 0.2:
        rating = "Fair"
    else:
        rating = "Poor"
        
    print(f"\nPerformance Rating: {rating}")


def compare_scenarios():
    """Compare results from multiple scenarios."""
    scenarios = ["results/best", "results/overshoot", "results/newTask"]
    
    print("=== Scenario Comparison ===")
    for scenario in scenarios:
        if os.path.exists(os.path.join(scenario, "Xerr_log.csv")):
            analyze_simulation_results(scenario)
        else:
            print(f"\nScenario {scenario}: No results found")


def create_gain_study(output_base="results/gain_study"):
    """Run multiple simulations with different gains for tuning study.
    
    Args:
        output_base: base directory for gain study results
    """
    print("=== Running Gain Study ===")
    
    # Define gain combinations to test
    Kp_values = [2, 5, 8, 12]
    Ki_values = [0, 0.1, 0.5, 1.0]
    
    Tsc_init, Tsc_goal = create_default_cube_poses()
    Tce_grasp, Tce_standoff = create_grasp_transforms()
    
    results = []
    
    for kp in Kp_values:
        for ki in Ki_values:
            print(f"Testing Kp={kp}, Ki={ki} ... ", end="", flush=True)
            
            # Create gain matrices
            Kp_matrix = np.diag([kp, kp, kp, kp, kp, kp])
            Ki_matrix = np.diag([ki, ki, ki, ki, ki, ki])
            
            # Output directory for this combination
            output_dir = os.path.join(output_base, f"Kp{kp}_Ki{ki}")
            
            try:
                config_log, error_log, success = run_capstone_simulation(
                    Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, 
                    Kp_matrix, Ki_matrix, output_dir
                )
                
                if success:
                    # Calculate performance metrics
                    final_angular = np.linalg.norm(error_log[-1, :3])
                    final_linear = np.linalg.norm(error_log[-1, 3:])
                    max_angular = np.max(np.linalg.norm(error_log[:, :3], axis=1))
                    max_linear = np.max(np.linalg.norm(error_log[:, 3:], axis=1))
                    
                    results.append({
                        'Kp': kp, 'Ki': ki,
                        'final_angular': final_angular,
                        'final_linear': final_linear,
                        'max_angular': max_angular,
                        'max_linear': max_linear,
                        'success': True
                    })
                    print("Success")
                else:
                    results.append({'Kp': kp, 'Ki': ki, 'success': False})
                    print("Failed")
                    
            except Exception as e:
                results.append({'Kp': kp, 'Ki': ki, 'success': False, 'error': str(e)})
                print(f"Error: {e}")
    
    # Save results summary
    summary_file = os.path.join(output_base, "gain_study_summary.txt")
    os.makedirs(output_base, exist_ok=True)
    
    with open(summary_file, 'w') as f:
        f.write("Gain Study Results\n")
        f.write("==================\n\n")
        f.write("Kp\tKi\tSuccess\tFinal_Ang\tFinal_Lin\tMax_Ang\tMax_Lin\n")
        
        for result in results:
            if result['success']:
                f.write(f"{result['Kp']}\t{result['Ki']}\tYes\t"
                       f"{result['final_angular']:.4f}\t{result['final_linear']:.4f}\t"
                       f"{result['max_angular']:.4f}\t{result['max_linear']:.4f}\n")
            else:
                f.write(f"{result['Kp']}\t{result['Ki']}\tNo\t-\t-\t-\t-\n")
    
    print(f"\nGain study completed - results in {output_base}/")
    print(f"Summary saved to {summary_file}")
    
    return results


if __name__ == "__main__":
    """Run all scenarios when called directly."""
    print("=== Modern Robotics Capstone - All Scenarios ===")
    
    # Run all three main scenarios
    scenarios = [
        ("Best Performance", run_best_scenario),
        ("Overshoot", run_overshoot_scenario), 
        ("New Task", run_newTask_scenario)
    ]
    
    for name, scenario_func in scenarios:
        print(f"\n{name} Scenario:")
        try:
            success = scenario_func()
            if success:
                print(f"✓ {name} completed successfully")
            else:
                print(f"✗ {name} failed")
        except Exception as e:
            print(f"✗ {name} error: {e}")
    
    # Compare all results
    print("\n" + "="*50)
    compare_scenarios()
