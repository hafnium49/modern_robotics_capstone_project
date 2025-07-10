"""
Milestone 3 Demonstration: Feed-Forward + PI Task-Space Control

This script demonstrates how to use the complete Milestone 3 implementation
for mobile manipulator control with trajectory following.
"""

import numpy as np
import matplotlib.pyplot as plt
from modern_robotics_sim.feedback_control import FeedbackController, FeedbackControl
from modern_robotics_sim.trajectory_generator import TrajectoryGenerator
from modern_robotics_sim.next_state import NextState


def create_simple_trajectory():
    """Create a simple straight-line trajectory for demonstration."""
    # Define poses
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
    
    # Generate trajectory
    trajectory = TrajectoryGenerator(
        T_se_init=T_se_init,
        T_sc_init=T_sc_init,
        T_sc_goal=T_sc_goal,
        T_ce_grasp=T_ce_grasp,
        T_ce_standoff=T_ce_standoff,
        k=1
    )
    
    return trajectory


def simulate_control_loop(trajectory, duration_seconds=1.0):
    """Simulate a control loop following the given trajectory."""
    
    dt = 0.01  # 10ms control loop
    max_steps = int(duration_seconds / dt)
    max_steps = min(max_steps, len(trajectory) - 1)
    
    # Initial robot configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Create controller with PI gains
    Kp = np.diag([5, 5, 5, 5, 5, 5])
    Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Small integral gain
    controller = FeedbackController(Kp=Kp, Ki=Ki)
    
    # Storage for results
    results = {
        'time': [],
        'config': [],
        'V_cmd': [],
        'controls': [], 
        'X_err': [],
        'X_desired': [],
        'X_actual': []
    }
    
    print(f"Simulating {max_steps} control steps ({duration_seconds:.2f}s)")
    print("Step | Time | X_err_norm | V_cmd_norm | Config_xy")
    print("-" * 60)
    
    for step in range(max_steps):
        t = step * dt
        
        # Extract desired poses from trajectory  
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        # Compute current end-effector pose from configuration
        # For this demo, we'll use a simplified X_actual (in practice, this comes from forward kinematics)
        X_actual = X_desired.copy()
        # Add some small tracking error
        X_actual[:3, 3] += 0.01 * np.random.randn(3) if step > 0 else 0
        
        # Compute control commands
        V_cmd, controls, X_err = controller.control(
            X_actual, X_desired, X_desired_next, config, dt
        )
        
        # Apply controls to robot (NextState simulation)
        new_config = NextState(config, controls, dt, 12.3)  # speed limit = 12.3 rad/s
        
        # Store results
        results['time'].append(t)
        results['config'].append(config.copy())
        results['V_cmd'].append(V_cmd.copy())
        results['controls'].append(controls.copy())
        results['X_err'].append(X_err.copy())
        results['X_desired'].append(X_desired.copy())
        results['X_actual'].append(X_actual.copy())
        
        # Update configuration
        config = new_config
        
        # Print progress every 10 steps
        if step % 10 == 0:
            X_err_norm = np.linalg.norm(X_err)
            V_cmd_norm = np.linalg.norm(V_cmd)
            print(f"{step:4d} | {t:4.2f} | {X_err_norm:8.4f} | {V_cmd_norm:8.4f} | ({config[1]:6.3f}, {config[2]:6.3f})")
    
    print("-" * 60)
    print("Simulation complete!")
    
    return results


def plot_results(results):
    """Plot the simulation results."""
    time = np.array(results['time'])
    configs = np.array(results['config'])
    V_cmds = np.array(results['V_cmd'])
    X_errs = np.array(results['X_err'])
    controls = np.array(results['controls'])
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Milestone 3 Control Simulation Results', fontsize=16)
    
    # Plot 1: Chassis trajectory
    ax1 = axes[0, 0]
    ax1.plot(configs[:, 1], configs[:, 2], 'b-', linewidth=2, label='Chassis trajectory')
    ax1.scatter(configs[0, 1], configs[0, 2], color='green', s=100, label='Start', zorder=5)
    ax1.scatter(configs[-1, 1], configs[-1, 2], color='red', s=100, label='End', zorder=5)
    ax1.set_xlabel('X position (m)')
    ax1.set_ylabel('Y position (m)')
    ax1.set_title('Chassis Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # Plot 2: Control commands
    ax2 = axes[0, 1]
    wheel_commands = controls[:, :4]
    for i in range(4):
        ax2.plot(time, wheel_commands[:, i], label=f'Wheel {i+1}')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Wheel speed (rad/s)')
    ax2.set_title('Wheel Commands')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot 3: Task-space error
    ax3 = axes[1, 0]
    error_norms = [np.linalg.norm(err) for err in X_errs]
    ax3.plot(time, error_norms, 'r-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('||X_err|| (m, rad)')
    ax3.set_title('Task-Space Error Magnitude')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Commanded twist
    ax4 = axes[1, 1]
    V_cmd_norms = [np.linalg.norm(V) for V in V_cmds]
    ax4.plot(time, V_cmd_norms, 'g-', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('||V_cmd|| (m/s, rad/s)')
    ax4.set_title('Commanded Twist Magnitude')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def demonstrate_gain_effects():
    """Demonstrate the effect of different control gains."""
    print("\n" + "="*60)
    print("Demonstrating the effect of different control gains")
    print("="*60)
    
    # Simple reference poses
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired  # No motion
    config = np.zeros(12)
    
    gain_sets = [
        ("Low Kp", np.diag([1, 1, 1, 1, 1, 1]), np.diag([0, 0, 0, 0, 0, 0])),
        ("High Kp", np.diag([10, 10, 10, 10, 10, 10]), np.diag([0, 0, 0, 0, 0, 0])),
        ("Kp + Ki", np.diag([5, 5, 5, 5, 5, 5]), np.diag([1, 1, 1, 1, 1, 1])),
    ]
    
    print("Gain Set    | X_err_norm | V_cmd_norm | Controls_norm")
    print("-" * 55)
    
    for name, Kp, Ki in gain_sets:
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, 0.01, np.zeros(6), config
        )
        
        X_err_norm = np.linalg.norm(X_err)
        V_cmd_norm = np.linalg.norm(V_cmd)
        controls_norm = np.linalg.norm(controls)
        
        print(f"{name:10s} | {X_err_norm:8.4f} | {V_cmd_norm:8.4f} | {controls_norm:10.4f}")


if __name__ == "__main__":
    print("Milestone 3 Demonstration: Feed-Forward + PI Control")
    print("=" * 60)
    
    # Generate a trajectory
    print("Generating reference trajectory...")
    trajectory = create_simple_trajectory()
    print(f"Generated trajectory with {len(trajectory)} time steps")
    
    # Simulate control
    print("\nSimulating control loop...")
    results = simulate_control_loop(trajectory, duration_seconds=0.5)
    
    # Demonstrate gain effects
    demonstrate_gain_effects()
    
    # Plot results (if matplotlib is available)
    try:
        plot_results(results)
        print("\nPlot displayed - close the plot window to continue")
    except ImportError:
        print("\nMatplotlib not available - skipping plots")
    except Exception as e:
        print(f"\nPlotting error: {e}")
    
    print("\nDemonstration complete!")
    print("The FeedbackControl implementation is ready for Milestone 3 testing.")
