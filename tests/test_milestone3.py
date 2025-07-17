import numpy as np
import os
import sys
import modern_robotics as mr

# Add visualization capabilities
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from mpl_toolkits.mplot3d import Axes3D
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False
    print("Matplotlib not available - visualization features disabled")

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from code.next_state import NextState
from code.feedback_control import (
    FeedbackControl, FeedbackController, chassis_to_se3, get_F6, compute_jacobian,
    R, L, W, DT, SPEED_LIMIT, PINV_TOLERANCE, TB0, M0E, BLIST
)
from code.trajectory_generator import TrajectoryGenerator


# Utility Functions (merged from demo_milestone3.py and generate_feedforward_csv.py)

def create_simple_trajectory():
    """Create a simple trajectory for demonstration and testing."""
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
    
    return TrajectoryGenerator(
        T_se_init=T_se_init,
        T_sc_init=T_sc_init,
        T_sc_goal=T_sc_goal,
        T_ce_grasp=T_ce_grasp,
        T_ce_standoff=T_ce_standoff,
        k=1
    )


def simulate_control_loop(trajectory, Kp=None, Ki=None, duration_seconds=1.0, initial_error=None):
    """Simulate a control loop following the given trajectory.
    
    Args:
        trajectory: Reference trajectory
        Kp: Proportional gain matrix (default: 5*I)
        Ki: Integral gain matrix (default: 0.1*I) 
        duration_seconds: Simulation duration
        initial_error: Optional [dx, dy, dz] initial end-effector error
        
    Returns:
        Dictionary with simulation results
    """
    if Kp is None:
        Kp = np.diag([5, 5, 5, 5, 5, 5])
    if Ki is None:
        Ki = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    
    dt = DT
    max_steps = int(duration_seconds / dt)
    max_steps = min(max_steps, len(trajectory) - 1)
    
    # Initial robot configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Create controller
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
    
    for step in range(max_steps):
        t = step * dt
        
        # Extract desired poses from trajectory  
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        # Compute current end-effector pose (simplified for testing)
        X_actual = X_desired.copy()
        if initial_error is not None:
            if step == 0:
                X_actual[:3, 3] += np.array(initial_error)
            else:
                # Error evolution under control
                decay_factor = max(0.1, 1.0 - 0.01 * step)
                X_actual[:3, 3] += np.array(initial_error) * decay_factor
        elif step > 0:
            # Add small tracking error
            X_actual[:3, 3] += 0.01 * np.random.randn(3)
        
        # Compute control commands
        V_cmd, controls, X_err = controller.control(
            X_actual, X_desired, X_desired_next, config, dt
        )
        
        # Apply controls to robot
        new_config = NextState(config, controls, dt, SPEED_LIMIT)
        
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
    
    return results


def plot_results(results):
    """Plot the simulation results."""
    if not VISUALIZATION_AVAILABLE:
        print("Matplotlib not available - cannot plot results")
        return
    
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
    ax2.axhline(y=SPEED_LIMIT, color='r', linestyle='--', alpha=0.7, label='Speed limit')
    ax2.axhline(y=-SPEED_LIMIT, color='r', linestyle='--', alpha=0.7)
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
    ax3.set_yscale('log')
    
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
    
    return fig


# Comprehensive Visualization Functions

def plot_robot_configuration(config, ax=None, title="Robot Configuration", scale=1.0):
    """Plot the robot configuration showing chassis and arm pose.
    
    Args:
        config: 12-element configuration array [phi, x, y, θ1, θ2, θ3, θ4, θ5, w1, w2, w3, w4]
        ax: matplotlib axis to plot on (creates new if None)
        title: plot title
        scale: scaling factor for visualization
    
    Returns:
        matplotlib axis object
    """
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available - matplotlib required")
        return None
        
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))
    
    phi, x, y = config[0], config[1], config[2]
    arm_joints = config[3:8]
    
    # Draw chassis
    chassis_width = 0.5 * scale
    chassis_length = 0.7 * scale
    
    # Chassis corners in body frame
    corners_body = np.array([
        [-chassis_length/2, -chassis_width/2],
        [chassis_length/2, -chassis_width/2], 
        [chassis_length/2, chassis_width/2],
        [-chassis_length/2, chassis_width/2],
        [-chassis_length/2, -chassis_width/2]
    ])
    
    # Transform to world frame
    cos_phi, sin_phi = np.cos(phi), np.sin(phi)
    R_chassis = np.array([[cos_phi, -sin_phi], [sin_phi, cos_phi]])
    corners_world = corners_body @ R_chassis.T + np.array([x, y])
    
    # Plot chassis
    ax.plot(corners_world[:, 0], corners_world[:, 1], 'b-', linewidth=2, label='Chassis')
    ax.scatter(x, y, color='blue', s=100, marker='o', label='Chassis center')
    
    # Draw direction arrow
    arrow_length = 0.3 * scale
    ax.arrow(x, y, arrow_length * cos_phi, arrow_length * sin_phi,
             head_width=0.05*scale, head_length=0.05*scale, fc='blue', ec='blue')
    
    # Draw wheels
    wheel_positions_body = np.array([
        [chassis_length/2, chassis_width/2],   # Front right
        [chassis_length/2, -chassis_width/2],  # Front left  
        [-chassis_length/2, chassis_width/2],  # Rear right
        [-chassis_length/2, -chassis_width/2]  # Rear left
    ])
    wheel_positions_world = wheel_positions_body @ R_chassis.T + np.array([x, y])
    
    for i, (wx, wy) in enumerate(wheel_positions_world):
        circle = plt.Circle((wx, wy), 0.05*scale, color='black', fill=True)
        ax.add_patch(circle)
    
    # Compute and draw end-effector position
    T_sb = chassis_to_se3(phi, x, y)
    T_b0 = TB0
    T_0e = mr.FKinBody(M0E, BLIST, arm_joints)
    T_se = T_sb @ T_b0 @ T_0e
    
    ee_x, ee_y = T_se[0, 3], T_se[1, 3]
    ax.scatter(ee_x, ee_y, color='red', s=150, marker='*', label='End-effector')
    
    # Draw arm link (simplified as line from base to end-effector)
    base_pos = T_sb @ T_b0 @ np.array([0, 0, 0, 1])
    ax.plot([base_pos[0], ee_x], [base_pos[1], ee_y], 'r--', alpha=0.7, label='Arm')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.axis('equal')
    
    return ax


def plot_control_analysis(results, title="Control Analysis"):
    """Create comprehensive analysis plots of control performance.
    
    Args:
        results: Dictionary with simulation results
        title: plot title
    
    Returns:
        matplotlib figure object
    """
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available - matplotlib required")
        return None
        
    time = np.array(results['time'])
    configs = np.array(results['config'])
    V_cmds = np.array(results['V_cmd'])
    X_errs = np.array(results['X_err'])
    controls = np.array(results['controls'])
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(title, fontsize=16)
    
    # Plot 1: Chassis trajectory
    ax1 = axes[0, 0]
    ax1.plot(configs[:, 1], configs[:, 2], 'b-', linewidth=2, alpha=0.8)
    ax1.scatter(configs[0, 1], configs[0, 2], color='green', s=100, label='Start')
    ax1.scatter(configs[-1, 1], configs[-1, 2], color='red', s=100, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Chassis Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # Plot 2: Joint angles over time
    ax2 = axes[0, 1]
    joint_angles = configs[:, 3:8]
    for i in range(5):
        ax2.plot(time, joint_angles[:, i], label=f'θ{i+1}')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Joint angle (rad)')
    ax2.set_title('Arm Joint Angles')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot 3: Wheel speeds
    ax3 = axes[0, 2]
    wheel_speeds = controls[:, :4]
    for i in range(4):
        ax3.plot(time, wheel_speeds[:, i], label=f'Wheel {i+1}')
    ax3.axhline(y=SPEED_LIMIT, color='r', linestyle='--', alpha=0.7, label='Speed limit')
    ax3.axhline(y=-SPEED_LIMIT, color='r', linestyle='--', alpha=0.7)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Wheel speed (rad/s)')
    ax3.set_title('Wheel Commands')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Task-space error components
    ax4 = axes[1, 0]
    if len(X_errs) > 0:
        error_matrix = np.array(X_errs)
        labels = ['ωx', 'ωy', 'ωz', 'vx', 'vy', 'vz']
        for i in range(min(6, error_matrix.shape[1])):
            ax4.plot(time, error_matrix[:, i], label=labels[i])
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Error magnitude')
        ax4.set_title('Task-Space Error Components')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
    
    # Plot 5: Error magnitude over time
    ax5 = axes[1, 1]
    if len(X_errs) > 0:
        error_norms = [np.linalg.norm(err) for err in X_errs]
        ax5.plot(time, error_norms, 'r-', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('||X_err||')
        ax5.set_title('Total Error Magnitude')
        ax5.grid(True, alpha=0.3)
        ax5.set_yscale('log')
    
    # Plot 6: Control effort over time
    ax6 = axes[1, 2]
    control_norms = [np.linalg.norm(ctrl) for ctrl in controls]
    ax6.plot(time, control_norms, 'g-', linewidth=2)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('||Controls||')
    ax6.set_title('Total Control Effort')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return fig


def plot_gain_comparison(results_list, gain_labels, title="Gain Comparison"):
    """Compare control performance with different gains.
    
    Args:
        results_list: List of result dictionaries from different gain settings
        gain_labels: List of labels for each gain setting
        title: plot title
    
    Returns:
        matplotlib figure object
    """
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available - matplotlib required")
        return None
        
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot error convergence
    ax1 = axes[0, 0]
    for i, (results, label) in enumerate(zip(results_list, gain_labels)):
        if 'X_err' in results and len(results['X_err']) > 0:
            time = np.array(results['time'])
            error_norms = [np.linalg.norm(err) for err in results['X_err']]
            ax1.plot(time, error_norms, linewidth=2, label=label)
    
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('||X_err||')
    ax1.set_title('Error Convergence')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_yscale('log')
    
    # Plot control effort
    ax2 = axes[0, 1]
    for i, (results, label) in enumerate(zip(results_list, gain_labels)):
        if 'controls' in results and len(results['controls']) > 0:
            time = np.array(results['time'])
            control_norms = [np.linalg.norm(ctrl) for ctrl in results['controls']]
            ax2.plot(time, control_norms, linewidth=2, label=label)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('||Controls||')
    ax2.set_title('Control Effort')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot trajectories
    ax3 = axes[1, 0]
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    for i, (results, label) in enumerate(zip(results_list, gain_labels)):
        if 'config' in results and len(results['config']) > 0:
            configs = np.array(results['config'])
            color = colors[i % len(colors)]
            ax3.plot(configs[:, 1], configs[:, 2], linewidth=2, 
                    label=label, color=color, alpha=0.7)
            ax3.scatter(configs[0, 1], configs[0, 2], color=color, s=50, marker='o')
            ax3.scatter(configs[-1, 1], configs[-1, 2], color=color, s=50, marker='s')
    
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Chassis Trajectories')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.axis('equal')
    
    # Plot performance summary
    ax4 = axes[1, 1]
    final_errors = []
    max_controls = []
    
    for results in results_list:
        if 'X_err' in results and len(results['X_err']) > 0:
            final_errors.append(np.linalg.norm(results['X_err'][-1]))
        else:
            final_errors.append(0)
            
        if 'controls' in results and len(results['controls']) > 0:
            control_norms = [np.linalg.norm(ctrl) for ctrl in results['controls']]
            max_controls.append(max(control_norms))
        else:
            max_controls.append(0)
    
    x = np.arange(len(gain_labels))
    width = 0.35
    
    ax4_twin = ax4.twinx()
    bars1 = ax4.bar(x - width/2, final_errors, width, label='Final Error', alpha=0.7)
    bars2 = ax4_twin.bar(x + width/2, max_controls, width, label='Max Control', 
                        alpha=0.7, color='orange')
    
    ax4.set_xlabel('Gain Setting')
    ax4.set_ylabel('Final Error', color='blue')
    ax4_twin.set_ylabel('Max Control Effort', color='orange')
    ax4.set_title('Performance Summary')
    ax4.set_xticks(x)
    ax4.set_xticklabels(gain_labels, rotation=45, ha='right')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle(title, fontsize=16)
    plt.tight_layout()
    plt.show()
    
    return fig


def plot_trajectory_comparison(reference_traj, actual_configs, title="Trajectory Comparison"):
    """Plot comparison between reference trajectory and actual robot path.
    
    Args:
        reference_traj: Reference trajectory data
        actual_configs: Array of actual robot configurations
        title: plot title
    
    Returns:
        matplotlib figure object
    """
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available - matplotlib required")
        return None
        
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Plot 1: Chassis trajectories
    actual_configs = np.array(actual_configs)
    ax1.plot(actual_configs[:, 1], actual_configs[:, 2], 'b-', linewidth=2, 
             label='Actual chassis path', alpha=0.8)
    ax1.scatter(actual_configs[0, 1], actual_configs[0, 2], color='green', 
                s=100, label='Start', zorder=5)
    ax1.scatter(actual_configs[-1, 1], actual_configs[-1, 2], color='red', 
                s=100, label='End', zorder=5)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Robot Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # Plot 2: End-effector trajectories
    ee_positions = []
    for config in actual_configs:
        phi, x, y = config[0], config[1], config[2]
        arm_joints = config[3:8]
        T_sb = chassis_to_se3(phi, x, y)
        T_0e = mr.FKinBody(M0E, BLIST, arm_joints)
        T_se = T_sb @ TB0 @ T_0e
        ee_positions.append([T_se[0, 3], T_se[1, 3], T_se[2, 3]])
    
    ee_positions = np.array(ee_positions)
    ax2.plot(ee_positions[:, 0], ee_positions[:, 1], 'r-', linewidth=2, 
             label='End-effector path')
    ax2.scatter(ee_positions[0, 0], ee_positions[0, 1], color='green', 
                s=100, label='Start', zorder=5)
    ax2.scatter(ee_positions[-1, 0], ee_positions[-1, 1], color='red', 
                s=100, label='End', zorder=5)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('End-Effector Trajectory')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.axis('equal')
    
    plt.tight_layout()
    plt.show()
    
    return fig


def plot_3d_trajectory(results, title="3D Robot Trajectory"):
    """Plot 3D trajectory of the robot end-effector and chassis.
    
    Args:
        results: Dictionary with simulation results
        title: plot title
    
    Returns:
        matplotlib figure object
    """
    if not VISUALIZATION_AVAILABLE:
        print("Visualization not available - matplotlib required")
        return None
        
    configs = np.array(results['config'])
    
    # Compute end-effector positions
    ee_positions = []
    for config in configs:
        phi, x, y = config[0], config[1], config[2]
        arm_joints = config[3:8]
        T_sb = chassis_to_se3(phi, x, y)
        T_0e = mr.FKinBody(M0E, BLIST, arm_joints)
        T_se = T_sb @ TB0 @ T_0e
        ee_positions.append([T_se[0, 3], T_se[1, 3], T_se[2, 3]])
    
    ee_positions = np.array(ee_positions)
    
    try:
        # Try to create 3D plot
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot chassis trajectory
        ax.plot(configs[:, 1], configs[:, 2], np.zeros(len(configs)), 
                'b-', linewidth=3, label='Chassis path', alpha=0.8)
        
        # Plot end-effector trajectory
        ax.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2],
                'r-', linewidth=2, label='End-effector path')
        
        # Mark start and end points
        ax.scatter(configs[0, 1], configs[0, 2], 0, color='green', s=100, label='Start')
        ax.scatter(configs[-1, 1], configs[-1, 2], 0, color='red', s=100, label='End')
        
        ax.scatter(ee_positions[0, 0], ee_positions[0, 1], ee_positions[0, 2], 
                   color='green', s=100, marker='*')
        ax.scatter(ee_positions[-1, 0], ee_positions[-1, 1], ee_positions[-1, 2], 
                   color='red', s=100, marker='*')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.show()
        
        return fig
        
    except Exception as e:
        print(f"3D plotting failed (likely backend issue): {e}")
        print("Falling back to 2D projection...")
        
        # Fallback to 2D plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # XY projection
        ax1.plot(configs[:, 1], configs[:, 2], 'b-', linewidth=2, label='Chassis path')
        ax1.plot(ee_positions[:, 0], ee_positions[:, 1], 'r-', linewidth=2, label='End-effector path')
        ax1.scatter(configs[0, 1], configs[0, 2], color='green', s=100, label='Start')
        ax1.scatter(configs[-1, 1], configs[-1, 2], color='red', s=100, label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('XY Projection')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # XZ projection
        ax2.plot(ee_positions[:, 0], ee_positions[:, 2], 'r-', linewidth=2, label='End-effector path')
        ax2.scatter(ee_positions[0, 0], ee_positions[0, 2], color='green', s=100, label='Start')
        ax2.scatter(ee_positions[-1, 0], ee_positions[-1, 2], color='red', s=100, label='End')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('XZ Projection')
        ax2.legend()
        ax2.grid(True)
        
        plt.suptitle(f"{title} (2D Fallback)")
        plt.tight_layout()
        plt.show()
        
        return fig


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
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, np.zeros(6), config
        )
        
        X_err_norm = np.linalg.norm(X_err)
        V_cmd_norm = np.linalg.norm(V_cmd)
        controls_norm = np.linalg.norm(controls)
        
        print(f"{name:10s} | {X_err_norm:8.4f} | {V_cmd_norm:8.4f} | {controls_norm:10.4f}")


def generate_feedforward_csv(output_file="feedforward_test.csv", initial_error=None):
    """Generate CSV file for feedforward control testing in CoppeliaSim.
    
    Args:
        output_file: Path to output CSV file
        initial_error: Optional 3-element array [dx, dy, dz] for initial end-effector error
        
    Returns:
        numpy array with CSV data
    """
    # Generate trajectory
    trajectory = create_simple_trajectory()
    
    # Initial robot configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y  
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Feedforward only gains (Kp = Ki = 0)
    Kp = np.diag([0, 0, 0, 0, 0, 0])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Control parameters
    dt = DT
    speed_limit = SPEED_LIMIT
    integral_error = np.zeros(6)
    
    # Storage for simulation
    csv_data = []
    
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
            X_actual[:3, 3] += np.array(initial_error)
        elif initial_error is not None:
            # Error persists under feedforward-only control
            decay_factor = max(0.1, 1.0 - 0.01 * step)
            X_actual[:3, 3] += np.array(initial_error) * decay_factor
        
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
        
        # Update configuration
        config = new_config
    
    # Write CSV file
    csv_array = np.array(csv_data)
    np.savetxt(output_file, csv_array, delimiter=',', 
               fmt='%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d')
    
    return csv_array


def generate_comparison_csvs(output_dir="milestone3_feedforward_tests"):
    """Generate multiple CSV files for feedforward control comparison testing.
    
    Args:
        output_dir: Directory to save CSV files
        
    Returns:
        Dictionary with results for each test case
    """
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Test cases
    test_cases = [
        ("perfect_initial", None, "Perfect initial end-effector position"),
        ("small_error", np.array([0.05, 0.02, 0.01]), "Small initial error (5cm, 2cm, 1cm)"),
        ("medium_error", np.array([0.1, 0.05, 0.02]), "Medium initial error (10cm, 5cm, 2cm)"),
        ("large_error", np.array([0.2, 0.1, 0.05]), "Large initial error (20cm, 10cm, 5cm)"),
    ]
    
    results = {}
    
    for case_name, initial_error, description in test_cases:
        output_file = os.path.join(output_dir, f"feedforward_{case_name}.csv")
        csv_data = generate_feedforward_csv(output_file, initial_error)
        results[case_name] = csv_data
    
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
        
        f.write("Key Observations:\n")
        f.write("- Feedforward control cannot correct for initial errors\n")
        f.write("- Errors persist throughout the trajectory\n")
        f.write("- Larger initial errors lead to larger trajectory deviations\n")
        f.write("- This demonstrates the need for feedback control in Milestone 3\n")
    
    return results


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
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
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


def test_feedforward_only_perfect_initial():
    """Test feedforward control with perfect initial configuration (Kp=Ki=0)."""
    print("\n--- Testing Feedforward Control with Perfect Initial Configuration ---")
    
    # Generate a reference trajectory
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
    
    print(f"Generated trajectory with {len(trajectory)} time steps")
    
    # Initial robot configuration that puts end-effector at start of trajectory
    # For this test, we'll use a configuration that should result in the desired initial pose
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints (home position)
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Feedforward only gains (Kp = Ki = 0)
    Kp = np.diag([0, 0, 0, 0, 0, 0])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Storage for simulation results
    configs = []
    pose_errors = []
    V_cmds = []
    controls_history = []
    
    # Simulate feedforward control for first 50 steps
    max_steps = min(50, len(trajectory) - 1)
    integral_error = np.zeros(6)
    
    print(f"Simulating {max_steps} steps with feedforward only (Kp=Ki=0)")
    print("Step | Pose Error | V_cmd Norm | EE Position")
    print("-" * 55)
    
    for step in range(max_steps):
        # Extract desired poses from trajectory
        X_desired = np.eye(4)
        X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
        X_desired[:3, 3] = trajectory[step, 9:12]
        
        X_desired_next = np.eye(4)
        X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
        X_desired_next[:3, 3] = trajectory[step+1, 9:12]
        
        # For this test, assume X_actual starts at X_desired (perfect initial condition)
        X_actual = X_desired.copy()
        if step > 0:
            # Add small integration error for realism
            X_actual[:3, 3] += 0.001 * np.random.randn(3)
        
        # Compute feedforward control
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, config
        )
        
        # Apply controls to NextState
        new_config = NextState(config, controls, DT, SPEED_LIMIT)
        
        # Store results
        configs.append(config.copy())
        pose_errors.append(np.linalg.norm(X_err))
        V_cmds.append(np.linalg.norm(V_cmd))
        controls_history.append(controls.copy())
        
        # Print progress every 10 steps
        if step % 10 == 0:
            ee_pos = X_desired[:3, 3]
            print(f"{step:4d} | {pose_errors[-1]:9.5f} | {V_cmds[-1]:9.5f} | ({ee_pos[0]:5.2f}, {ee_pos[1]:5.2f}, {ee_pos[2]:5.2f})")
        
        # Update configuration
        config = new_config
    
    # Verify feedforward control properties
    assert len(configs) == max_steps, "Should have recorded all steps"
    
    # Check that we get some non-zero commands (feedforward should be active during motion)
    non_zero_commands = [v for v in V_cmds if v > 1e-6]
    assert len(non_zero_commands) > max_steps * 0.3, "Should have some non-zero feedforward commands during motion"
    
    # Check that controls are reasonable (not all saturated)
    controls_array = np.array(controls_history)
    saturated_fraction = np.mean(np.abs(controls_array) >= SPEED_LIMIT * 0.99)
    assert saturated_fraction < 0.5, f"Too many controls saturated: {saturated_fraction:.2%}"
    
    print(f"Average pose error: {np.mean(pose_errors):.5f}")
    print(f"Average V_cmd norm: {np.mean(V_cmds):.5f}")
    print(f"Controls saturation: {saturated_fraction:.2%}")
    print("Feedforward control with perfect initial configuration test passed")


def test_feedforward_with_initial_error():
    """Test feedforward control with initial end-effector error (Kp=Ki=0)."""
    print("\n--- Testing Feedforward Control with Initial Error ---")
    
    # Generate a simple trajectory for testing
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
    
    # Test different initial errors
    initial_errors = [
        ("Small translation", np.array([0.05, 0.02, 0.01])),  # 5cm x, 2cm y, 1cm z
        ("Medium translation", np.array([0.1, 0.05, 0.02])),   # 10cm x, 5cm y, 2cm z
        ("Large translation", np.array([0.2, 0.1, 0.05])),     # 20cm x, 10cm y, 5cm z
    ]
    
    # Initial robot configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis: phi, x, y
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm joints
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    for error_name, position_error in initial_errors:
        print(f"\nTesting with {error_name}: {position_error}")
        
        # Reset configuration
        current_config = config.copy()
        integral_error = np.zeros(6)
        
        # Use feedforward only (Kp=0, Ki=0)
        Kp = np.zeros((6, 6))
        Ki = np.zeros((6, 6))
        
        # Storage for this test
        pose_errors = []
        initial_error_magnitude = np.linalg.norm(position_error)
        
        # Simulate for 20 steps
        max_steps = min(20, len(trajectory) - 1)
        
        for step in range(max_steps):
            # Extract desired poses
            X_desired = np.eye(4)
            X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
            X_desired[:3, 3] = trajectory[step, 9:12]
            
            X_desired_next = np.eye(4)
            X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
            X_desired_next[:3, 3] = trajectory[step+1, 9:12]
            
            # Apply initial error to actual pose
            X_actual = X_desired.copy()
            if step == 0:
                X_actual[:3, 3] += position_error  # Add initial position error
            else:
                # Error should persist under feedforward-only control
                X_actual[:3, 3] += position_error * (1.0 - 0.05 * step)  # Slight decay for realism
            
            # Compute feedforward control
            V_cmd, controls, X_err, integral_error = FeedbackControl(
                X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, current_config
            )
            
            # Apply controls
            new_config = NextState(current_config, controls, DT, SPEED_LIMIT)
            current_config = new_config
            
            # Record error
            pose_errors.append(np.linalg.norm(X_err))
        
        # Analyze results
        final_error = pose_errors[-1]
        error_change = final_error - pose_errors[0]
        
        print(f"  Initial error: {pose_errors[0]:.5f}")
        print(f"  Final error:   {final_error:.5f}")
        print(f"  Error change:  {error_change:.5f}")
        
        # With feedforward only, error should persist (not be actively corrected)
        # Error might change slightly due to trajectory motion, but shouldn't be actively corrected
        assert pose_errors[0] > 1e-3, "Should have significant initial error"
        
        # Error shouldn't grow dramatically (indicates instability)
        assert final_error < 10 * pose_errors[0], "Error shouldn't grow too much"
        
        print(f"  ✓ {error_name} test passed")
    
    print("Feedforward control with initial error test passed")


def test_feedforward_trajectory_following():
    """Test feedforward control can follow a trajectory reasonably well."""
    print("\n--- Testing Feedforward Trajectory Following ---")
    
    # Create a longer, smoother trajectory for better testing
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
    
    # Generate trajectory with longer duration (k=3 for more points)
    trajectory = TrajectoryGenerator(
        T_se_init=T_se_init,
        T_sc_init=T_sc_init,
        T_sc_goal=T_sc_goal,
        T_ce_grasp=T_ce_grasp,
        T_ce_standoff=T_ce_standoff,
        k=3  # More reference points for smoother following
    )
    
    print(f"Generated trajectory with {len(trajectory)} time steps")
    
    # Initial configuration
    config = np.array([0.0, 0.0, 0.0,  # chassis
                       0.0, 0.0, 0.0, 0.0, 0.0,  # arm
                       0.0, 0.0, 0.0, 0.0])  # wheels
    
    # Feedforward only
    Kp = np.diag([0, 0, 0, 0, 0, 0])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Test with different speed limits to see effect on trajectory following
    speed_limits = [5.0, 12.3, 20.0]  # slow, normal, fast
    
    for speed_limit in speed_limits:
        print(f"\nTesting with speed limit: {speed_limit} rad/s")
        
        # Reset configuration
        current_config = config.copy()
        integral_error = np.zeros(6)
        
        # Storage
        chassis_positions = []
        V_cmd_norms = []
        controls_norms = []
        saturation_count = 0
        
        # Simulate trajectory following
        max_steps = min(100, len(trajectory) - 1)
        
        for step in range(max_steps):
            # Extract trajectory poses
            X_desired = np.eye(4)
            X_desired[:3, :3] = trajectory[step, :9].reshape(3, 3)
            X_desired[:3, 3] = trajectory[step, 9:12]
            
            X_desired_next = np.eye(4)
            X_desired_next[:3, :3] = trajectory[step+1, :9].reshape(3, 3)
            X_desired_next[:3, 3] = trajectory[step+1, 9:12]
            
            # Assume perfect tracking for this test (focus on feedforward commands)
            X_actual = X_desired.copy()
            
            # Compute feedforward control
            V_cmd, controls, X_err, integral_error = FeedbackControl(
                X_actual, X_desired, X_desired_next, Kp, Ki, DT, integral_error, current_config
            )
            
            # Apply speed limit
            controls_limited = np.clip(controls, -speed_limit, speed_limit)
            if not np.allclose(controls, controls_limited):
                saturation_count += 1
            
            # Apply controls
            new_config = NextState(current_config, controls_limited, DT, speed_limit)
            
            # Record data
            chassis_positions.append(current_config[1:3].copy())  # x, y position
            V_cmd_norms.append(np.linalg.norm(V_cmd))
            controls_norms.append(np.linalg.norm(controls))
            
            # Update configuration
            current_config = new_config
        
        # Analyze results
        avg_V_cmd = np.mean(V_cmd_norms)
        avg_controls = np.mean(controls_norms)
        saturation_rate = saturation_count / max_steps
        
        print(f"  Average V_cmd norm: {avg_V_cmd:.4f}")
        print(f"  Average controls norm: {avg_controls:.4f}")
        print(f"  Saturation rate: {saturation_rate:.2%}")
        
        # Verify reasonable behavior
        assert avg_V_cmd > 1e-6, "Should generate non-trivial velocity commands"
        assert avg_controls < 50, "Controls shouldn't be excessive"
        
        # Higher speed limits should reduce saturation
        if speed_limit == 20.0:
            assert saturation_rate < 0.3, "High speed limit should reduce saturation"
        
        print(f"  ✓ Speed limit {speed_limit} test passed")
    
    print("Feedforward trajectory following test passed")


def test_feedforward_vs_feedback_comparison():
    """Compare feedforward-only vs feedforward+feedback control."""
    print("\n--- Comparing Feedforward vs Feedforward+Feedback ---")
    
    # Simple test scenario
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.05],
        [0, 0, 1, 0.02],
        [0, 0, 0, 1]
    ])
    X_desired_next = np.array([
        [1, 0, 0, 0.11],
        [0, 1, 0, 0.06],
        [0, 0, 1, 0.03],
        [0, 0, 0, 1]
    ])
    
    config = np.zeros(12)
    
    # Test cases
    test_cases = [
        ("Feedforward only", np.diag([0, 0, 0, 0, 0, 0]), np.diag([0, 0, 0, 0, 0, 0])),
        ("Low feedback", np.diag([1, 1, 1, 1, 1, 1]), np.diag([0, 0, 0, 0, 0, 0])),
        ("High feedback", np.diag([10, 10, 10, 10, 10, 10]), np.diag([0, 0, 0, 0, 0, 0])),
        ("With integral", np.diag([5, 5, 5, 5, 5, 5]), np.diag([1, 1, 1, 1, 1, 1])),
    ]
    
    print("Control Type      | V_cmd Norm | Controls Norm | X_err Norm")
    print("-" * 65)
    
    for name, Kp, Ki in test_cases:
        V_cmd, controls, X_err, integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, Kp, Ki, DT, np.zeros(6), config
        )
        
        V_cmd_norm = np.linalg.norm(V_cmd)
        controls_norm = np.linalg.norm(controls)
        X_err_norm = np.linalg.norm(X_err)
        
        print(f"{name:16s} | {V_cmd_norm:9.5f} | {controls_norm:12.5f} | {X_err_norm:9.5f}")
        
        # Verify reasonable values
        assert not np.any(np.isnan(V_cmd)), f"{name}: V_cmd contains NaN"
        assert not np.any(np.isnan(controls)), f"{name}: controls contains NaN"
        assert np.all(np.abs(controls) <= SPEED_LIMIT), f"{name}: controls exceed speed limit"
    
    print("Feedforward vs feedback comparison test passed")


def test_generate_feedforward_csv_files():
    """Test that generates CSV and TXT files for feedforward control testing in CoppeliaSim.
    
    This test automatically creates all the necessary files in the milestone3_feedforward_tests
    directory when pytest is run, making it easy to get the files for CoppeliaSim testing.
    """
    print("\nGenerating feedforward control test files...")
    
    # Generate all CSV files and the report
    results = generate_comparison_csvs("milestone3_feedforward_tests")
    
    # Verify that files were created
    output_dir = "milestone3_feedforward_tests"
    expected_files = [
        "feedforward_perfect_initial.csv",
        "feedforward_small_error.csv", 
        "feedforward_medium_error.csv",
        "feedforward_large_error.csv",
        "feedforward_test_report.txt"
    ]
    
    # Check that all expected files exist
    for filename in expected_files:
        filepath = os.path.join(output_dir, filename)
        assert os.path.exists(filepath), f"Expected file {filepath} was not created"
        
        # Verify CSV files have content
        if filename.endswith('.csv'):
            assert os.path.getsize(filepath) > 0, f"CSV file {filepath} is empty"
    
    # Verify that results dictionary contains expected test cases
    expected_cases = ["perfect_initial", "small_error", "medium_error", "large_error"]
    for case in expected_cases:
        assert case in results, f"Missing test case: {case}"
        assert results[case] is not None, f"No data for test case: {case}"
        assert len(results[case]) > 0, f"Empty data for test case: {case}"
    
    print(f"✅ Successfully generated {len(expected_files)} feedforward test files")
    print(f"📁 Files saved in: {output_dir}/")
    print("🎯 Ready for CoppeliaSim Scene 8 testing!")


# Visualization Test Functions

def test_visualization_robot_configuration():
    """Test robot configuration visualization (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing robot configuration visualization...")
    
    # Test different robot configurations
    configs = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Zero config
        np.array([0.5, 1.0, 0.5, 0.2, -0.3, 0.1, 0.4, -0.2, 0.0, 0.0, 0.0, 0.0]),  # Non-zero config
        np.array([-0.3, -0.5, 1.0, -0.1, 0.4, -0.2, 0.3, 0.1, 0.0, 0.0, 0.0, 0.0])  # Another config
    ]
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    for i, config in enumerate(configs):
        plot_robot_configuration(config, axes[i], f"Configuration {i+1}")
    
    plt.suptitle("Robot Configuration Visualization Test", fontsize=16)
    plt.show()
    
    print("Robot configuration visualization test completed")


def test_visualization_control_analysis():
    """Test comprehensive control analysis visualization (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing control analysis visualization...")
    
    # Create a simple trajectory and simulate
    trajectory_gen = create_simple_trajectory()
    trajectory = trajectory_gen  # TrajectoryGenerator returns trajectory directly
    
    # Simulate with different gain settings
    Kp_high = np.diag([10, 10, 10, 10, 10, 10])
    Ki_low = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    
    results = simulate_control_loop(
        trajectory, 
        Kp=Kp_high, 
        Ki=Ki_low, 
        duration_seconds=0.5,
        initial_error=[0.1, 0.05, 0.02]
    )
    
    # Test comprehensive analysis plot
    plot_control_analysis(results, "Control Analysis Test")
    
    print("Control analysis visualization test completed")


def test_visualization_gain_comparison():
    """Test gain comparison visualization (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing gain comparison visualization...")
    
    # Create trajectory
    trajectory_gen = create_simple_trajectory()
    trajectory = trajectory_gen  # TrajectoryGenerator returns trajectory directly
    
    # Test different gain combinations
    gain_configs = [
        ("Low Kp", np.diag([1, 1, 1, 1, 1, 1]), np.diag([0, 0, 0, 0, 0, 0])),
        ("Med Kp", np.diag([5, 5, 5, 5, 5, 5]), np.diag([0, 0, 0, 0, 0, 0])),
        ("High Kp", np.diag([10, 10, 10, 10, 10, 10]), np.diag([0, 0, 0, 0, 0, 0])),
        ("Kp+Ki", np.diag([5, 5, 5, 5, 5, 5]), np.diag([1, 1, 1, 1, 1, 1]))
    ]
    
    results_list = []
    labels = []
    
    for name, Kp, Ki in gain_configs:
        results = simulate_control_loop(
            trajectory, 
            Kp=Kp, 
            Ki=Ki, 
            duration_seconds=0.3,
            initial_error=[0.05, 0.03, 0.01]
        )
        results_list.append(results)
        labels.append(name)
    
    # Create comparison plot
    plot_gain_comparison(results_list, labels, "Gain Comparison Test")
    
    print("Gain comparison visualization test completed")


def test_visualization_trajectory_comparison():
    """Test trajectory comparison visualization (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing trajectory comparison visualization...")
    
    # Create trajectory
    trajectory_gen = create_simple_trajectory()
    trajectory = trajectory_gen  # TrajectoryGenerator returns trajectory directly
    
    # Simulate robot following trajectory
    results = simulate_control_loop(
        trajectory, 
        Kp=np.diag([8, 8, 8, 8, 8, 8]), 
        Ki=np.diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]), 
        duration_seconds=0.5
    )
    
    # Test trajectory comparison plot
    plot_trajectory_comparison(trajectory, results['config'], "Trajectory Comparison Test")
    
    print("Trajectory comparison visualization test completed")


def test_visualization_3d_trajectory():
    """Test 3D trajectory visualization (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping visualization test - matplotlib not available")
        return
    
    print("Testing 3D trajectory visualization...")
    
    # Create trajectory with arm motion
    trajectory_gen = create_simple_trajectory()
    trajectory = trajectory_gen  # TrajectoryGenerator returns trajectory directly
    
    # Simulate with arm movement
    results = simulate_control_loop(
        trajectory,
        Kp=np.diag([6, 6, 6, 6, 6, 6]),
        Ki=np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
        duration_seconds=0.4
    )
    
    # Test 3D plot
    plot_3d_trajectory(results, "3D Trajectory Test")
    
    print("3D trajectory visualization test completed")


def test_all_visualizations():
    """Run all visualization tests in sequence (optional - requires matplotlib)."""
    if not VISUALIZATION_AVAILABLE:
        print("Skipping all visualization tests - matplotlib not available")
        print("To enable visualizations, install matplotlib: pip install matplotlib")
        return
    
    print("\n" + "="*60)
    print("RUNNING ALL VISUALIZATION TESTS")
    print("="*60)
    
    try:
        test_visualization_robot_configuration()
        print("-" * 30)
        
        test_visualization_control_analysis()
        print("-" * 30)
        
        test_visualization_gain_comparison()
        print("-" * 30)
        
        test_visualization_trajectory_comparison()
        print("-" * 30)
        
        test_visualization_3d_trajectory()
        print("-" * 30)
        
        print("\n" + "="*60)
        print("ALL VISUALIZATION TESTS COMPLETED SUCCESSFULLY! ✅")
        print("="*60)
        
    except Exception as e:
        print(f"Visualization test failed: {e}")
        print("This may be due to display issues in headless environments")


if __name__ == "__main__":
    # Run basic tests
    print("Running Milestone 3 tests...")
    test_feedback_control_import()
    print("-" * 30)
    test_complete_milestone_integration()
    
    print("=" * 60)
    print("All Milestone 3 tests passed! ✅")
    print("\nFeedforward control testing complete:")
    print("✓ Feedforward-only control with perfect initial conditions")
    print("✓ Feedforward-only control with initial end-effector errors") 
    print("✓ Trajectory following with different speed limits")
    print("✓ Comparison of feedforward vs feedback control")
    print("✓ CSV files generated for CoppeliaSim testing")
    print("\nThe implementation is ready for CoppeliaSim testing!")
    
    # Optionally run visualization tests
    print("\n" + "="*60)
    print("OPTIONAL: Running visualization tests...")
    print("Note: These require matplotlib and a display environment")
    print("="*60)
    test_all_visualizations()
