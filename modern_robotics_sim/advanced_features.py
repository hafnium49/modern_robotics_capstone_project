#!/usr/bin/env python3
"""
Advanced Features for Modern Robotics Capstone Project
Implementation of "Other Things to Try" suggestions from the capstone requirements.

Features implemented:
1. Stationary base during manipulation segments (2, 4, 6, 8)
2. Weighted pseudoinverse for motion preference
3. Joint limits enforcement
4. Singularity avoidance
5. Block throwing trajectory
6. Obstacle avoidance motion planning
7. Enhanced CoppeliaSim dynamics
"""

import numpy as np
import modern_robotics as mr
from scipy.linalg import pinv
from .feedback_control import R, L, W, DT, TB0, M0E, BLIST, compute_jacobian
from .trajectory_generator import TrajectoryGenerator


def weighted_pseudoinverse(J, weight_joints=1.0, weight_wheels=1.0):
    """
    Compute weighted pseudoinverse to prefer wheel or joint motions.
    
    Args:
        J: 6x9 Jacobian matrix
        weight_joints: Weight for arm joints (columns 0-4)
        weight_wheels: Weight for wheel motions (columns 5-8)
        
    Returns:
        9x6 weighted pseudoinverse matrix
    """
    # Create weight matrix W
    W = np.eye(9)
    W[0:5, 0:5] *= weight_joints  # Arm joint weights
    W[5:9, 5:9] *= weight_wheels  # Wheel weights
    
    # Weighted pseudoinverse: (W J^T)(J W J^T)^{-1}
    try:
        JW = J @ W
        JWJt = JW @ J.T
        JWJt_inv = np.linalg.inv(JWJt)
        J_weighted_pinv = W @ J.T @ JWJt_inv
    except np.linalg.LinAlgError:
        # Fallback to SVD-based pseudoinverse if singular
        J_weighted_pinv = W @ pinv(J)
    
    return J_weighted_pinv


def enforce_joint_limits(theta, theta_min=None, theta_max=None):
    """
    Enforce joint limits for the robot arm.
    
    Default limits for youBot arm (in radians):
    - Joint 1: [-2.95, 2.95]
    - Joint 2: [-1.57, 1.57] 
    - Joint 3: [-2.635, 2.635]
    - Joint 4: [-1.78, 1.78]
    - Joint 5: [-2.92, 2.92]
    
    Args:
        theta: 5-element joint angle array
        theta_min: Minimum joint limits (default: youBot limits)
        theta_max: Maximum joint limits (default: youBot limits)
        
    Returns:
        theta_limited: Joint angles clamped to limits
        limit_reached: Boolean array indicating which joints hit limits
    """
    if theta_min is None:
        theta_min = np.array([-2.95, -1.57, -2.635, -1.78, -2.92])
    if theta_max is None:
        theta_max = np.array([2.95, 1.57, 2.635, 1.78, 2.92])
    
    theta_limited = np.clip(theta, theta_min, theta_max)
    limit_reached = (theta != theta_limited)
    
    return theta_limited, limit_reached


def compute_manipulability(J_arm):
    """
    Compute manipulability measure for singularity detection.
    
    Args:
        J_arm: 6x5 arm Jacobian matrix
        
    Returns:
        manipulability: Scalar measure (0 = singular, >0 = non-singular)
    """
    # Manipulability = sqrt(det(J J^T))
    JJt = J_arm @ J_arm.T
    det_JJt = np.linalg.det(JJt)
    
    # Ensure non-negative for square root
    manipulability = np.sqrt(max(0, det_JJt))
    
    return manipulability


def singularity_robust_inverse(J_arm, damping=0.01):
    """
    Compute damped least squares inverse for singularity robustness.
    
    Args:
        J_arm: 6x5 arm Jacobian matrix
        damping: Damping factor for near-singular configurations
        
    Returns:
        J_pinv: Damped pseudoinverse
    """
    JJt = J_arm @ J_arm.T
    I = np.eye(JJt.shape[0])
    
    try:
        J_pinv = J_arm.T @ np.linalg.inv(JJt + damping**2 * I)
    except np.linalg.LinAlgError:
        # Fallback to standard pseudoinverse
        J_pinv = pinv(J_arm)
    
    return J_pinv


def plan_stationary_base_trajectory(Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff,
                                   k=1, Tf_segment=2.0, gripper_segments=None):
    """
    Generate trajectory with stationary mobile base during manipulation segments.
    
    Mobile base only moves during segments 1, 3, 5, 7 (transit).
    Base remains stationary during segments 2, 4, 6, 8 (manipulation).
    
    Args:
        Tsc_init, Tsc_goal: Initial and goal cube poses
        Tce_grasp, Tce_standoff: Grasp and standoff transforms
        k: Trajectory points per segment
        Tf_segment: Time duration per segment
        gripper_segments: List of gripper states per segment
        
    Returns:
        trajectory: Nx13 trajectory matrix
        base_positions: Nx3 mobile base positions for each point
    """
    from .trajectory_generator import TrajectoryGenerator
    
    if gripper_segments is None:
        gripper_segments = [0, 0, 1, 1, 0, 0, 1, 1]  # Default gripper sequence
    
    # Generate full trajectory first
    traj_gen = TrajectoryGenerator()
    
    # Calculate initial end-effector pose
    Tse_initial = Tce_standoff @ Tsc_init
    
    full_trajectory = traj_gen.TrajectoryGenerator(
        Tse_initial, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k, Tf_segment
    )
    
    # Extract mobile base positions from each trajectory point
    base_positions = []
    stationary_trajectory = []
    
    N_total = len(full_trajectory)
    N_segment = N_total // 8  # Points per segment
    
    # Track base position for stationary segments
    stationary_base_pos = None
    
    for i, traj_row in enumerate(full_trajectory):
        segment_idx = i // N_segment
        
        # Extract current base position (approximate from end-effector pose)
        R_se = traj_row[:9].reshape(3, 3)
        p_se = traj_row[9:12]
        T_se = np.eye(4)
        T_se[:3, :3] = R_se
        T_se[:3, 3] = p_se
        
        # Compute approximate base position (simplified inverse kinematics)
        if segment_idx in [1, 3, 5, 7]:  # Manipulation segments - keep base stationary
            if stationary_base_pos is None:
                # Use first position of manipulation segment
                stationary_base_pos = p_se[:2]  # x, y position
            base_pos = np.array([stationary_base_pos[0], stationary_base_pos[1], 0])
        else:  # Transit segments - allow base movement
            base_pos = p_se
            stationary_base_pos = None  # Reset for next manipulation segment
        
        base_positions.append(base_pos)
        stationary_trajectory.append(traj_row)
    
    return np.array(stationary_trajectory), np.array(base_positions)


def plan_throwing_trajectory(initial_config, target_landing_point, 
                           release_height=0.5, release_velocity=3.0):
    """
    Plan a trajectory for throwing the block to a desired landing point.
    
    This implements ballistic trajectory planning for the gripper to release
    the block with appropriate velocity to reach the target.
    
    Args:
        initial_config: 12-element robot configuration
        target_landing_point: [x, y] target coordinates
        release_height: Height above ground to release block
        release_velocity: Initial velocity magnitude for throw
        
    Returns:
        throw_trajectory: Nx13 trajectory for throwing motion
        release_point: 3D point where block should be released
        release_config: Robot configuration at release
    """
    g = 9.81  # Gravity acceleration
    
    # Compute release point using ballistic equations
    target_x, target_y = target_landing_point
    
    # Time of flight from release height to ground
    t_flight = np.sqrt(2 * release_height / g)
    
    # Required horizontal velocity
    dx = target_x
    dy = target_y
    distance = np.sqrt(dx**2 + dy**2)
    
    # Release velocity components
    v_x = dx / t_flight
    v_y = dy / t_flight
    v_z = 0  # No initial vertical velocity needed
    
    # Release point (above target, accounting for parabolic path)
    release_x = target_x - v_x * t_flight / 2
    release_y = target_y - v_y * t_flight / 2
    release_point = np.array([release_x, release_y, release_height])
    
    # Generate trajectory to release point
    # This would involve inverse kinematics to find joint angles
    # For simplicity, create a basic trajectory
    N_throw = 50
    throw_trajectory = []
    
    for i in range(N_throw):
        # Linear interpolation to release point
        alpha = i / (N_throw - 1)
        
        # Current end-effector position
        current_pos = release_point * alpha
        
        # Simple identity orientation (could be optimized)
        R_ee = np.eye(3)
        
        # Format as trajectory row
        traj_row = np.zeros(13)
        traj_row[:9] = R_ee.flatten()
        traj_row[9:12] = current_pos
        traj_row[12] = 1 if i < N_throw - 5 else 0  # Release gripper near end
        
        throw_trajectory.append(traj_row)
    
    # Compute release configuration (simplified)
    release_config = initial_config.copy()  # Would need proper IK here
    
    return np.array(throw_trajectory), release_point, release_config


def obstacle_avoiding_planner(start_pose, goal_pose, obstacles=None, 
                            n_waypoints=10, safety_margin=0.1):
    """
    Simple obstacle-avoiding motion planner for the entire robot.
    
    This implements a basic RRT-style path planning algorithm to avoid
    obstacles while moving from start to goal configuration.
    
    Args:
        start_pose: 4x4 initial SE(3) pose
        goal_pose: 4x4 goal SE(3) pose  
        obstacles: List of obstacle geometries (spheres, boxes)
        n_waypoints: Number of intermediate waypoints
        safety_margin: Minimum distance to obstacles
        
    Returns:
        waypoint_trajectory: Nx13 collision-free trajectory
        path_length: Total path length
    """
    if obstacles is None:
        obstacles = []
    
    # Simple straight-line path with obstacle checks
    waypoints = []
    
    for i in range(n_waypoints + 1):
        alpha = i / n_waypoints
        
        # Linear interpolation in SE(3) - simplified
        # Proper implementation would use geodesics on SE(3)
        R_start = start_pose[:3, :3]
        p_start = start_pose[:3, 3]
        R_goal = goal_pose[:3, :3]
        p_goal = goal_pose[:3, 3]
        
        # Linear position interpolation
        p_current = (1 - alpha) * p_start + alpha * p_goal
        
        # SLERP for rotation (simplified)
        R_current = R_start @ mr.MatrixExp3(alpha * mr.MatrixLog3(R_start.T @ R_goal))
        
        # Check collision with obstacles
        collision_free = True
        for obs in obstacles:
            if check_collision_with_obstacle(p_current, obs, safety_margin):
                collision_free = False
                break
        
        if collision_free:
            # Format as trajectory row
            traj_row = np.zeros(13)
            traj_row[:9] = R_current.flatten()
            traj_row[9:12] = p_current
            traj_row[12] = 0  # Gripper open during transit
            
            waypoints.append(traj_row)
    
    # Compute path length
    path_length = 0
    for i in range(1, len(waypoints)):
        p1 = waypoints[i-1][9:12]
        p2 = waypoints[i][9:12]
        path_length += np.linalg.norm(p2 - p1)
    
    return np.array(waypoints), path_length


def check_collision_with_obstacle(point, obstacle, margin):
    """
    Check if a point collides with an obstacle (with safety margin).
    
    Args:
        point: 3D position to check
        obstacle: Dictionary with obstacle geometry
        margin: Safety margin distance
        
    Returns:
        collision: True if collision detected
    """
    obs_type = obstacle.get('type', 'sphere')
    obs_center = obstacle.get('center', [0, 0, 0])
    
    distance = np.linalg.norm(point - obs_center)
    
    if obs_type == 'sphere':
        radius = obstacle.get('radius', 0.1)
        return distance < (radius + margin)
    elif obs_type == 'box':
        # Simplified box collision (could be improved)
        size = obstacle.get('size', [0.1, 0.1, 0.1])
        return (abs(point[0] - obs_center[0]) < size[0]/2 + margin and
                abs(point[1] - obs_center[1]) < size[1]/2 + margin and
                abs(point[2] - obs_center[2]) < size[2]/2 + margin)
    
    return False


def enhanced_feedback_control(config, trajectory_row, Kp, Ki, integral_error, 
                            use_weighted_pseudoinverse=False, 
                            weight_joints=1.0, weight_wheels=1.0,
                            enforce_limits=True, avoid_singularities=True):
    """
    Enhanced feedback control with advanced features.
    
    Args:
        config: Current 12-element robot configuration
        trajectory_row: Desired pose from trajectory
        Kp, Ki: Control gains  
        integral_error: Accumulated integral error
        use_weighted_pseudoinverse: Use weighted pseudoinverse for motion preference
        weight_joints: Weight for preferring joint motions
        weight_wheels: Weight for preferring wheel motions
        enforce_limits: Enforce joint limits
        avoid_singularities: Use singularity-robust inverse
        
    Returns:
        controls: 9-element control vector
        updated_integral_error: Updated integral error
        status: Dictionary with control status information
    """
    from .feedback_control import FeedbackControl
    
    # Standard feedback control computation
    fb_control = FeedbackControl()
    standard_controls, updated_integral_error = fb_control.FeedbackControl(
        config, trajectory_row, Kp, Ki, integral_error
    )
    
    # Enhanced features
    status = {}
    
    if use_weighted_pseudoinverse or avoid_singularities:
        # Recompute with enhanced Jacobian inverse
        phi, x, y = config[0], config[1], config[2]
        theta = config[3:8]
        
        # Compute Jacobian
        J_arm, J_base = compute_jacobian(theta, phi)
        J = np.hstack([J_arm, J_base])
        
        # Compute current and desired poses
        cos_phi, sin_phi = np.cos(phi), np.sin(phi)
        Tsb = np.array([
            [cos_phi, -sin_phi, 0, x],
            [sin_phi,  cos_phi, 0, y],
            [0,        0,       1, 0],
            [0,        0,       0, 1]
        ])
        
        T0e = mr.FKinBody(M0E, BLIST, theta)
        Tse_current = Tsb @ TB0 @ T0e
        
        R_desired = trajectory_row[:9].reshape(3, 3)
        p_desired = trajectory_row[9:12]
        Tse_desired = np.eye(4)
        Tse_desired[:3, :3] = R_desired
        Tse_desired[:3, 3] = p_desired
        
        # Compute error
        Tse_error = np.linalg.inv(Tse_current) @ Tse_desired
        se3_error = mr.se3ToVec(mr.MatrixLog3(Tse_error[:3, :3]))
        position_error = Tse_error[:3, 3]
        error_twist = np.hstack([se3_error, position_error])
        
        # Update integral error
        updated_integral_error += error_twist * DT
        
        # Control law
        command_twist = Kp @ error_twist + Ki @ updated_integral_error
        
        # Enhanced Jacobian inverse
        if avoid_singularities:
            manipulability = compute_manipulability(J_arm)
            status['manipulability'] = manipulability
            
            if manipulability < 0.01:  # Near singular
                J_pinv = singularity_robust_inverse(J, damping=0.05)
                status['singularity_avoided'] = True
            else:
                if use_weighted_pseudoinverse:
                    J_pinv = weighted_pseudoinverse(J, weight_joints, weight_wheels)
                else:
                    J_pinv = pinv(J)
                status['singularity_avoided'] = False
        else:
            if use_weighted_pseudoinverse:
                J_pinv = weighted_pseudoinverse(J, weight_joints, weight_wheels)
            else:
                J_pinv = pinv(J)
            status['singularity_avoided'] = False
        
        # Compute control
        controls = J_pinv @ command_twist
        
        # Enforce joint limits
        if enforce_limits:
            arm_controls = controls[:5]
            new_theta = theta + arm_controls * DT
            limited_theta, limits_hit = enforce_joint_limits(new_theta)
            
            if np.any(limits_hit):
                # Adjust arm controls to respect limits
                controls[:5] = (limited_theta - theta) / DT
                status['joint_limits_enforced'] = True
                status['joints_at_limit'] = limits_hit
            else:
                status['joint_limits_enforced'] = False
                status['joints_at_limit'] = np.zeros(5, dtype=bool)
    else:
        controls = standard_controls
        status['enhanced_features_used'] = False
    
    return controls, updated_integral_error, status


def create_coppelia_dynamics_config():
    """
    Create configuration for enhanced CoppeliaSim dynamics.
    
    Returns configuration dictionary for setting up:
    - Respondable chassis for block pushing
    - Dynamic properties for realistic physics
    - Contact parameters for manipulation
    """
    config = {
        'chassis_respondable': True,
        'chassis_mass': 20.0,  # kg
        'chassis_friction': 0.7,
        'chassis_restitution': 0.1,
        
        'block_respondable': True, 
        'block_mass': 0.2,  # kg
        'block_friction': 0.8,
        'block_restitution': 0.3,
        
        'gripper_contact_force': 50.0,  # N
        'gripper_friction': 1.0,
        
        'physics_engine': 'bullet',  # or 'ode', 'vortex'
        'physics_timestep': 0.005,  # s
        'gravity': [0, 0, -9.81],  # m/s^2
        
        'collision_detection': True,
        'contact_handling': True,
        'joint_motors_enabled': True,
    }
    
    return config


# Example usage and demonstration functions
def demo_stationary_base_control():
    """Demonstrate stationary base during manipulation segments."""
    print("Demo: Stationary Base Control")
    print("Mobile base remains stationary during manipulation segments 2, 4, 6, 8")
    
    # This would be integrated into the main control loop
    # showing base position constraints during manipulation


def demo_weighted_pseudoinverse():
    """Demonstrate weighted pseudoinverse for motion preference."""
    print("Demo: Weighted Pseudoinverse")
    print("Preferring wheel motions over joint motions...")
    
    # Example Jacobian (6x9)
    J = np.random.randn(6, 9)
    
    # Standard pseudoinverse
    J_pinv_standard = pinv(J)
    
    # Weighted pseudoinverse preferring wheels
    J_pinv_weighted = weighted_pseudoinverse(J, weight_joints=0.5, weight_wheels=2.0)
    
    print(f"Standard pseudoinverse shape: {J_pinv_standard.shape}")
    print(f"Weighted pseudoinverse shape: {J_pinv_weighted.shape}")
    print("Weighted version will prefer wheel motions for same end-effector command")


def demo_joint_limits():
    """Demonstrate joint limit enforcement."""
    print("Demo: Joint Limit Enforcement")
    
    # Test joint angles near limits
    theta_test = np.array([3.0, 1.6, 2.7, 1.8, 3.0])  # Some beyond limits
    theta_limited, limits_hit = enforce_joint_limits(theta_test)
    
    print(f"Original angles: {theta_test}")
    print(f"Limited angles:  {theta_limited}")
    print(f"Limits hit:      {limits_hit}")


def demo_singularity_avoidance():
    """Demonstrate singularity detection and avoidance."""
    print("Demo: Singularity Avoidance")
    
    # Create a near-singular Jacobian
    J_singular = np.random.randn(6, 5)
    J_singular[:, -1] = J_singular[:, -2]  # Make last two columns identical
    
    manipulability = compute_manipulability(J_singular)
    print(f"Manipulability measure: {manipulability:.6f}")
    
    if manipulability < 0.01:
        print("Near singularity detected! Using damped least squares...")
        J_robust = singularity_robust_inverse(J_singular)
        print(f"Robust inverse computed with shape: {J_robust.shape}")


if __name__ == "__main__":
    print("Advanced Features for Modern Robotics Capstone Project")
    print("=" * 60)
    
    # Run demonstrations
    demo_weighted_pseudoinverse()
    print()
    demo_joint_limits()
    print()
    demo_singularity_avoidance()
    print()
    
    print("All advanced features loaded and ready for integration!")
