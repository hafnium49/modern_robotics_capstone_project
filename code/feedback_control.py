"""
Milestone 3: Feed-Forward + PI Task-Space Control

This module implements the FeedbackControl function for mobile manipulator control
according to the Modern Robotics capstone project specifications.
"""

import numpy as np
import modern_robotics as mr


# Fixed constants from Milestone 3 specification
R = 0.0475  # wheel radius (m)
L = 0.235   # half front-back wheel separation (m)
W = 0.150   # half side-side wheel separation (m)
DT = 0.01   # controller sample time (s)
SPEED_LIMIT = 12.3  # rad/s
PINV_TOLERANCE = 1e-3

# youBot transformation matrices (from spec)
TB0 = np.array([
    [1, 0, 0, 0.1662],
    [0, 1, 0, 0],
    [0, 0, 1, 0.0026],
    [0, 0, 0, 1]
])

M0E = np.array([
    [1, 0, 0, 0.033],
    [0, 1, 0, 0],
    [0, 0, 1, 0.6546],
    [0, 0, 0, 1]
])

# Body screw axes for the 5 joints (expressed in {e})
BLIST = np.array([
    [0, 0, 1, 0, 0.033, 0],
    [0, -1, 0, -0.5076, 0, 0],
    [0, -1, 0, -0.3526, 0, 0],
    [0, -1, 0, -0.2176, 0, 0],
    [0, 0, 1, 0, 0, 0]
]).T

# Pre-computed constant matrices (cached for performance)
INV_TB0 = np.linalg.inv(TB0)

# Integral error bounds for anti-windup (rad for orientation, m for position)
INTEGRAL_BOUNDS = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])

# Joint limits for youBot arm (in radians)
# Based on youbot-ros-pkg repository CKinematics.cpp isReachable() function:
# Joint limits for youBot arm (approximate, radians)
# Values taken from CKinematics.cpp isReachable() in the youbot-ros-pkg
# repository. These symmetric ranges work well for simulation and testing.
JOINT_LIMITS_MIN = np.array([-2.95, -1.57, -2.635, -1.78, -2.92])
JOINT_LIMITS_MAX = np.array([2.95, 1.57, 2.635, 1.78, 2.92])

# Conservative joint limits to avoid singularities and self-collisions
# As suggested in the document, constrain joints 3 and 4 to be less than -0.2 rad
# Joint 3 already has upper limit of -0.016, so we make it more conservative at -0.2
# Joint 4 minimum raised to avoid singularities near zero
JOINT_LIMITS_CONSERVATIVE_MIN = np.array([-2.95, -1.57, -2.635, 0.2, -2.92])
JOINT_LIMITS_CONSERVATIVE_MAX = np.array([2.95, 1.57, -0.2, 1.78, 2.92])


def _ensure_gain_matrix(K, default=None):
    """Return a 6x6 diagonal gain matrix.

    Accepts scalars, length-6 vectors, or 6x6 matrices. If ``K`` is ``None``
    the ``default`` value is returned. This utility makes the controller
    robust to different gain specifications.
    """
    if K is None:
        return default

    K = np.array(K, dtype=float)

    if K.ndim == 0:
        return np.eye(6) * float(K)

    if K.ndim == 1:
        if K.size != 6:
            raise ValueError("Gain vector must have length 6")
        return np.diag(K)

    if K.shape != (6, 6):
        raise ValueError("Gain matrix must be 6x6")

    return K


def checkJointLimits(theta, use_conservative_limits=False):
    """Test if joint angles violate joint limits.
    
    This function returns a list of joint limits that are violated
    given the robot arm's configuration θ.
    
    Args:
        theta: 5-element array of joint angles (radians)
        use_conservative_limits: If True, use conservative limits to avoid 
                               singularities and self-collisions
    
    Returns:
        violated_joints: List of joint indices (0-4) that violate limits
    """
    if use_conservative_limits:
        theta_min = JOINT_LIMITS_CONSERVATIVE_MIN
        theta_max = JOINT_LIMITS_CONSERVATIVE_MAX
    else:
        theta_min = JOINT_LIMITS_MIN
        theta_max = JOINT_LIMITS_MAX
    
    violated_joints = []
    
    for i in range(len(theta)):
        if theta[i] < theta_min[i] or theta[i] > theta_max[i]:
            violated_joints.append(i)
    
    return violated_joints


def enforceJointLimits(theta, use_conservative_limits=False):
    """Enforce joint limits by clamping joint angles.
    
    Args:
        theta: 5-element array of joint angles (radians)
        use_conservative_limits: If True, use conservative limits
    
    Returns:
        theta_limited: Joint angles clamped to limits
        violated_joints: List of joint indices that were clamped
    """
    if use_conservative_limits:
        theta_min = JOINT_LIMITS_CONSERVATIVE_MIN
        theta_max = JOINT_LIMITS_CONSERVATIVE_MAX
    else:
        theta_min = JOINT_LIMITS_MIN
        theta_max = JOINT_LIMITS_MAX
    
    theta_limited = np.clip(theta, theta_min, theta_max)
    violated_joints = []
    
    for i in range(len(theta)):
        if not np.isclose(theta[i], theta_limited[i]):
            violated_joints.append(i)
    
    return theta_limited, violated_joints


def modifyJacobianForLimits(Je, violated_joints):
    """Modify Jacobian to prevent motion of joints at limits.
    
    As described in the document, set columns corresponding to 
    offending joints to all zeros so the pseudoinverse will not
    request motion from these joints.
    
    Args:
        Je: 6x9 Jacobian matrix
        violated_joints: List of joint indices (0-4) that are at limits
    
    Returns:
        Je_modified: Modified Jacobian with zero columns for limited joints
    """
    Je_modified = Je.copy()
    
    # Joints are columns 4-8 in the 6x9 Jacobian (after the 4 wheel columns)
    for joint_idx in violated_joints:
        jacobian_col = 4 + joint_idx  # Offset by 4 wheel columns
        Je_modified[:, jacobian_col] = 0.0
    
    return Je_modified


def chassis_to_se3(phi, x, y):
    """Convert chassis configuration to SE(3) transformation matrix.
    
    Args:
        phi: chassis orientation (rad)
        x: x position (m)  
        y: y position (m)
        
    Returns:
        4x4 SE(3) transformation matrix
    """
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    
    return np.array([
        [cos_phi, -sin_phi, 0, x],
        [sin_phi,  cos_phi, 0, y],
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ])


def get_F6():
    """Compute the 6x4 F6 matrix mapping wheel rates to chassis twist.
    
    Wheel numbering convention: front-left=1, front-right=2, rear-left=3, rear-right=4
    This corresponds to the textbook's u2, u1, u3, u4.
    
    Returns:
        6x4 F6 matrix
    """
    # F maps [u_fl, u_fr, u_rl, u_rr] to [ωz, vx, vy]
    # This is the textbook F matrix with the first two columns swapped
    # to match our wheel order convention (fl, fr, rl, rr).
    lw = L + W
    F = (R / 4.0) * np.array([
        [-1 / lw, 1 / lw, 1 / lw, -1 / lw],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])
    
    # Embed in 6D: map wheel speeds to [ωx, ωy, ωz, vx, vy, vz]
    F6 = np.zeros((6, 4))
    F6[2, :] = F[0, :]  # ωz row
    F6[3, :] = F[1, :]  # vx row  
    F6[4, :] = F[2, :]  # vy row
    
    return F6


# Pre-compute F6 matrix at import time for performance
F6_MATRIX = get_F6()


def compute_jacobian(config):
    """Compute the 6x9 mobile manipulator Jacobian Je.
    
    Args:
        config: 12-element configuration [phi, x, y, theta1-5, w1-4]
        
    Returns:
        6x9 Jacobian matrix Je
    """
    phi, x, y = config[0], config[1], config[2]
    theta = config[3:8]  # joint angles
    
    # Forward kinematics
    Tsb = chassis_to_se3(phi, x, y)
    T0e = mr.FKinBody(M0E, BLIST, theta)
    
    # Base Jacobian columns (6x4) - using pre-computed constants
    J_base = mr.Adjoint(np.linalg.inv(T0e) @ INV_TB0) @ F6_MATRIX
    
    # Arm Jacobian columns (6x5) 
    J_arm = mr.JacobianBody(BLIST, theta)
    
    # Combine base and arm Jacobians
    Je = np.hstack([J_base, J_arm])
    
    return Je


def FeedbackControlWithJointLimits(X_actual, X_desired, X_desired_next, Kp, Ki, dt,
                                   integral_error_prev, config, use_conservative_limits=False):
    """Feed-forward + PI control with joint limits enforcement.
    
    This enhanced version checks if the computed controls would violate joint limits
    at the next time step and modifies the Jacobian accordingly.
    
    Args:
        X_actual: 4x4 SE(3) matrix - current end-effector pose
        X_desired: 4x4 - reference pose at time step i
        X_desired_next: 4x4 - reference pose at time step i+1
        Kp: 6x6 diagonal proportional gain matrix
        Ki: 6x6 diagonal integral gain matrix
        dt: scalar time step (seconds)
        integral_error_prev: 6-vector carried over from last call
        config: 12-element configuration [phi, x, y, theta1-5, w1-4]
        use_conservative_limits: If True, use conservative joint limits
        
    Returns:
        V_cmd: 6-vector - commanded twist in frame {e}
        controls: 9-vector - [u1 u2 u3 u4 θ̇1 … θ̇5]
        X_err: 6-vector - twist that takes X_actual to X_desired
        integral_error_new: 6-vector - updated ∫X_err dt
        joint_limits_info: dict with joint limits information
    """
    
    # Convert gains to 6x6 matrices in case scalars or vectors were provided
    Kp = _ensure_gain_matrix(Kp, np.eye(6))
    Ki = _ensure_gain_matrix(Ki, np.zeros((6, 6)))

    # 1. Feed-forward twist
    Vd = (1/dt) * mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X_desired) @ X_desired_next))
    
    # 2. Configuration error (twist)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X_actual) @ X_desired))
    
    # 3. Integral update with anti-windup guard
    integral_error_new = integral_error_prev + X_err * dt
    integral_error_new = np.clip(integral_error_new, -INTEGRAL_BOUNDS, INTEGRAL_BOUNDS)
    
    # 4. Commanded twist (body frame)
    V_cmd = (mr.Adjoint(np.linalg.inv(X_actual) @ X_desired) @ Vd + 
             Kp @ X_err + 
             Ki @ integral_error_new)
    
    # 5. Initial Jacobian and controls calculation
    Je = compute_jacobian(config)
    controls_raw = np.linalg.pinv(Je, rcond=PINV_TOLERANCE) @ V_cmd
    
    # 6. Check for joint limit violations
    current_theta = config[3:8]  # Current joint angles
    theta_dot = controls_raw[4:9]  # Joint velocities
    predicted_theta = current_theta + theta_dot * dt  # Predicted joint angles
    
    # Test if predicted configuration violates limits
    violated_joints = checkJointLimits(predicted_theta, use_conservative_limits)
    
    joint_limits_info = {
        'current_theta': current_theta.copy(),
        'predicted_theta': predicted_theta.copy(),
        'violated_joints': violated_joints.copy(),
        'limits_enforced': len(violated_joints) > 0
    }
    
    # 7. Modify Jacobian if limits would be violated
    if len(violated_joints) > 0:
        Je_modified = modifyJacobianForLimits(Je, violated_joints)
        controls_raw = np.linalg.pinv(Je_modified, rcond=PINV_TOLERANCE) @ V_cmd
        joint_limits_info['jacobian_modified'] = True
    else:
        joint_limits_info['jacobian_modified'] = False
    
    # 8. Apply speed limits
    controls = np.clip(controls_raw, -SPEED_LIMIT, SPEED_LIMIT)
    
    return V_cmd, controls, X_err, integral_error_new, joint_limits_info


def FeedbackControl(X_actual, X_desired, X_desired_next, Kp, Ki, dt, integral_error_prev, config=None):
    """Feed-forward + PI task-space control for mobile manipulator.
    
    Args:
        X_actual: 4x4 SE(3) matrix - current end-effector pose
        X_desired: 4x4 - reference pose at time step i
        X_desired_next: 4x4 - reference pose at time step i+1 (Δt later)
        Kp: 6x6 diagonal proportional gain matrix
        Ki: 6x6 diagonal integral gain matrix
        dt: scalar time step (seconds)
        integral_error_prev: 6-vector carried over from last call
        config: 12-element configuration [phi, x, y, theta1-5, w1-4] 
                (optional, defaults to zeros - only for unit tests)
        
    Returns:
        V_cmd: 6-vector - commanded twist in frame {e}
        controls: 9-vector - [u1 u2 u3 u4 θ̇1 … θ̇5]
        X_err: 6-vector - twist that takes X_actual to X_desired
        integral_error_new: 6-vector - updated ∫X_err dt
    """
    
    # Convert gains to 6x6 matrices in case scalars or vectors were provided
    Kp = _ensure_gain_matrix(Kp, np.eye(6))
    Ki = _ensure_gain_matrix(Ki, np.zeros((6, 6)))

    # 1. Feed-forward twist
    Vd = (1/dt) * mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X_desired) @ X_desired_next))
    
    # 2. Configuration error (twist)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X_actual) @ X_desired))
    
    # 3. Integral update with anti-windup guard
    integral_error_new = integral_error_prev + X_err * dt
    integral_error_new = np.clip(integral_error_new, -INTEGRAL_BOUNDS, INTEGRAL_BOUNDS)
    
    # 4. Commanded twist (body frame)
    V_cmd = (mr.Adjoint(np.linalg.inv(X_actual) @ X_desired) @ Vd + 
             Kp @ X_err + 
             Ki @ integral_error_new)
    
    # 5. Wheel + joint rates
    if config is None:
        config = np.zeros(12)  # Default configuration for testing only
    
    Je = compute_jacobian(config)
    controls_raw = np.linalg.pinv(Je, rcond=PINV_TOLERANCE) @ V_cmd
    controls = np.clip(controls_raw, -SPEED_LIMIT, SPEED_LIMIT)
    
    return V_cmd, controls, X_err, integral_error_new


class FeedbackController:
    """Stateful feedback controller for mobile manipulator.
    
    This class maintains the integral error state between control cycles,
    making it easier to use in simulation loops.
    """
    
    def __init__(self, Kp=None, Ki=None):
        """Initialize the controller with gain matrices.
        
        Args:
            Kp: 6x6 proportional gain matrix (default: 5*I)
            Ki: 6x6 integral gain matrix (default: 0*I)
        """
        if Kp is None:
            Kp = np.diag([5, 5, 5, 5, 5, 5])
        if Ki is None:
            Ki = np.diag([0, 0, 0, 0, 0, 0])

        self.Kp = _ensure_gain_matrix(Kp, np.eye(6))
        self.Ki = _ensure_gain_matrix(Ki, np.zeros((6, 6)))
        self.integral_error = np.zeros(6)
        
    def reset_integral(self):
        """Reset the integral error accumulator."""
        self.integral_error = np.zeros(6)
        
    def control(self, X_actual, X_desired, X_desired_next, config, dt=DT):
        """Compute control commands for one time step.
        
        Args:
            X_actual: 4x4 SE(3) matrix - current end-effector pose
            X_desired: 4x4 - reference pose at time step i
            X_desired_next: 4x4 - reference pose at time step i+1
            config: 12-element configuration [phi, x, y, theta1-5, w1-4]
            dt: time step (default: 0.01s)
            
        Returns:
            V_cmd: 6-vector - commanded twist in frame {e}
            controls: 9-vector - [u1 u2 u3 u4 θ̇1 … θ̇5]
            X_err: 6-vector - twist error
        """
        V_cmd, controls, X_err, self.integral_error = FeedbackControl(
            X_actual, X_desired, X_desired_next, 
            self.Kp, self.Ki, dt, self.integral_error, config
        )
        
        return V_cmd, controls, X_err


if __name__ == "__main__":
    # Example usage
    print("Milestone 3 FeedbackControl module")
    print("==================================")
    
    # Create a simple test
    X_actual = np.eye(4)
    X_desired = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    X_desired_next = X_desired
    
    # Default gains
    Kp = np.diag([5, 5, 5, 5, 5, 5])
    Ki = np.diag([0, 0, 0, 0, 0, 0])
    
    # Test function call
    V_cmd, controls, X_err, integral_error = FeedbackControl(
        X_actual, X_desired, X_desired_next, Kp, Ki, DT, np.zeros(6)
    )
    
    print(f"V_cmd: {V_cmd}")
    print(f"controls: {controls}")
    print(f"X_err: {X_err}")
    
    # Test stateful controller
    controller = FeedbackController()
    V_cmd2, controls2, X_err2 = controller.control(
        X_actual, X_desired, X_desired_next, np.zeros(12)
    )
    
    print(f"\nStateful controller results:")
    print(f"V_cmd: {V_cmd2}")
    print(f"controls: {controls2}")
    print(f"X_err: {X_err2}")
