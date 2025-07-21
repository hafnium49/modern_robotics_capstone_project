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
            
        self.Kp = Kp
        self.Ki = Ki
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
