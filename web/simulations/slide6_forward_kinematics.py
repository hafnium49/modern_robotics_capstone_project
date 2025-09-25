#!/usr/bin/env python3
"""
Slide 6: Forward Kinematics - Product of Exponentials (PoE)
Demonstrates how screw axes compose to create robot motion
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
import matplotlib.patches as patches

def skew(v):
    """Convert 3D vector to skew-symmetric matrix"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def exp_twist(S, theta):
    """
    Matrix exponential for twist
    S = [omega, v] where omega is unit rotation axis, v is linear component
    Returns 4x4 transformation matrix
    """
    omega = S[:3]
    v = S[3:]
    
    if np.linalg.norm(omega) < 1e-6:
        # Pure translation
        T = np.eye(4)
        T[:3, 3] = v * theta
        return T
    
    # Rotation with translation (screw motion)
    omega_skew = skew(omega)
    omega_norm = np.linalg.norm(omega)
    omega_hat = omega / omega_norm
    omega_hat_skew = skew(omega_hat)
    
    theta_norm = theta * omega_norm
    
    # Rodrigues' formula for rotation
    R = np.eye(3) + np.sin(theta_norm) * omega_hat_skew + \
        (1 - np.cos(theta_norm)) * omega_hat_skew @ omega_hat_skew
    
    # Translation component
    G = np.eye(3) * theta_norm + (1 - np.cos(theta_norm)) * omega_hat_skew + \
        (theta_norm - np.sin(theta_norm)) * omega_hat_skew @ omega_hat_skew
    p = G @ (v / omega_norm)
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    
    return T

class ThreeJointRobot:
    """3R planar robot for PoE demonstration"""
    
    def __init__(self):
        # Link lengths
        self.L1 = 1.0
        self.L2 = 0.8
        self.L3 = 0.6
        
        # Home configuration (all joints at 0)
        # End-effector at (L1+L2+L3, 0)
        self.M = np.array([
            [1, 0, 0, self.L1 + self.L2 + self.L3],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Screw axes at home configuration
        # All joints rotate around z-axis
        # Joint 1 at origin
        self.S1 = np.array([0, 0, 1, 0, 0, 0])
        
        # Joint 2 at (L1, 0, 0)
        q2 = np.array([self.L1, 0, 0])
        omega2 = np.array([0, 0, 1])
        v2 = -np.cross(omega2, q2)
        self.S2 = np.concatenate([omega2, v2])
        
        # Joint 3 at (L1+L2, 0, 0)
        q3 = np.array([self.L1 + self.L2, 0, 0])
        omega3 = np.array([0, 0, 1])
        v3 = -np.cross(omega3, q3)
        self.S3 = np.concatenate([omega3, v3])
        
    def forward_kinematics(self, theta):
        """
        Compute forward kinematics using PoE
        theta: [theta1, theta2, theta3]
        Returns: end-effector transformation matrix and intermediate transforms
        """
        # Compute exponentials
        exp_S1_theta1 = exp_twist(self.S1, theta[0])
        exp_S2_theta2 = exp_twist(self.S2, theta[1])
        exp_S3_theta3 = exp_twist(self.S3, theta[2])
        
        # Product of exponentials
        T = exp_S1_theta1 @ exp_S2_theta2 @ exp_S3_theta3 @ self.M
        
        # Also compute intermediate joint positions for visualization
        T1 = exp_S1_theta1 @ np.array([[1, 0, 0, self.L1],
                                       [0, 1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]])
        
        T2 = exp_S1_theta1 @ exp_S2_theta2 @ np.array([[1, 0, 0, self.L1 + self.L2],
                                                        [0, 1, 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]])
        
        return T, T1, T2
    
    def get_link_positions(self, theta):
        """Get all joint and end-effector positions"""
        T, T1, T2 = self.forward_kinematics(theta)
        
        positions = {
            'base': np.array([0, 0]),
            'joint1': T1[:2, 3],
            'joint2': T2[:2, 3],
            'end': T[:2, 3]
        }
        
        return positions

def main():
    # Create robot
    robot = ThreeJointRobot()
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Main robot visualization
    ax1 = fig.add_subplot(221)
    ax1.set_title('Robot Workspace', fontsize=14, fontweight='bold')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_xlim([-3, 3])
    ax1.set_ylim([-3, 3])
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    # Screw axis visualization
    ax2 = fig.add_subplot(222)
    ax2.set_title('Screw Axes at Home Configuration', fontsize=14, fontweight='bold')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_xlim([-0.5, 3])
    ax2.set_ylim([-2, 2])
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    
    # Matrix evolution visualization
    ax3 = fig.add_subplot(223)
    ax3.set_title('Product of Exponentials Build-up', fontsize=14, fontweight='bold')
    ax3.set_xlim([0, 10])
    ax3.set_ylim([0, 5])
    ax3.axis('off')
    
    # End-effector trajectory
    ax4 = fig.add_subplot(224)
    ax4.set_title('End-Effector Trajectory', fontsize=14, fontweight='bold')
    ax4.set_xlabel('x (m)')
    ax4.set_ylabel('y (m)')
    ax4.set_xlim([-3, 3])
    ax4.set_ylim([-3, 3])
    ax4.set_aspect('equal')
    ax4.grid(True, alpha=0.3)
    
    # Animation setup
    n_frames = 100
    t = np.linspace(0, 2*np.pi, n_frames)
    
    # Define joint trajectories
    theta1_traj = np.sin(t) * 0.5
    theta2_traj = np.sin(2*t) * 0.4
    theta3_traj = np.cos(t) * 0.3
    
    # Store end-effector trajectory
    ee_trajectory = []
    
    # Draw screw axes at home configuration (static)
    ax2.arrow(0, 0, 0, 0.3, head_width=0.1, head_length=0.1, fc='red', ec='red')
    ax2.text(0, 0.5, 'S₁', fontsize=12, ha='center', color='red')
    
    ax2.arrow(robot.L1, 0, 0, 0.3, head_width=0.1, head_length=0.1, fc='green', ec='green')
    ax2.text(robot.L1, 0.5, 'S₂', fontsize=12, ha='center', color='green')
    
    ax2.arrow(robot.L1 + robot.L2, 0, 0, 0.3, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
    ax2.text(robot.L1 + robot.L2, 0.5, 'S₃', fontsize=12, ha='center', color='blue')
    
    # Draw home configuration links
    ax2.plot([0, robot.L1], [0, 0], 'k-', linewidth=2, alpha=0.3)
    ax2.plot([robot.L1, robot.L1 + robot.L2], [0, 0], 'k-', linewidth=2, alpha=0.3)
    ax2.plot([robot.L1 + robot.L2, robot.L1 + robot.L2 + robot.L3], [0, 0], 'k-', linewidth=2, alpha=0.3)
    
    # Initialize animation elements
    robot_lines = []
    joints = []
    
    def init():
        return []
    
    def animate(frame):
        # Clear axes
        ax1.clear()
        ax3.clear()
        
        # Setup axes
        ax1.set_title('Robot Workspace', fontsize=14, fontweight='bold')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.set_xlim([-3, 3])
        ax1.set_ylim([-3, 3])
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        
        ax3.set_title('Product of Exponentials Build-up', fontsize=14, fontweight='bold')
        ax3.set_xlim([0, 10])
        ax3.set_ylim([0, 5])
        ax3.axis('off')
        
        # Current joint angles
        theta = [theta1_traj[frame], theta2_traj[frame], theta3_traj[frame]]
        
        # Get positions
        positions = robot.get_link_positions(theta)
        
        # Draw robot in ax1
        # Links
        ax1.plot([positions['base'][0], positions['joint1'][0]], 
                [positions['base'][1], positions['joint1'][1]], 
                'k-', linewidth=4, label='Link 1')
        ax1.plot([positions['joint1'][0], positions['joint2'][0]], 
                [positions['joint1'][1], positions['joint2'][1]], 
                'k-', linewidth=3.5, label='Link 2')
        ax1.plot([positions['joint2'][0], positions['end'][0]], 
                [positions['joint2'][1], positions['end'][1]], 
                'k-', linewidth=3, label='Link 3')
        
        # Joints
        ax1.plot(positions['base'][0], positions['base'][1], 'ko', markersize=10)
        ax1.plot(positions['joint1'][0], positions['joint1'][1], 'ro', markersize=8)
        ax1.plot(positions['joint2'][0], positions['joint2'][1], 'go', markersize=8)
        ax1.plot(positions['end'][0], positions['end'][1], 'bo', markersize=10)
        
        # Store and draw trajectory
        ee_trajectory.append(positions['end'].copy())
        if len(ee_trajectory) > 1:
            traj = np.array(ee_trajectory)
            ax4.plot(traj[:, 0], traj[:, 1], 'b-', alpha=0.5, linewidth=1)
            ax4.plot(traj[-1, 0], traj[-1, 1], 'bo', markersize=5)
        
        # Show PoE formula build-up in ax3
        ax3.text(5, 4.5, 'Forward Kinematics via Product of Exponentials', 
                fontsize=14, ha='center', fontweight='bold')
        
        # Show individual exponentials
        ax3.text(5, 3.8, r'$T = e^{[S_1]\theta_1} \cdot e^{[S_2]\theta_2} \cdot e^{[S_3]\theta_3} \cdot M$',
                fontsize=12, ha='center')
        
        # Current values
        ax3.text(1, 2.8, r'$\theta_1 = {:.2f}$'.format(theta[0]), 
                fontsize=11, color='red')
        ax3.text(3.5, 2.8, r'$\theta_2 = {:.2f}$'.format(theta[1]), 
                fontsize=11, color='green')
        ax3.text(6, 2.8, r'$\theta_3 = {:.2f}$'.format(theta[2]), 
                fontsize=11, color='blue')
        
        # Show resulting end-effector position
        ax3.text(5, 1.8, 'End-effector position:', fontsize=11, ha='center')
        ax3.text(5, 1.2, 'x = {:.3f} m, y = {:.3f} m'.format(
                positions['end'][0], positions['end'][1]), 
                fontsize=11, ha='center')
        
        # Add annotations
        ax3.text(5, 0.5, 'Notice: No D-H parameters needed!', 
                fontsize=10, ha='center', style='italic', color='darkblue')
        
        # Joint angle indicators on main plot
        ax1.text(-2.5, 2.5, 'θ₁ = {:.2f}°'.format(np.degrees(theta[0])), 
                fontsize=10, color='red')
        ax1.text(-2.5, 2.2, 'θ₂ = {:.2f}°'.format(np.degrees(theta[1])), 
                fontsize=10, color='green')
        ax1.text(-2.5, 1.9, 'θ₃ = {:.2f}°'.format(np.degrees(theta[2])), 
                fontsize=10, color='blue')
        
        return []
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=n_frames,
                        interval=50, blit=False, repeat=True)
    
    plt.suptitle('Forward Kinematics: Product of Exponentials\n'
                 'Each joint contributes a screw motion, composed via matrix multiplication',
                 fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
