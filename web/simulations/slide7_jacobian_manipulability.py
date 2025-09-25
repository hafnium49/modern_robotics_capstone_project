#!/usr/bin/env python3
"""
Slide 7: Jacobian and Velocity Kinematics
Visualizes Jacobian columns, manipulability ellipsoid, and singularities
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse, Arrow
import matplotlib.patches as patches

class TwoLinkArm:
    def __init__(self, L1=1.0, L2=1.0):
        self.L1 = L1
        self.L2 = L2
        
    def forward_kinematics(self, theta1, theta2):
        """Compute joint and end-effector positions"""
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        return x1, y1, x2, y2
    
    def compute_jacobian(self, theta1, theta2):
        """Compute the 2x2 Jacobian matrix"""
        J = np.array([
            [-self.L1*np.sin(theta1) - self.L2*np.sin(theta1+theta2), 
             -self.L2*np.sin(theta1+theta2)],
            [self.L1*np.cos(theta1) + self.L2*np.cos(theta1+theta2),  
             self.L2*np.cos(theta1+theta2)]
        ])
        return J
    
    def compute_manipulability_ellipse(self, theta1, theta2):
        """
        Compute manipulability ellipsoid parameters
        Returns: center, width, height, angle
        """
        J = self.compute_jacobian(theta1, theta2)
        
        # Compute J @ J^T for velocity ellipsoid
        A = J @ J.T
        
        # Eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eig(A)
        
        # Sort by eigenvalue magnitude
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Ellipse parameters
        width = 2 * np.sqrt(eigenvalues[0]) if eigenvalues[0] > 0 else 0
        height = 2 * np.sqrt(eigenvalues[1]) if eigenvalues[1] > 0 else 0
        angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
        
        # Compute manipulability measure
        manipulability = np.sqrt(np.linalg.det(A)) if np.linalg.det(A) > 0 else 0
        
        # Compute condition number
        if eigenvalues[1] > 1e-10:
            condition = np.sqrt(eigenvalues[0] / eigenvalues[1])
        else:
            condition = np.inf
            
        return width, height, angle, manipulability, condition
    
    def get_jacobian_columns(self, theta1, theta2):
        """Get the two columns of the Jacobian as velocity vectors"""
        J = self.compute_jacobian(theta1, theta2)
        return J[:, 0], J[:, 1]

def main():
    # Create robot
    robot = TwoLinkArm(L1=1.0, L2=1.0)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Main robot visualization
    ax1 = fig.add_subplot(221)
    ax1.set_title('Robot Configuration & Manipulability Ellipsoid', 
                  fontsize=14, fontweight='bold')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_xlim([-2.5, 2.5])
    ax1.set_ylim([-2.5, 2.5])
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    # Jacobian columns visualization
    ax2 = fig.add_subplot(222)
    ax2.set_title('Jacobian Column Vectors (Joint Velocities → EE Twist)', 
                  fontsize=14, fontweight='bold')
    ax2.set_xlabel('vx')
    ax2.set_ylabel('vy')
    ax2.set_xlim([-3, 3])
    ax2.set_ylim([-3, 3])
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    
    # Manipulability measure over time
    ax3 = fig.add_subplot(223)
    ax3.set_title('Manipulability & Condition Number', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Frame')
    ax3.set_ylabel('Value')
    ax3.set_xlim([0, 100])
    ax3.grid(True, alpha=0.3)
    
    # Force ellipsoid (dual of velocity)
    ax4 = fig.add_subplot(224)
    ax4.set_title('Force Ellipsoid (Dual of Velocity)', fontsize=14, fontweight='bold')
    ax4.set_xlabel('fx')
    ax4.set_ylabel('fy')
    ax4.set_xlim([-3, 3])
    ax4.set_ylim([-3, 3])
    ax4.set_aspect('equal')
    ax4.grid(True, alpha=0.3)
    
    # Animation parameters
    n_frames = 100
    t = np.linspace(0, 2*np.pi, n_frames)
    
    # Define joint trajectories (will pass through singularity)
    theta1_traj = np.pi/4 + np.sin(t) * np.pi/3
    theta2_traj = np.sin(2*t) * np.pi/2
    
    # Storage for metrics
    manipulability_history = []
    condition_history = []
    
    def init():
        return []
    
    def animate(frame):
        # Clear axes
        ax1.clear()
        ax2.clear()
        ax4.clear()
        
        # Reset axes properties
        ax1.set_title('Robot Configuration & Manipulability Ellipsoid', 
                     fontsize=14, fontweight='bold')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.set_xlim([-2.5, 2.5])
        ax1.set_ylim([-2.5, 2.5])
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        
        ax2.set_title('Jacobian Column Vectors (Joint Velocities → EE Twist)', 
                     fontsize=14, fontweight='bold')
        ax2.set_xlabel('vx')
        ax2.set_ylabel('vy')
        ax2.set_xlim([-3, 3])
        ax2.set_ylim([-3, 3])
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        
        ax4.set_title('Force Ellipsoid (Dual of Velocity)', 
                     fontsize=14, fontweight='bold')
        ax4.set_xlabel('fx')
        ax4.set_ylabel('fy')
        ax4.set_xlim([-3, 3])
        ax4.set_ylim([-3, 3])
        ax4.set_aspect('equal')
        ax4.grid(True, alpha=0.3)
        
        # Current configuration
        theta1 = theta1_traj[frame]
        theta2 = theta2_traj[frame]
        
        # Get positions
        x1, y1, x2, y2 = robot.forward_kinematics(theta1, theta2)
        
        # Draw robot
        ax1.plot([0, x1], [0, y1], 'k-', linewidth=4, label='Link 1')
        ax1.plot([x1, x2], [y1, y2], 'k-', linewidth=4, label='Link 2')
        ax1.plot(0, 0, 'ko', markersize=10)
        ax1.plot(x1, y1, 'ko', markersize=8)
        ax1.plot(x2, y2, 'ro', markersize=10)
        
        # Compute and draw manipulability ellipsoid
        width, height, angle, manip, cond = robot.compute_manipulability_ellipse(theta1, theta2)
        
        # Store metrics
        manipulability_history.append(manip)
        condition_history.append(min(cond, 20))  # Cap for visualization
        
        # Draw velocity ellipsoid
        if manip > 0.01:  # Only draw if not too close to singularity
            ellipse = Ellipse((x2, y2), width, height, 
                             angle=np.degrees(angle), 
                             fill=False, edgecolor='blue', linewidth=2)
            ax1.add_patch(ellipse)
            
            # Color based on condition number
            if cond < 2:
                color = 'green'
                status = 'Good'
            elif cond < 5:
                color = 'orange'
                status = 'Fair'
            else:
                color = 'red'
                status = 'Poor/Singular'
        else:
            color = 'red'
            status = 'SINGULAR!'
        
        # Status text
        ax1.text(0.02, 0.98, f'Manipulability: {manip:.3f}\nCondition: {cond:.2f}\nStatus: {status}',
                transform=ax1.transAxes, fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor=color, alpha=0.3))
        
        # Get Jacobian columns
        J = robot.compute_jacobian(theta1, theta2)
        col1, col2 = robot.get_jacobian_columns(theta1, theta2)
        
        # Draw Jacobian columns in velocity space
        ax2.arrow(0, 0, col1[0], col1[1], head_width=0.1, head_length=0.1, 
                 fc='red', ec='red', alpha=0.7, linewidth=2)
        ax2.arrow(0, 0, col2[0], col2[1], head_width=0.1, head_length=0.1, 
                 fc='blue', ec='blue', alpha=0.7, linewidth=2)
        
        ax2.text(col1[0]*1.1, col1[1]*1.1, 'J₁ (θ̇₁=1)', fontsize=10, color='red')
        ax2.text(col2[0]*1.1, col2[1]*1.1, 'J₂ (θ̇₂=1)', fontsize=10, color='blue')
        
        # Draw unit circle to show joint rate constraint
        circle = plt.Circle((0, 0), 1, fill=False, edgecolor='gray', 
                           linestyle='--', alpha=0.5)
        ax2.add_patch(circle)
        ax2.text(0.7, 0.7, '‖θ̇‖=1', fontsize=9, color='gray')
        
        # Draw velocity ellipsoid in velocity space
        if manip > 0.01:
            ellipse2 = Ellipse((0, 0), width, height, 
                              angle=np.degrees(angle), 
                              fill=True, facecolor='cyan', alpha=0.3,
                              edgecolor='blue', linewidth=1)
            ax2.add_patch(ellipse2)
        
        # Force ellipsoid (inverse of velocity ellipsoid)
        if manip > 0.01:
            # For force ellipsoid, use inverse relationship
            J_inv = np.linalg.pinv(J)
            A_force = J_inv.T @ J_inv
            eigenvalues_f, eigenvectors_f = np.linalg.eig(A_force)
            
            idx = eigenvalues_f.argsort()[::-1]
            eigenvalues_f = eigenvalues_f[idx]
            eigenvectors_f = eigenvectors_f[:, idx]
            
            width_f = 2 * np.sqrt(eigenvalues_f[0]) if eigenvalues_f[0] > 0 else 0
            height_f = 2 * np.sqrt(eigenvalues_f[1]) if eigenvalues_f[1] > 0 else 0
            angle_f = np.arctan2(eigenvectors_f[1, 0], eigenvectors_f[0, 0])
            
            ellipse3 = Ellipse((0, 0), min(width_f, 3), min(height_f, 3), 
                              angle=np.degrees(angle_f), 
                              fill=True, facecolor='orange', alpha=0.3,
                              edgecolor='red', linewidth=2)
            ax4.add_patch(ellipse3)
            
            ax4.text(0.02, 0.98, 'Force capacity\n(inverse of velocity)',
                    transform=ax4.transAxes, fontsize=10,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Plot metrics history
        ax3.clear()
        ax3.set_title('Manipulability & Condition Number', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Frame')
        ax3.set_ylabel('Value')
        ax3.set_xlim([0, 100])
        ax3.grid(True, alpha=0.3)
        
        frames = list(range(len(manipulability_history)))
        ax3.plot(frames, manipulability_history, 'b-', label='Manipulability', linewidth=2)
        ax3.plot(frames, condition_history, 'r-', label='Condition Number', linewidth=2)
        ax3.axhline(y=5, color='orange', linestyle='--', alpha=0.5)
        ax3.text(90, 5.5, 'Poor', fontsize=8, color='orange')
        ax3.legend(loc='upper left')
        
        # Add current frame marker
        ax3.axvline(x=frame, color='gray', linestyle=':', alpha=0.5)
        
        # Mathematical formulas
        ax1.text(0.5, -0.05, 'V = J(θ)θ̇    τ = J^T(θ)F',
                transform=ax1.transAxes, fontsize=11, ha='center',
                style='italic', color='darkblue')
        
        return []
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=n_frames,
                        interval=50, blit=False, repeat=True)
    
    plt.suptitle('Jacobian: The Bridge Between Joint and Task Space\n'
                 'Velocity ellipsoid shows motion capability; Force ellipsoid shows strength',
                 fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
