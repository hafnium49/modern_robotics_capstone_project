#!/usr/bin/env python3
"""
Slide 4: Configuration Space Visualization
2R Planar Arm - Joint Space to Task Space Mapping
Shows how a straight line in joint space creates a curved path in task space
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import matplotlib.patches as mpatches

class TwoLinkArm:
    def __init__(self, L1=1.0, L2=0.7):
        self.L1 = L1  # Length of first link
        self.L2 = L2  # Length of second link
        
    def forward_kinematics(self, theta1, theta2):
        """Compute end-effector position from joint angles"""
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        
        return x1, y1, x2, y2
    
    def get_workspace_boundary(self, n_points=1000):
        """Get the reachable workspace boundary"""
        theta = np.linspace(0, 2*np.pi, n_points)
        
        # Outer circle (full extension)
        r_max = self.L1 + self.L2
        outer_x = r_max * np.cos(theta)
        outer_y = r_max * np.sin(theta)
        
        # Inner circle (if L1 > L2)
        if self.L1 > self.L2:
            r_min = self.L1 - self.L2
            inner_x = r_min * np.cos(theta)
            inner_y = r_min * np.sin(theta)
        else:
            inner_x = inner_y = None
            
        return outer_x, outer_y, inner_x, inner_y

def main():
    # Create robot
    robot = TwoLinkArm(L1=1.0, L2=0.7)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 8))
    
    # Configuration space (left)
    ax1 = fig.add_subplot(121)
    ax1.set_title('Configuration Space (Joint Space)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('θ₁ (rad)', fontsize=12)
    ax1.set_ylabel('θ₂ (rad)', fontsize=12)
    ax1.set_xlim([0, 2*np.pi])
    ax1.set_ylim([0, 2*np.pi])
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Add torus topology indicators
    ax1.text(np.pi, -0.3, '↔ Periodic', ha='center', fontsize=10, color='blue')
    ax1.text(-0.3, np.pi, '↕ Periodic', ha='center', rotation=90, fontsize=10, color='blue')
    
    # Task space (right)
    ax2 = fig.add_subplot(122)
    ax2.set_title('Task Space (Workspace)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('x (m)', fontsize=12)
    ax2.set_ylabel('y (m)', fontsize=12)
    ax2.set_xlim([-2, 2])
    ax2.set_ylim([-2, 2])
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # Draw workspace boundary
    outer_x, outer_y, inner_x, inner_y = robot.get_workspace_boundary()
    ax2.plot(outer_x, outer_y, 'k--', alpha=0.3, label='Workspace boundary')
    if inner_x is not None:
        ax2.plot(inner_x, inner_y, 'k--', alpha=0.3)
    
    # Path 1: Straight line in joint space
    n_points = 50
    t = np.linspace(0, 1, n_points)
    
    # Start and end configurations
    theta1_start, theta2_start = np.pi/6, np.pi/3
    theta1_end, theta2_end = 5*np.pi/6, 5*np.pi/3
    
    # Linear interpolation in joint space
    theta1_path = theta1_start + t * (theta1_end - theta1_start)
    theta2_path = theta2_start + t * (theta2_end - theta2_start)
    
    # Plot joint space path
    ax1.plot(theta1_path, theta2_path, 'b-', linewidth=2, label='Straight line path')
    ax1.plot(theta1_start, theta2_start, 'go', markersize=10, label='Start')
    ax1.plot(theta1_end, theta2_end, 'ro', markersize=10, label='End')
    
    # Compute and plot task space path
    x_path = []
    y_path = []
    for theta1, theta2 in zip(theta1_path, theta2_path):
        _, _, x, y = robot.forward_kinematics(theta1, theta2)
        x_path.append(x)
        y_path.append(y)
    
    ax2.plot(x_path, y_path, 'b-', linewidth=2, label='Resulting curved path')
    ax2.plot(x_path[0], y_path[0], 'go', markersize=10, label='Start')
    ax2.plot(x_path[-1], y_path[-1], 'ro', markersize=10, label='End')
    
    # Animation elements
    robot_lines = []
    joint_dots = []
    
    # Initialize robot visualization
    line1, = ax2.plot([], [], 'k-', linewidth=3)
    line2, = ax2.plot([], [], 'k-', linewidth=3)
    joint0 = Circle((0, 0), 0.05, color='black')
    joint1 = Circle((0, 0), 0.05, color='black')
    joint2 = Circle((0, 0), 0.05, color='red')
    ax2.add_patch(joint0)
    ax2.add_patch(joint1)
    ax2.add_patch(joint2)
    
    # Current position markers
    c_space_marker, = ax1.plot([], [], 'ko', markersize=8)
    t_space_marker, = ax2.plot([], [], 'ko', markersize=8)
    
    # Path traces
    c_space_trace, = ax1.plot([], [], 'b-', linewidth=2, alpha=0.5)
    t_space_trace, = ax2.plot([], [], 'b-', linewidth=2, alpha=0.5)
    
    def init():
        return [line1, line2, joint1, joint2, c_space_marker, t_space_marker,
                c_space_trace, t_space_trace]
    
    def animate(frame):
        # Current configuration
        theta1 = theta1_path[frame]
        theta2 = theta2_path[frame]
        
        # Update configuration space marker
        c_space_marker.set_data([theta1], [theta2])
        c_space_trace.set_data(theta1_path[:frame+1], theta2_path[:frame+1])
        
        # Compute robot pose
        x1, y1, x2, y2 = robot.forward_kinematics(theta1, theta2)
        
        # Update robot visualization
        line1.set_data([0, x1], [0, y1])
        line2.set_data([x1, x2], [y1, y2])
        joint1.center = (x1, y1)
        joint2.center = (x2, y2)
        
        # Update task space marker
        t_space_marker.set_data([x2], [y2])
        t_space_trace.set_data(x_path[:frame+1], y_path[:frame+1])
        
        return [line1, line2, joint1, joint2, c_space_marker, t_space_marker,
                c_space_trace, t_space_trace]
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=n_points, 
                        interval=50, blit=True, repeat=True)
    
    # Add legends
    ax1.legend(loc='upper right')
    ax2.legend(loc='upper right')
    
    # Add information text
    fig.suptitle('Configuration Space: 2R Planar Arm\n'
                 'Notice how a straight line in joint space creates a curved path in task space',
                 fontsize=16, fontweight='bold')
    
    # Add mathematical note
    fig.text(0.5, 0.02, 
             'C-space topology: T² = S¹ × S¹ (torus)\n'
             'Each joint angle wraps at 2π, creating periodic boundary conditions',
             ha='center', fontsize=11, style='italic')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
