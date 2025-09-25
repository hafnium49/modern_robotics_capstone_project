#!/usr/bin/env python3
"""
Slide 5: Rigid Body Motion - SE(3) and Twist Visualization
Demonstrates rotation matrices, homogeneous transformations, and screw motions
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    """Custom 3D arrow for matplotlib"""
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

def rotation_matrix(axis, theta):
    """
    Create rotation matrix using Rodrigues' formula
    axis: unit vector
    theta: angle in radians
    """
    axis = axis / np.linalg.norm(axis)
    a = np.cos(theta / 2)
    b, c, d = -axis * np.sin(theta / 2)
    return np.array([
        [a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
        [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
        [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]
    ])

def se3_to_matrix(omega, v, theta):
    """
    Convert screw parameters to SE(3) matrix using exponential map
    omega: rotation axis (3x1)
    v: linear velocity component (3x1)
    theta: motion magnitude
    """
    if np.linalg.norm(omega) < 1e-6:
        # Pure translation
        R = np.eye(3)
        p = v * theta
    else:
        omega_hat = omega / np.linalg.norm(omega)
        R = rotation_matrix(omega_hat, theta)
        
        # Compute translation using screw motion formula
        omega_skew = skew(omega_hat)
        G_theta = np.eye(3) * theta + (1 - np.cos(theta)) * omega_skew + \
                  (theta - np.sin(theta)) * omega_skew @ omega_skew
        p = G_theta @ v
    
    # Build 4x4 homogeneous transformation
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def skew(v):
    """Convert 3D vector to skew-symmetric matrix"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def draw_frame(ax, T, label='', scale=0.5, alpha=1.0):
    """Draw a coordinate frame at pose T"""
    origin = T[:3, 3]
    
    # Extract rotation matrix
    R = T[:3, :3]
    
    # Draw axes
    colors = ['r', 'g', 'b']
    labels = ['x', 'y', 'z']
    
    for i, (color, axis_label) in enumerate(zip(colors, labels)):
        direction = R[:, i] * scale
        arrow = Arrow3D([origin[0], origin[0] + direction[0]],
                       [origin[1], origin[1] + direction[1]],
                       [origin[2], origin[2] + direction[2]],
                       mutation_scale=20, lw=2, arrowstyle='-|>',
                       color=color, alpha=alpha)
        ax.add_artist(arrow)
    
    # Add label
    if label:
        ax.text(origin[0], origin[1], origin[2], label, fontsize=10)

def main():
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Left plot: SO(3) rotation demonstration
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('SO(3): Rotation Matrix Evolution', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-2, 2])
    ax1.set_zlim([-2, 2])
    
    # Right plot: SE(3) screw motion
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_title('SE(3): Screw Motion (Rotation + Translation)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim([-3, 3])
    ax2.set_ylim([-3, 3])
    ax2.set_zlim([-1, 5])
    
    # Animation parameters
    n_frames = 100
    t = np.linspace(0, 2*np.pi, n_frames)
    
    # SO(3) parameters - rotation around axis
    axis_so3 = np.array([1, 1, 1])
    axis_so3 = axis_so3 / np.linalg.norm(axis_so3)
    
    # SE(3) parameters - screw motion
    omega = np.array([0, 0, 1])  # Rotation around z-axis
    v = np.array([0, 0, 0.5])    # Pitch of 0.5
    
    # Store trajectory points for screw motion
    trajectory = []
    
    def init():
        ax1.clear()
        ax2.clear()
        
        # Setup axes for SO(3)
        ax1.set_title('SO(3): Rotation Matrix Evolution', fontsize=14, fontweight='bold')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_xlim([-2, 2])
        ax1.set_ylim([-2, 2])
        ax1.set_zlim([-2, 2])
        
        # Setup axes for SE(3)
        ax2.set_title('SE(3): Screw Motion (Rotation + Translation)', fontsize=14, fontweight='bold')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.set_xlim([-3, 3])
        ax2.set_ylim([-3, 3])
        ax2.set_zlim([-1, 5])
        
        return []
    
    def animate(frame):
        ax1.clear()
        ax2.clear()
        
        # Current angle
        theta = t[frame]
        
        # === SO(3) Visualization ===
        ax1.set_title('SO(3): Rotation Matrix Evolution\nθ = {:.2f} rad'.format(theta), 
                     fontsize=14, fontweight='bold')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_xlim([-2, 2])
        ax1.set_ylim([-2, 2])
        ax1.set_zlim([-2, 2])
        
        # Draw rotation axis
        ax1.plot([0, axis_so3[0]*2], [0, axis_so3[1]*2], [0, axis_so3[2]*2], 
                'k--', alpha=0.3, linewidth=1)
        ax1.text(axis_so3[0]*2.2, axis_so3[1]*2.2, axis_so3[2]*2.2, 
                'rotation axis', fontsize=8)
        
        # Draw initial frame (identity)
        T_identity = np.eye(4)
        draw_frame(ax1, T_identity, 'Initial', scale=0.7, alpha=0.3)
        
        # Draw rotated frame
        R = rotation_matrix(axis_so3, theta)
        T_rotated = np.eye(4)
        T_rotated[:3, :3] = R
        draw_frame(ax1, T_rotated, 'Rotated', scale=1.0, alpha=1.0)
        
        # Show rotation matrix values
        ax1.text2D(0.02, 0.95, 'R ∈ SO(3):\n' + 
                  'det(R) = 1\n' +
                  'R^T R = I', 
                  transform=ax1.transAxes, fontsize=10,
                  verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # === SE(3) Visualization ===
        ax2.set_title('SE(3): Screw Motion\nθ = {:.2f} rad'.format(theta), 
                     fontsize=14, fontweight='bold')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.set_xlim([-3, 3])
        ax2.set_ylim([-3, 3])
        ax2.set_zlim([-1, 5])
        
        # Draw screw axis
        ax2.plot([0, 0], [0, 0], [-1, 5], 'k--', alpha=0.3, linewidth=2)
        ax2.text(0.2, 0.2, 5, 'screw axis', fontsize=8)
        
        # Compute screw motion
        T_screw = se3_to_matrix(omega, v, theta)
        
        # Store trajectory point
        origin = T_screw[:3, 3]
        trajectory.append(origin.copy())
        
        # Draw trajectory
        if len(trajectory) > 1:
            traj = np.array(trajectory)
            ax2.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', alpha=0.5, linewidth=2)
        
        # Draw initial frame
        draw_frame(ax2, np.eye(4), 'Start', scale=0.5, alpha=0.3)
        
        # Draw current frame
        draw_frame(ax2, T_screw, 'Current', scale=0.7, alpha=1.0)
        
        # Draw helix to show screw motion
        theta_helix = np.linspace(0, theta, 50)
        x_helix = np.cos(theta_helix)
        y_helix = np.sin(theta_helix)
        z_helix = theta_helix * 0.5 / (2*np.pi) * 2*np.pi  # Adjust pitch
        ax2.plot(x_helix, y_helix, z_helix, 'r--', alpha=0.3, linewidth=1)
        
        # Show transformation matrix info
        ax2.text2D(0.02, 0.95, 'T ∈ SE(3):\n' +
                  'T = [R  p]\n' +
                  '    [0  1]\n' +
                  'Screw: ω = [0,0,1]\n' +
                  '       v = [0,0,0.5]',
                  transform=ax2.transAxes, fontsize=10,
                  verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Mathematical formulas
        ax1.text2D(0.5, 0.02, "Rodrigues' formula: R = e^{[ω̂]θ} = I + sin(θ)[ω̂] + (1-cos(θ))[ω̂]²",
                  transform=ax1.transAxes, fontsize=10, ha='center',
                  style='italic', color='darkblue')
        
        ax2.text2D(0.5, 0.02, "Screw motion: T = e^{[S]θ} where S = (ω, v) is the screw axis",
                  transform=ax2.transAxes, fontsize=10, ha='center',
                  style='italic', color='darkblue')
        
        return []
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=n_frames,
                        interval=50, blit=False, repeat=True)
    
    plt.suptitle('Rigid Body Motion: SO(3) and SE(3)\n'
                 'Unified representation of rotation and transformation',
                 fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
