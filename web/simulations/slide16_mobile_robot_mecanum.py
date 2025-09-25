#!/usr/bin/env python3
"""
Slide 16: Mobile Robot - Mecanum Wheel Kinematics
Demonstrates omnidirectional motion and velocity constraints
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle, Arrow

class MecanumRobot:
    """
    Four-wheel mecanum robot simulation
    Wheel configuration:
    1---2
    |   |
    3---4
    """
    
    def __init__(self, length=0.235, width=0.15):
        self.L = length  # Half-length
        self.W = width   # Half-width
        self.wheel_radius = 0.05
        
        # Robot pose: [x, y, phi]
        self.pose = np.array([0.0, 0.0, 0.0])
        
        # Mecanum wheel angles (45 degrees for rollers)
        self.roller_angle = np.pi/4
        
        # H matrix for mecanum kinematics (body to wheel velocities)
        # This is the inverse of the usual formulation
        self.H_inv = np.array([
            [-1/(self.L+self.W),  1,  -1],  # Wheel 1 (front-left)
            [ 1/(self.L+self.W),  1,   1],  # Wheel 2 (front-right)
            [ 1/(self.L+self.W),  1,  -1],  # Wheel 3 (rear-left)
            [-1/(self.L+self.W),  1,   1],  # Wheel 4 (rear-right)
        ]) / self.wheel_radius
        
        # Forward kinematics matrix (wheel velocities to body twist)
        self.H = np.linalg.pinv(self.H_inv)
        
    def body_to_wheel_velocities(self, vx, vy, omega):
        """Convert body twist to wheel velocities"""
        body_twist = np.array([omega, vx, vy])
        wheel_velocities = self.H_inv @ body_twist
        return wheel_velocities
    
    def wheel_to_body_velocities(self, wheel_velocities):
        """Convert wheel velocities to body twist"""
        body_twist = self.H @ wheel_velocities
        return body_twist[1], body_twist[2], body_twist[0]  # vx, vy, omega
    
    def update_pose(self, vx, vy, omega, dt):
        """Update robot pose given body velocities"""
        # Convert body velocities to world frame
        phi = self.pose[2]
        R = np.array([
            [np.cos(phi), -np.sin(phi)],
            [np.sin(phi),  np.cos(phi)]
        ])
        
        v_world = R @ np.array([vx, vy])
        
        # Update pose
        self.pose[0] += v_world[0] * dt
        self.pose[1] += v_world[1] * dt
        self.pose[2] += omega * dt
        
        # Wrap angle
        self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
        
    def get_wheel_positions(self):
        """Get wheel positions in world frame"""
        x, y, phi = self.pose
        R = np.array([
            [np.cos(phi), -np.sin(phi)],
            [np.sin(phi),  np.cos(phi)]
        ])
        
        # Wheel positions in body frame
        wheel_body = np.array([
            [-self.L,  self.W],  # Front-left
            [ self.L,  self.W],  # Front-right
            [-self.L, -self.W],  # Rear-left
            [ self.L, -self.W],  # Rear-right
        ])
        
        # Transform to world frame
        wheel_world = []
        for wb in wheel_body:
            ww = R @ wb + np.array([x, y])
            wheel_world.append(ww)
            
        return wheel_world
    
    def get_velocity_constraints(self, max_wheel_speed=10.0):
        """
        Get velocity polytope constraints
        Returns vertices of feasible velocity region
        """
        # Sample wheel speed combinations at limits
        n_samples = 8
        angles = np.linspace(0, 2*np.pi, n_samples, endpoint=False)
        
        vertices = []
        for angle in angles:
            # Generate wheel speeds on boundary
            wheel_speeds = max_wheel_speed * np.array([
                np.cos(angle + i*np.pi/2) for i in range(4)
            ])
            
            # Convert to body velocities
            vx, vy, omega = self.wheel_to_body_velocities(wheel_speeds)
            vertices.append([vx, vy, omega])
            
        return np.array(vertices)

def main():
    # Create robot
    robot = MecanumRobot()
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Robot workspace visualization
    ax1 = fig.add_subplot(221)
    ax1.set_title('Mecanum Robot Workspace', fontsize=14, fontweight='bold')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-2, 2])
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    # Velocity space (3D projected to 2D)
    ax2 = fig.add_subplot(222)
    ax2.set_title('Body Velocity Space (vx, vy)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('vx (m/s)')
    ax2.set_ylabel('vy (m/s)')
    ax2.set_xlim([-1, 1])
    ax2.set_ylim([-1, 1])
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    
    # Wheel velocities
    ax3 = fig.add_subplot(223)
    ax3.set_title('Individual Wheel Velocities', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Velocity (rad/s)')
    ax3.set_xlim([0, 10])
    ax3.set_ylim([-15, 15])
    ax3.grid(True, alpha=0.3)
    
    # Motion modes
    ax4 = fig.add_subplot(224)
    ax4.set_title('Motion Mode', fontsize=14, fontweight='bold')
    ax4.set_xlim([0, 1])
    ax4.set_ylim([0, 1])
    ax4.axis('off')
    
    # Animation parameters
    dt = 0.05
    total_time = 10.0
    n_frames = int(total_time / dt)
    time_array = np.linspace(0, total_time, n_frames)
    
    # Define motion sequence
    def get_command(t):
        """Get velocity command based on time"""
        phase = (t / 2.5) % 4
        
        if phase < 1:
            # Forward
            return 0.5, 0, 0, "Forward"
        elif phase < 2:
            # Sideways
            return 0, 0.5, 0, "Sideways"
        elif phase < 3:
            # Diagonal
            return 0.35, 0.35, 0, "Diagonal"
        else:
            # Rotation
            return 0, 0, 1.0, "Pure Rotation"
    
    # Storage for history
    trajectory = []
    wheel_velocity_history = {i: [] for i in range(4)}
    time_history = []
    
    def draw_robot(ax, pose, wheel_velocities=None):
        """Draw robot at given pose"""
        x, y, phi = pose
        
        # Robot body
        robot_width = 0.3
        robot_length = 0.47
        
        # Create rotation matrix
        R = np.array([
            [np.cos(phi), -np.sin(phi)],
            [np.sin(phi),  np.cos(phi)]
        ])
        
        # Body corners in body frame
        corners_body = np.array([
            [-robot_length/2, -robot_width/2],
            [ robot_length/2, -robot_width/2],
            [ robot_length/2,  robot_width/2],
            [-robot_length/2,  robot_width/2],
            [-robot_length/2, -robot_width/2]
        ])
        
        # Transform to world frame
        corners_world = np.array([R @ c + np.array([x, y]) for c in corners_body])
        
        # Draw body
        body = patches.Polygon(corners_world, closed=True, 
                              edgecolor='black', facecolor='lightgray', 
                              linewidth=2)
        ax.add_patch(body)
        
        # Draw wheels
        wheel_positions = robot.get_wheel_positions()
        wheel_colors = ['red', 'green', 'blue', 'orange']
        
        for i, (wp, color) in enumerate(zip(wheel_positions, wheel_colors)):
            # Draw wheel
            wheel = Circle(wp, 0.03, color=color, alpha=0.7)
            ax.add_patch(wheel)
            
            # Draw roller direction (45 degree lines)
            roller_angle = phi + ((-1)**(i//2)) * np.pi/4
            dx = 0.04 * np.cos(roller_angle)
            dy = 0.04 * np.sin(roller_angle)
            ax.plot([wp[0]-dx, wp[0]+dx], [wp[1]-dy, wp[1]+dy], 
                   color=color, linewidth=2, alpha=0.5)
            
            # Show wheel velocity if provided
            if wheel_velocities is not None:
                # Arrow showing rotation direction
                if abs(wheel_velocities[i]) > 0.1:
                    arrow_scale = np.clip(wheel_velocities[i] / 10, -1, 1) * 0.05
                    arrow_angle = phi + np.pi/2 if wheel_velocities[i] > 0 else phi - np.pi/2
                    ax.arrow(wp[0], wp[1], 
                           arrow_scale * np.cos(arrow_angle),
                           arrow_scale * np.sin(arrow_angle),
                           head_width=0.01, head_length=0.01,
                           fc=color, ec=color, alpha=0.8)
        
        # Draw heading arrow
        ax.arrow(x, y, 0.2*np.cos(phi), 0.2*np.sin(phi),
                head_width=0.05, head_length=0.05,
                fc='black', ec='black')
    
    def init():
        return []
    
    def animate(frame):
        # Clear axes
        ax1.clear()
        ax2.clear()
        ax4.clear()
        
        # Reset axes properties
        ax1.set_title('Mecanum Robot Workspace', fontsize=14, fontweight='bold')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.set_xlim([-2, 2])
        ax1.set_ylim([-2, 2])
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        
        ax2.set_title('Body Velocity Space (vx, vy)', fontsize=14, fontweight='bold')
        ax2.set_xlabel('vx (m/s)')
        ax2.set_ylabel('vy (m/s)')
        ax2.set_xlim([-1, 1])
        ax2.set_ylim([-1, 1])
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        
        ax4.set_title('Motion Mode', fontsize=14, fontweight='bold')
        ax4.set_xlim([0, 1])
        ax4.set_ylim([0, 1])
        ax4.axis('off')
        
        # Get current command
        current_time = frame * dt
        vx_cmd, vy_cmd, omega_cmd, mode = get_command(current_time)
        
        # Compute wheel velocities
        wheel_velocities = robot.body_to_wheel_velocities(vx_cmd, vy_cmd, omega_cmd)
        
        # Update robot pose
        robot.update_pose(vx_cmd, vy_cmd, omega_cmd, dt)
        
        # Store history
        trajectory.append(robot.pose[:2].copy())
        time_history.append(current_time)
        for i in range(4):
            wheel_velocity_history[i].append(wheel_velocities[i])
        
        # Draw robot
        draw_robot(ax1, robot.pose, wheel_velocities)
        
        # Draw trajectory
        if len(trajectory) > 1:
            traj = np.array(trajectory)
            ax1.plot(traj[:, 0], traj[:, 1], 'b-', alpha=0.5, linewidth=1)
        
        # Draw velocity in velocity space
        ax2.arrow(0, 0, vx_cmd, vy_cmd, head_width=0.05, head_length=0.05,
                 fc='blue', ec='blue', linewidth=2)
        ax2.plot(vx_cmd, vy_cmd, 'bo', markersize=8)
        
        # Draw velocity constraint polytope
        max_wheel_speed = 10.0
        # Simple box approximation for visualization
        vmax = 0.5
        rectangle = Rectangle((-vmax, -vmax), 2*vmax, 2*vmax, 
                            fill=False, edgecolor='gray', linestyle='--', alpha=0.5)
        ax2.add_patch(rectangle)
        ax2.text(vmax*0.7, vmax*1.1, 'Feasible velocities', fontsize=9, color='gray')
        
        # Update wheel velocity plot
        ax3.clear()
        ax3.set_title('Individual Wheel Velocities', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angular Velocity (rad/s)')
        ax3.set_xlim([0, 10])
        ax3.set_ylim([-15, 15])
        ax3.grid(True, alpha=0.3)
        
        colors = ['red', 'green', 'blue', 'orange']
        labels = ['FL', 'FR', 'RL', 'RR']
        
        for i, (color, label) in enumerate(zip(colors, labels)):
            if len(time_history) > 0:
                ax3.plot(time_history, wheel_velocity_history[i], 
                        color=color, label=f'Wheel {i+1} ({label})', linewidth=2)
        
        ax3.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        ax3.legend(loc='upper right')
        
        # Show motion mode
        ax4.text(0.5, 0.7, mode, fontsize=20, ha='center', fontweight='bold')
        ax4.text(0.5, 0.5, f'vx = {vx_cmd:.2f} m/s', fontsize=14, ha='center')
        ax4.text(0.5, 0.4, f'vy = {vy_cmd:.2f} m/s', fontsize=14, ha='center')
        ax4.text(0.5, 0.3, f'ω = {omega_cmd:.2f} rad/s', fontsize=14, ha='center')
        
        # Wheel velocity pattern
        ax4.text(0.5, 0.1, f'Wheels: [{wheel_velocities[0]:.1f}, {wheel_velocities[1]:.1f}, '
                          f'{wheel_velocities[2]:.1f}, {wheel_velocities[3]:.1f}] rad/s',
                fontsize=10, ha='center', style='italic')
        
        # Add info text
        ax1.text(0.02, 0.02, f't = {current_time:.1f}s\nPose: ({robot.pose[0]:.2f}, '
                           f'{robot.pose[1]:.2f}, {np.degrees(robot.pose[2]):.1f}°)',
                transform=ax1.transAxes, fontsize=9,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        return []
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=n_frames,
                        interval=50, blit=False, repeat=True)
    
    plt.suptitle('Mecanum Wheels: True Omnidirectional Motion\n'
                 'Independent control of vx, vy, and ω enables holonomic motion',
                 fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
