#!/usr/bin/env python3
"""
Slide 11: Trajectory Generation
Demonstrates the separation of path and time scaling
Shows different time scaling profiles (trapezoidal, S-curve, quintic)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

def quintic_time_scaling(T, t):
    """
    Quintic polynomial time scaling
    Returns s(t), s_dot(t), s_ddot(t)
    """
    tau = t / T
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    s_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / (T**2)
    return s, s_dot, s_ddot

def trapezoidal_time_scaling(T, t, acc_time_ratio=0.3):
    """
    Trapezoidal velocity profile
    """
    t_acc = T * acc_time_ratio
    t_const = T - 2 * t_acc
    v_max = 1.0 / (T - t_acc)
    
    if t <= t_acc:
        # Acceleration phase
        a = v_max / t_acc
        s = 0.5 * a * t**2
        s_dot = a * t
        s_ddot = a
    elif t <= T - t_acc:
        # Constant velocity phase
        s = 0.5 * v_max * t_acc + v_max * (t - t_acc)
        s_dot = v_max
        s_ddot = 0
    else:
        # Deceleration phase
        t_dec = t - (T - t_acc)
        a = v_max / t_acc
        s = 0.5 * v_max * t_acc + v_max * t_const + v_max * t_dec - 0.5 * a * t_dec**2
        s_dot = v_max - a * t_dec
        s_ddot = -a
    
    return s, s_dot, s_ddot

def s_curve_time_scaling(T, t, jerk_time_ratio=0.2):
    """
    S-curve (7-segment) profile with jerk limits
    Simplified version for visualization
    """
    tj = T * jerk_time_ratio
    
    if t <= tj:
        # Jerk phase 1
        j = 2.0 / (tj * T)
        s = (j * t**3) / 6
        s_dot = 0.5 * j * t**2
        s_ddot = j * t
    elif t <= T/2:
        # Constant acceleration
        t1 = t - tj
        a_max = 2.0 * tj / (tj * T)
        v0 = 0.5 * a_max * tj
        s0 = a_max * tj**3 / 6
        s = s0 + v0 * t1 + 0.5 * a_max * t1**2
        s_dot = v0 + a_max * t1
        s_ddot = a_max
    elif t <= T - tj:
        # Use symmetry
        s, s_dot, s_ddot = s_curve_time_scaling(T, T - t, jerk_time_ratio)
        s = 1 - s
        s_dot = -s_dot
        s_ddot = -s_ddot
    else:
        # Final jerk phase
        t_end = T - t
        j = 2.0 / (tj * T)
        s = 1 - (j * t_end**3) / 6
        s_dot = 0.5 * j * t_end**2
        s_ddot = -j * t_end
    
    return s, s_dot, s_ddot

class PathTrajectoryDemo:
    def __init__(self):
        # Define a simple 2D path (figure-8)
        self.path_parameter = np.linspace(0, 1, 200)
        self.path_x = 2 * np.sin(2 * np.pi * self.path_parameter)
        self.path_y = np.sin(4 * np.pi * self.path_parameter)
        
        # Time parameters
        self.T = 5.0  # Total time
        self.dt = 0.05
        self.time_array = np.arange(0, self.T, self.dt)
        
    def get_path_point(self, s):
        """Get (x, y) position on path at parameter s"""
        if s <= 0:
            return self.path_x[0], self.path_y[0]
        elif s >= 1:
            return self.path_x[-1], self.path_y[-1]
        
        # Linear interpolation
        idx = int(s * (len(self.path_parameter) - 1))
        if idx >= len(self.path_parameter) - 1:
            return self.path_x[-1], self.path_y[-1]
        
        alpha = s * (len(self.path_parameter) - 1) - idx
        x = (1 - alpha) * self.path_x[idx] + alpha * self.path_x[idx + 1]
        y = (1 - alpha) * self.path_y[idx] + alpha * self.path_y[idx + 1]
        return x, y

def main():
    # Create demo object
    demo = PathTrajectoryDemo()
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Path visualization
    ax1 = fig.add_subplot(221)
    ax1.set_title('Geometric Path (Independent of Time)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_xlim([-3, 3])
    ax1.set_ylim([-1.5, 1.5])
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    # Time scaling profiles
    ax2 = fig.add_subplot(222)
    ax2.set_title('Time Scaling Profiles s(t)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Path Parameter s')
    ax2.set_xlim([0, demo.T])
    ax2.set_ylim([0, 1.1])
    ax2.grid(True, alpha=0.3)
    
    # Velocity profiles
    ax3 = fig.add_subplot(223)
    ax3.set_title('Velocity Profiles ṡ(t)', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity ṡ')
    ax3.set_xlim([0, demo.T])
    ax3.set_ylim([0, 0.6])
    ax3.grid(True, alpha=0.3)
    
    # Acceleration profiles
    ax4 = fig.add_subplot(224)
    ax4.set_title('Acceleration Profiles s̈(t)', fontsize=14, fontweight='bold')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Acceleration s̈')
    ax4.set_xlim([0, demo.T])
    ax4.set_ylim([-0.5, 0.5])
    ax4.grid(True, alpha=0.3)
    
    # Draw static path
    ax1.plot(demo.path_x, demo.path_y, 'k-', linewidth=2, alpha=0.3)
    ax1.plot(demo.path_x[0], demo.path_y[0], 'go', markersize=10, label='Start')
    ax1.plot(demo.path_x[-1], demo.path_y[-1], 'ro', markersize=10, label='End')
    
    # Pre-compute all time scaling profiles
    profiles = {
        'Quintic': {'color': 'blue', 'data': []},
        'Trapezoidal': {'color': 'red', 'data': []},
        'S-curve': {'color': 'green', 'data': []}
    }
    
    for t in demo.time_array:
        s_q, v_q, a_q = quintic_time_scaling(demo.T, t)
        profiles['Quintic']['data'].append((s_q, v_q, a_q))
        
        s_t, v_t, a_t = trapezoidal_time_scaling(demo.T, t)
        profiles['Trapezoidal']['data'].append((s_t, v_t, a_t))
        
        s_s, v_s, a_s = s_curve_time_scaling(demo.T, t)
        profiles['S-curve']['data'].append((s_s, v_s, a_s))
    
    # Draw complete profiles
    for name, profile in profiles.items():
        s_vals = [d[0] for d in profile['data']]
        v_vals = [d[1] for d in profile['data']]
        a_vals = [d[2] for d in profile['data']]
        
        ax2.plot(demo.time_array, s_vals, profile['color'], 
                label=name, linewidth=2, alpha=0.6)
        ax3.plot(demo.time_array, v_vals, profile['color'], 
                label=name, linewidth=2, alpha=0.6)
        ax4.plot(demo.time_array, a_vals, profile['color'], 
                label=name, linewidth=2, alpha=0.6)
    
    ax2.legend(loc='upper left')
    ax3.legend(loc='upper right')
    ax4.legend(loc='upper right')
    
    # Animation elements
    markers = {}
    for name, profile in profiles.items():
        marker, = ax1.plot([], [], 'o', color=profile['color'], 
                          markersize=8, label=name)
        markers[name] = marker
    
    # Add path markers
    path_markers = {}
    for name, profile in profiles.items():
        trail, = ax1.plot([], [], '-', color=profile['color'], 
                         linewidth=1, alpha=0.3)
        path_markers[name] = trail
    
    # Time indicators
    time_lines = {
        'ax2': ax2.axvline(x=0, color='gray', linestyle='--', alpha=0.5),
        'ax3': ax3.axvline(x=0, color='gray', linestyle='--', alpha=0.5),
        'ax4': ax4.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
    }
    
    # Store trails
    trails = {name: {'x': [], 'y': []} for name in profiles.keys()}
    
    ax1.legend(loc='upper right')
    
    def init():
        return list(markers.values()) + list(path_markers.values()) + list(time_lines.values())
    
    def animate(frame):
        if frame >= len(demo.time_array):
            return []
        
        current_time = demo.time_array[frame]
        
        # Update time indicators
        for line in time_lines.values():
            line.set_xdata([current_time])
        
        # Update position markers
        for name, profile in profiles.items():
            s = profile['data'][frame][0]
            x, y = demo.get_path_point(s)
            
            markers[name].set_data([x], [y])
            
            # Update trail
            trails[name]['x'].append(x)
            trails[name]['y'].append(y)
            
            # Keep trail length limited
            if len(trails[name]['x']) > 30:
                trails[name]['x'].pop(0)
                trails[name]['y'].pop(0)
            
            path_markers[name].set_data(trails[name]['x'], trails[name]['y'])
        
        # Update time display
        ax1.set_title(f'Geometric Path (t = {current_time:.2f}s)', 
                     fontsize=14, fontweight='bold')
        
        # Add text annotations showing current values
        text_y = 1.3
        for name, profile in profiles.items():
            s, v, a = profile['data'][frame]
            ax1.text(-2.8, text_y, f'{name}: s={s:.3f}, ṡ={v:.3f}, s̈={a:.3f}',
                    fontsize=9, color=profile['color'])
            text_y -= 0.2
        
        return list(markers.values()) + list(path_markers.values()) + list(time_lines.values())
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, 
                        frames=len(demo.time_array) + 20,
                        interval=50, blit=True, repeat=True)
    
    plt.suptitle('Trajectory = Path + Time Scaling\n'
                 'Same geometric path, different time profiles → different trajectories',
                 fontsize=16, fontweight='bold')
    
    # Add mathematical note
    fig.text(0.5, 0.01, 
             'Key insight: Decoupling path from timing allows independent optimization\n'
             'Quintic: smooth but slow | Trapezoidal: time-optimal for velocity limits | '
             'S-curve: jerk-limited for comfort',
             ha='center', fontsize=10, style='italic')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
