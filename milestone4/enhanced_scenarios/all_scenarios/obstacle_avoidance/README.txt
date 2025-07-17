Modern Robotics Capstone Project - Obstacle Avoidance
===============================================================

Scenario: Obstacle Avoidance
Description: Collision-free motion planning around workspace obstacles

Controller Type: Feedforward + PI Control (Obstacle Avoidance)
Proportional Gains (Kp): diag(np.int64(4), np.int64(4), np.int64(4), np.int64(4), np.int64(4), np.int64(4))
Integral Gains (Ki): diag(np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1))

Special Features:
  • Number of obstacles: 3
  • Path length: 1.00m
  • Safety margin: 0.15m
  • Waypoints: 21
  • RRT-style path planning
  • Collision detection and avoidance

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
