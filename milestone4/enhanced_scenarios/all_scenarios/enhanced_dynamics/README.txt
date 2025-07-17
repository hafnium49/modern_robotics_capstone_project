Modern Robotics Capstone Project - Enhanced Dynamics
==============================================================

Scenario: Enhanced Dynamics
Description: Respondable chassis and enhanced physics in CoppeliaSim

Controller Type: Feedforward + PI Control (Enhanced Physics)
Proportional Gains (Kp): diag(np.int64(4), np.int64(4), np.int64(4), np.int64(4), np.int64(4), np.int64(4))
Integral Gains (Ki): diag(np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2))

Special Features:
  • Respondable youBot chassis
  • Block pushing with chassis
  • Enhanced contact physics
  • Realistic friction and restitution
  • Physics engine: bullet
  • Chassis mass: 20.0kg
  • Block mass: 0.2kg

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
