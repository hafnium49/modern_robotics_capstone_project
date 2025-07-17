Modern Robotics Capstone Project - Joint Limit Enforcement
====================================================================

Scenario: Joint Limit Enforcement
Description: Demonstrates safe operation with joint limit constraints

Controller Type: Feedforward + PI Control (Joint Limits)
Proportional Gains (Kp): diag(np.int64(3), np.int64(3), np.int64(3), np.int64(3), np.int64(3), np.int64(3))
Integral Gains (Ki): diag(np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1))

Special Features:
  • Joint limits enforced during control
  • Safety margins applied to prevent limit violations
  • Graceful handling of constrained motions
  • Extended task with challenging joint configurations

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
