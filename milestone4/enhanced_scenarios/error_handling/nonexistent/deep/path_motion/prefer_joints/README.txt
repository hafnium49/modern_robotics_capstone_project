Modern Robotics Capstone Project - Motion Preference - Prefer Joints Concept
======================================================================================

Scenario: Motion Preference - Prefer Joints Concept
Description: Demonstrates concept of preferring joint motions via weighted pseudoinverse

Controller Type: Feedforward + PI Control (Joint Preference Concept)
Proportional Gains (Kp): diag(np.int64(8), np.int64(8), np.int64(8), np.int64(8), np.int64(8), np.int64(8))
Integral Gains (Ki): diag(np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1))

Special Features:
  • CONCEPT: Weighted pseudoinverse with joints=2.0, wheels=0.3
  • Higher arm gains simulate preference for joint motions
  • Arm joints would move more to achieve end-effector goals
  • Implementation would use: J_pinv = weighted_pseudoinverse(J, 2.0, 0.3)

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
