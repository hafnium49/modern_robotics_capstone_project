Modern Robotics Capstone Project - Motion Preference - Prefer Wheels Concept
======================================================================================

Scenario: Motion Preference - Prefer Wheels Concept
Description: Demonstrates concept of preferring wheel motions via weighted pseudoinverse

Controller Type: Feedforward + PI Control (Wheel Preference Concept)
Proportional Gains (Kp): diag(np.int64(3), np.int64(3), np.int64(3), np.int64(3), np.int64(3), np.int64(3))
Integral Gains (Ki): diag(np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2), np.float64(0.2))

Special Features:
  • CONCEPT: Weighted pseudoinverse with joints=0.3, wheels=2.0
  • Lower arm gains simulate preference for wheel motions
  • Mobile base would move more to achieve end-effector goals
  • Implementation would use: J_pinv = weighted_pseudoinverse(J, 0.3, 2.0)

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
