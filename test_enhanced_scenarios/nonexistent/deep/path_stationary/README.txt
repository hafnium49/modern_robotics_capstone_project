Modern Robotics Capstone Project - Stationary Base Manipulation
=========================================================================

Scenario: Stationary Base Manipulation
Description: Demonstrates concept of keeping mobile base stationary during manipulation

Controller Type: Feedforward + PI Control (Stationary Base Concept)
Proportional Gains (Kp): diag(np.int64(6), np.int64(6), np.int64(6), np.int64(6), np.int64(6), np.int64(6))
Integral Gains (Ki): diag(np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1), np.float64(0.1))

Special Features:
  • CONCEPT: Base would be stationary during segments 2, 4, 6, 8
  • Enhanced manipulation precision through reduced base motion
  • Arm-dominated motion during grasping operations
  • This demo uses standard control but documents the approach

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
