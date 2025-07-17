Modern Robotics Capstone Project - Block Throwing
===========================================================

Scenario: Block Throwing
Description: Ballistic trajectory planning concept for throwing block to target

Controller Type: Feedforward + P Control (Throwing Concept)
Proportional Gains (Kp): diag(np.int64(8), np.int64(8), np.int64(8), np.int64(8), np.int64(8), np.int64(8))
Integral Gains (Ki): diag(np.int64(0), np.int64(0), np.int64(0), np.int64(0), np.int64(0), np.int64(0))

Special Features:
  • Target landing point: [2.  1.5]
  • Release height: 0.8m
  • Required velocity: 6.19m/s
  • Flight time: 0.40s
  • CONCEPT: Ballistic physics calculation
  • CONCEPT: Dynamic gripper release timing
  • This is the 'fun' scenario from capstone requirements!

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
