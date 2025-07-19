Modern Robotics Capstone Project - Singularity Avoidance
==================================================================

Scenario: Singularity Avoidance
Description: Robust control behavior near singular configurations

Controller Type: Feedforward + PI Control (Singularity Robust)
Proportional Gains (Kp): diag(np.int64(2), np.int64(2), np.int64(2), np.int64(2), np.int64(2), np.int64(2))
Integral Gains (Ki): diag(np.float64(0.05), np.float64(0.05), np.float64(0.05), np.float64(0.05), np.float64(0.05), np.float64(0.05))

Special Features:
  • Damped least squares inverse near singularities
  • Manipulability monitoring
  • Robust control through singular configurations
  • Graceful degradation in ill-conditioned poses

This scenario demonstrates advanced capabilities from the
'Other Things to Try' section of the capstone requirements.

Files in this directory:
  • README.txt - This description
  • youBot_output.csv - Robot trajectory for CoppeliaSim
  • Xerr_log.csv - End-effector error log
  • Xerr_plot.pdf - Error convergence visualization
  • program_log.txt - Execution log with details
