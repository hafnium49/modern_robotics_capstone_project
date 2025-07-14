"""
Modern Robotics Simulation Package

This package contains implementations for the Modern Robotics capstone project:
- Milestone 1: NextState kinematic simulator
- Milestone 2: TrajectoryGenerator for pick-and-place paths  
- Milestone 3: FeedbackControl for task-space control
- Milestone 4: Complete system integration and scenarios
"""

from .next_state import NextState
from .trajectory_generator import (
    TrajectoryGenerator,
    OPEN_STATE,
    CLOSED_STATE,
    DT_REF,
    segment_time,
)
from .feedback_control import FeedbackControl, FeedbackController
from .run_capstone import (
    create_default_cube_poses, create_grasp_transforms, create_initial_ee_pose,
    create_initial_config_with_error, run_capstone_simulation
)
from .scenarios import (
    run_feedforward_only_scenario, run_proportional_only_scenario,
    run_pi_only_scenario, run_feedforward_plus_pi_scenario,
    run_best_scenario, run_overshoot_scenario, run_newTask_scenario,
    analyze_simulation_results, create_gain_study
)

__all__ = [
    "NextState",
    "TrajectoryGenerator",
    "OPEN_STATE",
    "CLOSED_STATE", 
    "DT_REF",
    "segment_time",
    "FeedbackControl", 
    "FeedbackController",
    "create_default_cube_poses", 
    "create_grasp_transforms",
    "create_initial_ee_pose",
    "create_initial_config_with_error", 
    "run_capstone_simulation",
    "run_feedforward_only_scenario",
    "run_proportional_only_scenario",
    "run_pi_only_scenario", 
    "run_feedforward_plus_pi_scenario",
    "run_best_scenario", 
    "run_overshoot_scenario", 
    "run_newTask_scenario",
    "analyze_simulation_results", 
    "create_gain_study"
]
