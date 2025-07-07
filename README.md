# modern_robotics_capstone_project

This repository contains a simple kinematic simulator for the Modern Robotics
capstone project.  The main function `NextState` propagates the robot
configuration forward one time step given wheel and joint commands.

The simulator requires Python 3.10+ and the `modern_robotics` library. Install
all dependencies using `pip install -r requirements.txt`.

The `driver` module can generate CSV files compatible with the CoppeliaSim
scenes.  Example usage for the first sanity check scenario is:

```bash
python -m modern_robotics_sim.driver initial_config.csv output.csv \
    --controls 10 10 10 10 0 0 0 0 0 --speed-limit 20
```

The package also exposes a `TrajectoryGenerator` utility for the
pick-and-place task in later milestones. It outputs an `N x 13` NumPy
array that can be written directly to a CSV file for use with the
CoppeliaSim scenes.

Unit tests verify the original `NextState` behaviour as well as the
new trajectory generator. Run the tests with:

```bash
pytest -q
```
