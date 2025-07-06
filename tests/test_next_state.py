import os
import sys
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from modern_robotics_sim.next_state import NextState


def run_for_one_second(controls, speed_limit=20.0):
    config = np.zeros(12)
    dt = 0.01
    for _ in range(100):
        config = NextState(config, controls, dt, speed_limit)
    return config


def test_forward_x():
    controls = [10, 10, 10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[1], 0.475, atol=1e-3)


def test_forward_y():
    controls = [-10, 10, -10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[2], 0.475, atol=1e-3)


def test_rotation():
    controls = [-10, 10, 10, -10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[0], 1.234, atol=1e-3)
