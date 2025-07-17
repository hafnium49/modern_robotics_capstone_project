import os
import sys
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from code.next_state import NextState


def run_for_one_second(controls, speed_limit=20.0):
    config = np.zeros(12)
    dt = 0.01
    for _ in range(100):
        config = NextState(config, controls, dt, speed_limit)
    return config


def test_forward_x():
    """Test 1: u = (10, 10, 10, 10) - robot drives forward in +x̂_b direction by 0.475m"""
    controls = [10, 10, 10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[1], 0.475, atol=1e-3)  # x displacement
    assert np.allclose(cfg[8], 10.0, atol=1e-6)   # wheel angle updated


def test_forward_y():
    """Test 2: u = (-10, 10, -10, 10) - robot slides sideways in +ŷ_b direction by 0.475m"""
    controls = [-10, 10, -10, 10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[2], 0.475, atol=1e-3)  # y displacement


def test_rotation():
    """Test 3: u = (-10, 10, 10, -10) - robot spins counter-clockwise by 1.234 radians"""
    controls = [-10, 10, 10, -10] + [0]*5
    cfg = run_for_one_second(controls)
    assert np.allclose(cfg[0], 1.234, atol=1e-3)  # φ rotation


def test_speed_limit():
    """Test 4: Speed limit test - with limit=5, robot should move half distance (0.2375m)"""
    controls = [10, 10, 10, 10] + [0]*5
    cfg = run_for_one_second(controls, speed_limit=5.0)
    assert np.allclose(cfg[1], 0.2375, atol=1e-3)  # half of 0.475m
