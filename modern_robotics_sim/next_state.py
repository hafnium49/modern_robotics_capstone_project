import numpy as np
from modern_robotics import VecTose3, MatrixExp6


def NextState(config, controls, dt, speed_limit):
    """Advance the robot configuration by one time step."""
    config = np.asarray(config, dtype=float)
    controls = np.asarray(controls, dtype=float)

    if config.shape != (12,):
        raise ValueError("config must be length 12")
    if controls.shape != (9,):
        raise ValueError("controls must be length 9")

    # Constants
    r = 0.0475
    l = 0.235
    w = 0.15
    h = 0.0963

    # Clip control inputs
    controls_clipped = np.clip(controls, -speed_limit, speed_limit)
    u = controls_clipped[:4]
    thetadot = controls_clipped[4:]

    # Current configuration
    phi = config[0]
    x = config[1]
    y = config[2]
    theta = config[3:8]
    wheels = config[8:]

    # Chassis kinematics
    F = (r / 4) * np.array([
        [-1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w)],
        [     1  ,       1  ,       1  ,      1 ],
        [    -1  ,       1  ,      -1  ,      1 ],
    ])

    Vb = F @ u
    # 6D body twist [wx, wy, wz, vx, vy, vz]
    Vb6 = np.array([0.0, 0.0, Vb[0], Vb[1], Vb[2], 0.0])

    Tsb = np.array([
        [np.cos(phi), -np.sin(phi), 0, x],
        [np.sin(phi),  np.cos(phi), 0, y],
        [0,            0,           1, h],
        [0,            0,           0, 1],
    ])

    Tsb_new = Tsb @ MatrixExp6(VecTose3(Vb6 * dt))

    phi_new = np.arctan2(Tsb_new[1, 0], Tsb_new[0, 0])
    x_new = Tsb_new[0, 3]
    y_new = Tsb_new[1, 3]

    theta_new = theta + thetadot * dt
    wheels_new = wheels + u * dt

    return np.concatenate(([phi_new, x_new, y_new], theta_new, wheels_new))
