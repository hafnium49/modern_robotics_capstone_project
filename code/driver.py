import argparse
import csv
import numpy as np

try:
    from .next_state import NextState
except ImportError:
    from next_state import NextState


def parse_args():
    p = argparse.ArgumentParser(description="Run NextState for multiple steps and output CSV")
    p.add_argument('initial', help='CSV file with a single line of 13 values (config + gripper)')
    p.add_argument('output', help='output CSV path')
    p.add_argument('--controls', type=float, nargs=9, required=True,
                   metavar=('u1','u2','u3','u4','th1','th2','th3','th4','th5'))
    p.add_argument('--dt', type=float, default=0.01)
    p.add_argument('--speed-limit', type=float, default=20.0)
    p.add_argument('--steps', type=int, default=100)
    p.add_argument('--gripper', type=int, choices=[0, 1], default=0,
                   help='gripper open (0) or closed (1)')
    return p.parse_args()


def load_initial(path):
    with open(path, 'r') as f:
        row = next(csv.reader(f))
    values = [float(v) for v in row]
    if len(values) != 13:
        raise ValueError('initial configuration must have 13 values')
    return np.array(values[:12])


def main():
    args = parse_args()
    config = load_initial(args.initial)
    controls = np.array(args.controls, dtype=float)

    out_rows = [list(config) + [args.gripper]]
    for _ in range(args.steps):
        config = NextState(config, controls, args.dt, args.speed_limit)
        out_rows.append(list(config) + [args.gripper])

    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(out_rows)


if __name__ == '__main__':
    main()
