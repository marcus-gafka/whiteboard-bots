"""Reads and writes the combined per-robot trajectory file: one row per
point, with the board coordinate, its IK solution, and the derived stepper
command — the single schema both the save side (after DXF processing) and
the load side (TXT-replay mode) must agree on."""

import os

_COLUMNS = ("x", "y", "theta1", "theta2", "steps1", "steps2", "interval")


def save_trajectory(rows, path):
    """rows: iterable of (x, y, theta1, theta2, steps1, steps2, interval)."""
    with open(path, "w") as f:
        f.write("\t".join(_COLUMNS) + "\n")
        for x, y, t1, t2, s1, s2, interval in rows:
            f.write(f"{x:.3f}\t{y:.3f}\t{t1:.3f}\t{t2:.3f}\t{s1}\t{s2}\t{interval}\n")
    print(f"\nSaved full trajectory + steps to: {path}")


def load_trajectory(path):
    """Returns (trajectory, commands):
    trajectory: list of (x, y, theta1_deg, theta2_deg)
    commands: list of (steps1, steps2, interval_ms)
    """
    trajectory, commands = [], []
    with open(path, "r") as f:
        for line in f.readlines()[1:]:  # skip header
            x, y, t1, t2, s1, s2, interval = line.strip().split("\t")
            trajectory.append((float(x), float(y), float(t1), float(t2)))
            commands.append((int(s1), int(s2), int(interval)))
    return trajectory, commands


def trajectory_file_path(folder, robot_class_name, source_name, point_count):
    base_name = f"{robot_class_name}_{source_name}_{point_count}"
    return os.path.join(folder, f"{base_name}.txt")
