"""File I/O for the plain (x, y) points file produced after ordering."""

import os
import pathlib


def save_points(points, dxf_filename, folder):
    os.makedirs(folder, exist_ok=True)
    base_name = "_" + pathlib.Path(dxf_filename).stem
    filename = f"{base_name}_{len(points)}.txt"
    full_path = os.path.join(folder, filename)

    with open(full_path, "w") as f:
        f.write("x\ty\n")
        for x, y in points:
            f.write(f"{x:.3f}\t{y:.3f}\n")

    print(f"\nSaved points to: {full_path}")
    return full_path
