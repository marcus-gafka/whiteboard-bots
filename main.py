import ezdxf
import time
import os
import matplotlib.pyplot as plt

from Constants import Constants as c
from paths import DxfExtraction as dxf_extraction
from paths import GeometryTransform as geometry_transform
from paths import PathOrdering as path_ordering
from paths import ManualOrder as manual_order
from paths import PointsIO as points_io
from paths import TrajectoryIO as trajectory_io
from robots.FiveBar import FiveBar
from robots.Spooler import Spooler
from robots.TwoLink import TwoLink
from comms.SerialComms import SerialComms
from comms.NetworkComms import NetworkComms
from comms.PsuedoComms import PsuedoComms
from Plotter import Plotter2D
from core.pipeline import build_trajectory

ROBOT_CLASSES = {"1": FiveBar, "2": Spooler, "3": TwoLink}
COMMS_CLASSES = {"1": SerialComms, "2": NetworkComms}


class FatalInputError(Exception):
    """Raised when a stage hits a condition the program can't recover from
    within the same run (e.g. no input files available at all) — distinct
    from an invalid prompt answer, which just re-runs the outer loop."""


def select_file(folder, ext, robot_name=None, preselection=False, file_index=None):
    # Filter files by extension (and optionally robot name)
    files = [f for f in os.listdir(folder) if f.endswith(ext)]
    if robot_name:
        files = [f for f in files if f.startswith(robot_name)]
    if not files:
        print(f"No {ext} files found in {folder}")
        return None, None

    # Sort alphabetically for display
    files.sort()
    for i, f in enumerate(files):
        print(f"{i+1}: {f}")

    # Determine selection
    if preselection:
        # Use file_index as if it were user input
        user_input = str(file_index) if file_index is not None else "0"
    else:
        user_input = input(f"Enter file number (0 = most recent, blank = most recent): ").strip()
        if user_input == "":
            user_input = "0"

    # Handle "0" = most recent file
    if user_input == "0":
        files.sort(key=lambda f: os.path.getmtime(os.path.join(folder, f)), reverse=True)
        index = 0
    else:
        try:
            index = int(user_input) - 1
            if index < 0 or index >= len(files):
                raise ValueError
        except ValueError:
            print("Invalid input, defaulting to most recent file")
            files.sort(key=lambda f: os.path.getmtime(os.path.join(folder, f)), reverse=True)
            index = 0

    path = os.path.join(folder, files[index])
    source_name = os.path.splitext(os.path.basename(path))[0]
    return path, source_name


def select_robot():
    """Always prompts (this is also where "0" enters preselection mode for
    every later stage). Returns (robot, preselection), or None on invalid input."""
    print("Choose robot: (0 = Preselection)")
    print("1 - FiveBar")
    print("2 - Spooler")
    print("3 - TwoLink")
    temp_input = input("Enter 0, 1, 2, 3:").strip()

    if temp_input == "0":
        return ROBOT_CLASSES["2"](), True
    if temp_input in ROBOT_CLASSES:
        return ROBOT_CLASSES[temp_input](), False

    print("Invalid input.")
    return None


def select_input_and_order_mode(preselection):
    """Returns (input_mode, order_mode), or None on invalid input.
    Skipped (hardcoded defaults) when preselection is True."""
    if preselection:
        return "1", "1"

    print("\nChoose mode:")
    print("1 - Run from DXF file")
    print("2 - Run from saved TXT coordinates")
    input_mode = input("Enter 1 or 2: ").strip()
    if input_mode not in {"1", "2"}:
        print("Invalid input.")
        return None

    if input_mode != "1":
        return input_mode, "1"

    print("\nChoose geometry ordering method:")
    print("1 - Automatic (nearest neighbor)")
    print("2 - Manual")
    order_mode = input("Enter 1 or 2: ").strip()
    if order_mode not in {"1", "2"}:
        print("Invalid input.")
        return None

    return input_mode, order_mode


def prompt_point_count(points):
    """Returns the (possibly reduced) point list, or None on invalid input."""
    user_input = input("Enter desired quantity of points (blank = all): ").strip()
    if user_input == "":
        return points
    try:
        n = int(user_input)
        if n < 1 or n > len(points):
            print("Invalid input.")
            return None
        return path_ordering.reduce_points(points, n)
    except ValueError:
        print("Invalid input.")
        return None


def run_dxf_pipeline(robot, preselection, order_mode, file_index):
    """Returns (filtered_points, trajectory, commands), or None if the run
    should be retried (an invalid answer partway through)."""
    dxf_path, source_name = select_file(c.DXF_FOLDER, ".dxf", preselection=preselection, file_index=file_index)
    if not dxf_path:
        raise FatalInputError("no DXF files available")
    print(f"\nUsing DXF: {dxf_path}")

    doc = ezdxf.readfile(dxf_path)
    geometries, lines, circles, arcs, polylines = dxf_extraction.extract_geometries(doc, dxf_extraction.INITIAL_SPACING)
    print(f"\nExtracted {len(geometries)} geometries: {lines} lines, {circles} circles, {arcs} arcs, {polylines} polylines")

    all_points = [pt for geometry in geometries for pt in geometry]
    print("\nBounding box BEFORE scaling & translation:")
    geometry_transform.print_bounding_box(all_points)

    scaled_geometries = geometry_transform.scale_and_translate_geometries(robot, geometries)
    scaled_all_points = [pt for geometry in scaled_geometries for pt in geometry]
    print("\nBounding box AFTER scaling & translation:")
    geometry_transform.print_bounding_box(scaled_all_points)

    if order_mode == "2":
        try:
            ordered_geometries = manual_order.prompt_manual_order(scaled_geometries, robot)
        except manual_order.ManualOrderAborted:
            print("Manual ordering window closed — aborting this run.")
            return None
    else:
        ordered_geometries = path_ordering.tsp_geometry_order(robot, scaled_geometries)
    ordered_all_points = [pt for g in ordered_geometries for pt in g]

    print(f"\nTotal points before filtering: {len(ordered_all_points)}")
    filtered_points = prompt_point_count(ordered_all_points)
    if filtered_points is None:
        return None
    print(f"Total points after filtering: {len(filtered_points)}")

    points_io.save_points(filtered_points, dxf_path, folder=c.TXT_FOLDER)

    trajectory, commands = build_trajectory(robot, filtered_points)
    combined_path = trajectory_io.trajectory_file_path(
        c.TXT_FOLDER, robot.__class__.__name__, source_name, len(filtered_points)
    )
    trajectory_io.save_trajectory(
        [(x, y, t1, t2, s1, s2, interval)
         for (x, y, t1, t2), (s1, s2, interval) in zip(trajectory, commands)],
        combined_path,
    )

    return filtered_points, trajectory, commands


def run_txt_pipeline(robot, preselection, file_index):
    txt_path, _source_name = select_file(
        c.TXT_FOLDER, ".txt", robot_name=robot.name, preselection=preselection, file_index=file_index
    )
    if not txt_path:
        raise FatalInputError("no saved trajectory files available")

    trajectory, commands = trajectory_io.load_trajectory(txt_path)
    filtered_points = [(x, y) for x, y, _, _ in trajectory]
    return filtered_points, trajectory, commands


def show_preview_plot(robot, filtered_points):
    fig, ax = plt.subplots()
    plt.ion()
    fig.set_size_inches(robot.DRAWABLE_WIDTH, robot.DRAWABLE_HEIGHT)
    ax.set_xlim(0, robot.DRAWABLE_WIDTH)
    ax.set_ylim(0, robot.DRAWABLE_HEIGHT)
    ax.set_aspect("equal")

    points_plot, = ax.plot(*zip(*filtered_points), "bo", label="Points")
    path_plot, = ax.plot(*zip(*([(0, 0)] + filtered_points)), "r-", linewidth=2, label="Path")
    points_plot.set_visible(False)
    path_plot.set_visible(False)
    plt.legend()
    plt.show()

    input("Press Enter to show points...")
    points_plot.set_visible(True)
    fig.canvas.draw()

    input("Press Enter to show path...")
    path_plot.set_visible(True)
    fig.canvas.draw()


def init_comms(comms_mode):
    if comms_mode in COMMS_CLASSES:
        return COMMS_CLASSES[comms_mode]()
    if comms_mode in ("3", ""):
        return PsuedoComms()
    print("Invalid comms method.")
    return None


def dispatch_commands(comms, commands):
    input("Press Enter to start execution...")
    comms.prepare()
    for s1, s2, interval in commands:
        comms.send_steps(s1, s2, interval)


def animate_run(robot, trajectory, commands):
    plt.close()
    plt.ioff()

    plotter = Plotter2D(robot, trajectory, commands, show_trace=True, show_manipulability=True,
                        show_ellipse=True)
    plotter.ax.set_xlim(robot.MIN_PLOT_X, robot.MAX_PLOT_X)
    plotter.ax.set_ylim(robot.MIN_PLOT_Y, robot.MAX_PLOT_Y)
    plotter.ani = plotter.animate(blit=True)
    plt.show()
    plt.ion()
    print("DONE!")


def main():
    while True:
        file_index = "5"
        # comms-method prompt is intentionally not offered; comms_mode stays
        # fixed at Psuedo (dry-run, no hardware) below.
        comms_mode = "3"

        robot_selection = select_robot()
        if robot_selection is None:
            continue
        robot, preselection = robot_selection

        mode_selection = select_input_and_order_mode(preselection)
        if mode_selection is None:
            continue
        input_mode, order_mode = mode_selection

        try:
            if input_mode == "1":
                pipeline_result = run_dxf_pipeline(robot, preselection, order_mode, file_index)
            else:
                pipeline_result = run_txt_pipeline(robot, preselection, file_index)
        except FatalInputError:
            return

        if pipeline_result is None:
            continue
        filtered_points, trajectory, commands = pipeline_result

        show_preview_plot(robot, filtered_points)

        comms = init_comms(comms_mode)
        if comms is None:
            return

        dispatch_commands(comms, commands)
        animate_run(robot, trajectory, commands)

        restart = input("Plot closed. Do you want to restart the program? [y/n]: ").strip().lower()
        if restart != "y":
            break

    print("Program exited.")


if __name__ == "__main__":
    main()
