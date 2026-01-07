import ezdxf
import time
import os
import matplotlib.pyplot as plt

from Constants import Constants as c
from paths.PathProcessor import PathProcessor as p
from robots.FiveBar import FiveBar
from robots.Spooler import Spooler
from robots.TwoLink import TwoLink
from comms.SerialComms import SerialComms
from comms.NetworkComms import NetworkComms
from comms.PsuedoComms import PsuedoComms
from Plotter import Plotter2D

import os

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

def main():
    while True:
        # Preselection
        preselection = False
        robot_mode = "2"
        input_mode = "1"
        comms_mode = "3"
        file_index = "5"

        # =======================
        # Robot selection
        # =======================
        print("Choose robot: (0 = Preselection)")
        print("1 - FiveBar")
        print("2 - Spooler")
        print("3 - TwoLink")
        temp_input = input("Enter 0, 1, 2, 3:").strip()

        if temp_input == "0":
            preselection = True
        else:
            robot_mode = temp_input
            temp_input = "1"
        
        if robot_mode == "1":
            robot = FiveBar()
        elif robot_mode == "2":
            robot = Spooler()
        elif robot_mode == "3":
            robot = TwoLink()
        else:
            print("Invalid robot selected.")
            return

        # =======================
        # Input selection
        # =======================
        if not preselection:
            print("\nChoose mode:")
            print("1 - Run from DXF file")
            print("2 - Run from saved TXT coordinates")
            input_mode = input("Enter 1 or 2: ").strip()
            if input_mode not in {"1", "2"}:
                print("Invalid mode.")
                return

        # =======================
        # Load points / trajectory
        # =======================
        if input_mode == "1":  # DXF mode
            dxf_path, source_name = select_file(c.DXF_FOLDER, ".dxf", preselection=preselection, file_index=file_index)
            if not dxf_path:
                return
            print(f"\nUsing DXF: {dxf_path}")

            # --- Extract geometries ---
            doc = ezdxf.readfile(dxf_path)
            geometries, lines, circles, arcs, polylines = p.extract_geometries(doc, p.INITIAL_SPACING)
            print(f"\nExtracted {len(geometries)} geometries: {lines} lines, {circles} circles, {arcs} arcs, {polylines} polylines")

            all_points = [pt for geometry in geometries for pt in geometry]

            # --- Bounding box BEFORE scaling & translation ---
            print("\nBounding box BEFORE scaling & translation:")
            p.print_bounding_box(all_points)

            # --- Scale and translate to robot ---
            scaled_geometries = p.scale_and_translate_geometries(robot, geometries)
            scaled_all_points = [pt for geometry in scaled_geometries for pt in geometry]

            # --- Bounding box AFTER scaling & translation ---
            print("\nBounding box AFTER scaling & translation:")
            p.print_bounding_box(scaled_all_points)

            # --- Order geometries ---
            ordered_geometries = p.tsp_geometry_order(scaled_geometries)
            ordered_all_points = [pt for g in ordered_geometries for pt in g]

            print(f"\nTotal points before filtering: {len(ordered_all_points)}")

            # --- Filter points ---
            user_input = input("Enter desired quantity of points (blank = all): ").strip()
            filtered_points = ordered_all_points if user_input == "" else p.reduce_points(ordered_all_points, int(user_input))
            print(f"Total points after filtering: {len(filtered_points)}")

            # --- Save points ---
            p.save_points(filtered_points, dxf_path, folder=c.TXT_FOLDER)

            # --- Generate trajectory + commands ---
            trajectory = []
            commands = []
            currentTheta1, currentTheta2 = robot.ik(robot.START_X, robot.START_Y)
            base_name = f"{robot.__class__.__name__}_{source_name}_{len(filtered_points)}"
            combined_path = os.path.join(c.TXT_FOLDER, f"{base_name}.txt")

            with open(combined_path, "w") as f:
                f.write("x\ty\ttheta1\ttheta2\tsteps1\tsteps2\tinterval\n")
                for x, y in filtered_points:
                    t1, t2 = robot.ik(x, y)
                    s1, s2, interval = robot.calc_steps(currentTheta1, currentTheta2, t1, t2)
                    trajectory.append((x, y, t1, t2))
                    commands.append((s1, s2, interval))
                    f.write(f"{x:.3f}\t{y:.3f}\t{t1:.3f}\t{t2:.3f}\t{s1}\t{s2}\t{interval}\n")
                    currentTheta1, currentTheta2 = t1, t2

            print(f"\nSaved full trajectory + steps to: {combined_path}")

        else:  # TXT mode
            txt_path, source_name = select_file(c.TXT_FOLDER, ".txt", robot_name=robot.name, preselection=preselection, file_index=file_index)
            if not txt_path:
                return

            trajectory = []
            commands = []

            with open(txt_path, "r") as f:
                lines = f.readlines()[1:]  # skip header
                for l in lines:
                    x, y, t1, t2, s1, s2, interval = l.strip().split("\t")
                    trajectory.append((float(x), float(y), float(t1), float(t2)))
                    commands.append((int(s1), int(s2), int(interval)))

            filtered_points = [(x, y) for x, y, _, _ in trajectory]

        # =======================
        # Plot setup
        # =======================
        fig, ax = plt.subplots()
        plt.ion()
        fig.set_size_inches(robot.DRAWABLE_WIDTH, robot.DRAWABLE_HEIGHT)
        ax.set_xlim(0, robot.DRAWABLE_WIDTH)
        ax.set_ylim(0, robot.DRAWABLE_HEIGHT)
        ax.set_aspect("equal")

        points_plot, = ax.plot(*zip(*filtered_points), "bo", label="Points")
        path_plot, = ax.plot(*zip(*([(0,0)] + filtered_points)), "r-", linewidth=2, label="Path")
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

        #if not preselection:
        if False:
            print("\nChoose comms method:")
            print("1 - Serial")
            print("2 - Network")
            print("3 - Psuedo")
            comms_mode = input("Enter 1, 2, or 3: ").strip()

        # Initialize comms
        if comms_mode == "1":
            comms = SerialComms()
        elif comms_mode == "2":
            comms = NetworkComms()
        elif comms_mode == "3" or "":
            comms = PsuedoComms()
        else:
            print("Invalid comms method.")
            return

        # =======================
        # Send commands
        # =======================
        input("Press Enter to start execution...")

        if comms_mode == "1":
            time.sleep(2)
            comms.ser.flush()
        elif comms_mode == "2":
            time.sleep(0.5)

        for s1, s2, interval in commands:
            comms.send_steps(s1, s2, interval)

        # =======================
        # Animate
        # =======================
        plt.close()
        plt.ioff()  # turn off interactive mode

        plotter = Plotter2D(robot, trajectory, commands, show_trace=True, show_manipulability=True)
        plotter.ax.set_xlim(robot.MIN_PLOT_X, robot.MAX_PLOT_X)
        plotter.ax.set_ylim(robot.MIN_PLOT_Y, robot.MAX_PLOT_Y)
        plotter.ani = plotter.animate(blit=True)
        plt.show()
        plt.ion()
        print("DONE!")

        restart = input("Plot closed. Do you want to restart the program? [y/n]: ").strip().lower()
        if restart != "y":
            break

    print("Program exited.")

if __name__ == "__main__":
    main()
