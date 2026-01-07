import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class Plotter2D:
    def __init__(self, robot, trajectory, commands, show_trace=True, show_manipulability=True, Nx=200, Ny=200):
        self.robot = robot
        self.trajectory = trajectory
        self.commands = commands
        self.show_trace = show_trace
        self.show_manipulability = show_manipulability

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(robot.MIN_PLOT_X, robot.MAX_PLOT_X)
        self.ax.set_ylim(robot.MIN_PLOT_Y, robot.MAX_PLOT_Y)
        self.ax.grid(True)

        # Robot links
        self.lines = [self.ax.plot([], [], 'o-', lw=3)[0] for _ in range(4)]

        # Trace of end-effector
        if self.show_trace:
            self.trace_x = []
            self.trace_y = []
            self.trace_line, = self.ax.plot([], [], 'r--', lw=1)

        # Manipulability background
        if self.show_manipulability:
            self.X_grid, self.Y_grid = np.meshgrid(
                np.linspace(0, robot.DRAWABLE_WIDTH, Nx),
                np.linspace(0, robot.DRAWABLE_HEIGHT, Ny)
            )
            self.manip_grid = np.zeros_like(self.X_grid)
            for i in range(Nx):
                for j in range(Ny):
                    board_x, board_y = self.X_grid[j, i], self.Y_grid[j, i]
                    try:
                        # Convert to robot coordinates
                        rx, ry = robot.apply_board_offset(board_x, board_y)

                        t1, t2 = robot.ik(board_x, board_y)
                        self.manip_grid[j, i] = robot.manipulability(t1, t2)

                        # Overwrite grid points to robot coords
                        self.X_grid[j, i] = rx
                        self.Y_grid[j, i] = ry

                    except:
                        self.manip_grid[j, i] = 0.0

            self.heatmap = self.ax.imshow(
                self.manip_grid,
                extent=(
                    np.min(self.X_grid), np.max(self.X_grid),
                    np.min(self.Y_grid), np.max(self.Y_grid)
                ),
                origin="lower",
                cmap="viridis",
                alpha=0.6,
                interpolation="bilinear"
            )
            plt.colorbar(self.heatmap, ax=self.ax, label="Manipulability")


    def init_func(self):
        for line in self.lines:
            line.set_data([], [])
        if self.show_trace:
            self.trace_line.set_data([], [])

        box_segments = self.robot.get_drawable_box()
        xs = [p[0] for seg in box_segments for p in seg]
        ys = [p[1] for seg in box_segments for p in seg]
        self.box_line, = self.ax.plot(xs, ys, 'k--', lw=1)

        return self.lines + ([self.trace_line] if self.show_trace else []) + [self.box_line]

    def update_func(self, frame):
        _, _, theta1, theta2 = self.trajectory[frame]
        links = self.robot.get_links(theta1, theta2)

        for line, (p1, p2) in zip(self.lines, links):
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

        if self.show_trace:
            ee_pos = links[0][0]
            self.trace_x.append(ee_pos[0])
            self.trace_y.append(ee_pos[1])
            self.trace_line.set_data(self.trace_x, self.trace_y)

        return self.lines + ([self.trace_line] if self.show_trace else []) + [self.box_line]

    def animate(self, blit=True):
        def frame_update(frame):
            interval = self.commands[frame][2]
            self.ani.event_source.interval = interval
            return self.update_func(frame)

        self.ani = FuncAnimation(
            self.fig,
            frame_update,
            frames=len(self.trajectory),
            init_func=self.init_func,
            blit=blit
        )
        return self.ani
