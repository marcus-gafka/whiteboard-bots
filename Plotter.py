import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse

from robots.ManipulabilityGrid import ManipulabilityGrid


class Plotter2D:
    """Animates a robot tracing `trajectory` over its drawable board, with
    an optional end-effector trace and manipulability heatmap background.

    trajectory: list of (x, y, theta1_deg, theta2_deg) tuples, one per frame.
    commands: list of (steps1, steps2, interval_ms) tuples, same length as
        trajectory — only `interval_ms` (index 2) is used here, to pace
        each animation frame to match the real move duration.
    """

    LINK_STYLE = "o-"
    LINK_LINEWIDTH = 3
    TRACE_STYLE = "r--"
    TRACE_LINEWIDTH = 1
    BOX_STYLE = "k--"
    BOX_LINEWIDTH = 1
    HEATMAP_CMAP = "viridis"
    HEATMAP_ALPHA = 0.6
    DEFAULT_GRID_RESOLUTION = 200
    ELLIPSE_EDGE_COLOR = "magenta"
    ELLIPSE_LINEWIDTH = 2
    # Largest semi-axis seen along the trajectory is drawn at this fraction
    # of the plot's larger dimension (J has units of length/rad, which don't
    # map meaningfully onto board length, so the ellipse is auto-scaled).
    ELLIPSE_MAX_FRACTION = 0.12

    def __init__(self, robot, trajectory, commands, show_trace=True, show_manipulability=True,
                 show_ellipse=False, Nx=DEFAULT_GRID_RESOLUTION, Ny=DEFAULT_GRID_RESOLUTION):
        self.robot = robot
        self.trajectory = trajectory
        self.commands = commands
        self.show_trace = show_trace
        self.show_manipulability = show_manipulability
        self.show_ellipse = show_ellipse

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(robot.MIN_PLOT_X, robot.MAX_PLOT_X)
        self.ax.set_ylim(robot.MIN_PLOT_Y, robot.MAX_PLOT_Y)
        self.ax.grid(True)

        # Every instance attribute is declared here (even when unused by the
        # current flags) so the object's shape is knowable from __init__
        # alone, rather than materializing attributes conditionally.
        self.trace_x = []
        self.trace_y = []
        self.trace_line = None
        self.manip_grid = None
        self.heatmap = None
        self.box_line = None
        self.ellipse = None
        self.ellipse_scale = 1.0

        link_count = len(robot.get_links(*trajectory[0][2:4])) if trajectory else 0
        self.lines = [self.ax.plot([], [], self.LINK_STYLE, lw=self.LINK_LINEWIDTH)[0]
                      for _ in range(link_count)]

        if self.show_trace:
            self.trace_line, = self.ax.plot([], [], self.TRACE_STYLE, lw=self.TRACE_LINEWIDTH)

        if self.show_manipulability:
            self.manip_grid = ManipulabilityGrid(robot, Nx, Ny)
            self.heatmap = self.ax.imshow(
                self.manip_grid.values,
                extent=self.manip_grid.extent,
                origin="lower",
                cmap=self.HEATMAP_CMAP,
                alpha=self.HEATMAP_ALPHA,
                interpolation="bilinear",
            )
            plt.colorbar(self.heatmap, ax=self.ax, label="Manipulability")

        if self.show_ellipse:
            self.ellipse_scale = self._compute_ellipse_scale()
            self.ellipse = Ellipse((0, 0), 0, 0, angle=0, fill=False,
                                   edgecolor=self.ELLIPSE_EDGE_COLOR,
                                   lw=self.ELLIPSE_LINEWIDTH, zorder=5)
            self.ax.add_patch(self.ellipse)

    def _ellipse_axes(self, theta1, theta2):
        """Returns (sigma1, sigma2, angle_deg) of the velocity manipulability
        ellipse — image of the unit joint-velocity circle under J — or None
        if the Jacobian can't be evaluated at this pose."""
        try:
            J = np.asarray(self.robot.jacobian(theta1, theta2), dtype=float)
            U, S, _ = np.linalg.svd(J)
        except (np.linalg.LinAlgError, ValueError, ZeroDivisionError):
            return None
        if not np.all(np.isfinite(S)):
            return None
        return S[0], S[1], math.degrees(math.atan2(U[1, 0], U[0, 0]))

    def _compute_ellipse_scale(self):
        sigmas = [ax[0] for _, _, t1, t2 in self.trajectory
                  if (ax := self._ellipse_axes(t1, t2)) is not None]
        max_sigma = max(sigmas, default=0.0)
        if max_sigma <= 0:
            return 1.0
        span = max(self.robot.MAX_PLOT_X - self.robot.MIN_PLOT_X,
                   self.robot.MAX_PLOT_Y - self.robot.MIN_PLOT_Y)
        return self.ELLIPSE_MAX_FRACTION * span / max_sigma

    def _animated_artists(self):
        artists = list(self.lines)
        if self.show_trace:
            artists.append(self.trace_line)
        artists.append(self.box_line)
        if self.show_ellipse:
            artists.append(self.ellipse)
        return artists

    def init_func(self):
        for line in self.lines:
            line.set_data([], [])
        if self.show_trace:
            self.trace_line.set_data([], [])

        box_segments = self.robot.get_drawable_box()
        xs = [p[0] for seg in box_segments for p in seg]
        ys = [p[1] for seg in box_segments for p in seg]
        self.box_line, = self.ax.plot(xs, ys, self.BOX_STYLE, lw=self.BOX_LINEWIDTH)

        return self._animated_artists()

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

        if self.show_ellipse:
            axes = self._ellipse_axes(theta1, theta2)
            if axes is None:
                self.ellipse.set_visible(False)
            else:
                s1, s2, angle = axes
                self.ellipse.set_visible(True)
                self.ellipse.set_center(links[0][0])
                self.ellipse.set_width(2 * s1 * self.ellipse_scale)
                self.ellipse.set_height(2 * s2 * self.ellipse_scale)
                self.ellipse.set_angle(angle)

        return self._animated_artists()

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
