"""Interactive cut/reorder tool: lets a user split a geometry at a clicked
point and build the draw order by hand, with a live-updating plot.

Automatic ordering (PathOrdering.tsp_geometry_order) only decides sequence
and direction. Sometimes the best result requires cutting a geometry in two
and drawing something else in the gap — since the robot can never lift its
pen, every transition between pieces draws a real connector line, so where
that line falls is a drawing-quality judgment call a human has to make.
"""

import colorsys

import matplotlib.pyplot as plt
import numpy as np


class ManualOrderAborted(Exception):
    """Raised when the manual-ordering plot window is closed mid-session."""


MIN_PIECE_LEN = 2


def pick_colors(n):
    if n <= 20:
        return plt.cm.tab20(np.linspace(0, 1, n))
    return plt.cm.turbo(np.linspace(0, 1, n, endpoint=False))


def color_for_piece(piece_id, colors_by_root, root_of, child_path):
    base = colors_by_root[root_of[piece_id]]
    path = child_path[piece_id]
    if not path:
        return base

    h, l, s = colorsys.rgb_to_hls(base[0], base[1], base[2])
    for i, branch in enumerate(path):
        step = 0.18 / (i + 1)
        l = min(0.85, l + step) if branch == 0 else max(0.15, l - step)
    red, green, blue = colorsys.hls_to_rgb(h, l, s)
    alpha = base[3] if len(base) > 3 else 1.0
    return (red, green, blue, alpha)


def nearest_point_index(points, click):
    pts = np.array(points)
    return int(np.argmin(np.linalg.norm(pts - np.array(click), axis=1)))


def cuttable(points):
    return len(points) >= 2 * MIN_PIECE_LEN


def valid_cut_index_range(points):
    lo = MIN_PIECE_LEN - 1
    hi = len(points) - 1 - MIN_PIECE_LEN
    return lo, hi


def split_piece(points, idx):
    return points[:idx + 1], points[idx + 1:]


def renumber_pool(pool_ids):
    # Fresh, compact 1..m labels over exactly the current pool membership. Only
    # called when the pool's membership actually changes shape (a cut, or
    # undoing one) — NOT on a plain placement, which must not disturb the
    # numbers of everything still sitting in the pool.
    order = sorted(pool_ids)
    pool_display = {piece_id: str(i + 1) for i, piece_id in enumerate(order)}
    pool_reverse = {label: piece_id for piece_id, label in pool_display.items()}
    return pool_display, pool_reverse


def build_ordered_path(all_pieces, placed_order):
    path = []
    for piece_id, reverse in placed_order:
        points = all_pieces[piece_id]
        path.append(list(reversed(points)) if reverse else points)
    return path


class ManualOrderSession:
    """Holds all state for one interactive cut/reorder run and drives its
    command loop + live plot. Construct and call run() once; use the
    module-level prompt_manual_order() convenience function otherwise."""

    def __init__(self, geometries, robot):
        self.robot = robot
        n = len(geometries)
        colors = pick_colors(n)

        self.next_id = n
        self.all_pieces = {i: geometries[i] for i in range(n)}
        self.colors_by_root = {i: colors[i] for i in range(n)}
        self.root_of = {i: i for i in range(n)}
        self.child_path = {i: () for i in range(n)}
        self.pool_ids = set(range(n))
        self.pool_display, self.pool_reverse = renumber_pool(self.pool_ids)
        self.placed_order = []
        self.history = []

        self.fig, self.ax = plt.subplots()

    def _color_for(self, piece_id):
        return color_for_piece(piece_id, self.colors_by_root, self.root_of, self.child_path)

    def _safe_redraw(self):
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            raise ManualOrderAborted()

    def _render(self, highlight_id=None):
        ax = self.ax
        ax.clear()
        ax.set_xlim(0, self.robot.DRAWABLE_WIDTH)
        ax.set_ylim(0, self.robot.DRAWABLE_HEIGHT)
        ax.set_aspect("equal")
        ax.set_title(
            "Manual geometry order\n"
            "^ = start, s = end  |  faint dots = not yet added  |  solid = added  |  "
            "black dotted = pen-down travel between pieces"
        )
        ax.text(
            0.01, 0.01,
            "commands: <#> place | <#>r place reversed | cut <#> | undo",
            transform=ax.transAxes, fontsize=8, color="0.3", va="bottom",
        )

        prev_point = self.robot.START_XY
        ax.plot(*self.robot.START_XY, marker="*", color="black", markersize=12)
        ax.annotate("home", self.robot.START_XY, fontsize=8, xytext=(4, 4), textcoords="offset points")

        # Placed pieces are labeled by their fixed position in the draw order
        # (1..len(placed_order)) — this never needs renumbering since it's
        # just "which step is this," recomputed trivially from the list.
        for i, (piece_id, reverse) in enumerate(self.placed_order):
            points = self.all_pieces[piece_id]
            points = list(reversed(points)) if reverse else points
            color = self._color_for(piece_id)
            ax.plot(*zip(*points), linestyle="-", color=color, linewidth=2)
            ax.plot(*points[0], marker="^", color=color, markersize=9, linestyle="None")
            ax.plot(*points[-1], marker="s", markerfacecolor="none", markeredgecolor=color, markersize=7, linestyle="None")
            ax.annotate(str(i + 1), points[0], fontsize=10, fontweight="bold", color=color,
                        xytext=(4, 4), textcoords="offset points")
            ax.plot([prev_point[0], points[0][0]], [prev_point[1], points[0][1]],
                    linestyle=":", color="black", linewidth=1.5)
            prev_point = points[-1]

        for piece_id in sorted(self.pool_display, key=lambda pid: int(self.pool_display[pid])):
            points = self.all_pieces[piece_id]
            color = self._color_for(piece_id)
            is_highlighted = piece_id == highlight_id
            ax.plot(*zip(*points), linestyle="None", marker=".", markersize=(4 if is_highlighted else 2),
                    color=color, alpha=(0.95 if is_highlighted else (0.5 if highlight_id is None else 0.15)))
            ax.plot(*points[0], marker="^", color=color, markersize=9, linestyle="None")
            ax.plot(*points[-1], marker="s", markerfacecolor="none", markeredgecolor=color, markersize=7, linestyle="None")
            ax.annotate(self.pool_display[piece_id], points[0], fontsize=10, fontweight="bold", color=color,
                        xytext=(4, 4), textcoords="offset points")

        self._safe_redraw()

    def _cmd_undo(self):
        if not self.history:
            print("  Nothing to undo.")
            return

        kind, *rest = self.history.pop()
        if kind == "place":
            piece_id, _reverse, label = rest
            self.placed_order.pop()
            self.pool_ids.add(piece_id)
            self.pool_display[piece_id] = label
            self.pool_reverse[label] = piece_id
        else:  # "cut"
            parent_id, child_a, child_b = rest
            self.pool_ids.discard(child_a)
            self.pool_ids.discard(child_b)
            del self.all_pieces[child_a]
            del self.all_pieces[child_b]
            del self.root_of[child_a]
            del self.root_of[child_b]
            del self.child_path[child_a]
            del self.child_path[child_b]
            self.pool_ids.add(parent_id)
            # Pool membership is now exactly what it was right before the
            # cut, so a fresh renumber deterministically reproduces the same
            # labels it had then — no separate bookkeeping needed.
            self.pool_display, self.pool_reverse = renumber_pool(self.pool_ids)

    def _cmd_cut(self, label):
        piece_id = self.pool_reverse.get(label)
        if piece_id is None:
            print(f"  '{label}' is not an available geometry number.")
            return

        points = self.all_pieces[piece_id]
        if not cuttable(points):
            print(f"  '{label}' is too short to cut further.")
            return

        self._render(highlight_id=piece_id)
        print(f"  Click the point on '{label}' where it should be cut "
              f"(Esc/Enter/middle-click to cancel)...")
        try:
            clicks = plt.ginput(1, timeout=0)
        except Exception:
            raise ManualOrderAborted()

        if not clicks:
            print("  Cut cancelled.")
            return

        idx = nearest_point_index(points, clicks[0])
        lo, hi = valid_cut_index_range(points)
        if idx < lo or idx > hi:
            print("  Cut point too close to an end of the geometry — pick a point further inside.")
            return

        piece_a, piece_b = split_piece(points, idx)
        id_a, id_b = self.next_id, self.next_id + 1
        self.next_id += 2
        self.all_pieces[id_a] = piece_a
        self.all_pieces[id_b] = piece_b
        self.root_of[id_a] = self.root_of[piece_id]
        self.root_of[id_b] = self.root_of[piece_id]
        self.child_path[id_a] = self.child_path[piece_id] + (0,)
        self.child_path[id_b] = self.child_path[piece_id] + (1,)
        self.pool_ids.discard(piece_id)
        self.pool_ids.add(id_a)
        self.pool_ids.add(id_b)
        self.pool_display, self.pool_reverse = renumber_pool(self.pool_ids)
        self.history.append(("cut", piece_id, id_a, id_b))

    def _cmd_place(self, token):
        reverse = token[-1:] in ("r", "R")
        label = token[:-1] if reverse else token
        piece_id = self.pool_reverse.get(label)
        if piece_id is None:
            print(f"  '{label}' is not a placeable geometry number right now.")
            return

        pool_label = self.pool_display[piece_id]
        self.pool_ids.discard(piece_id)
        del self.pool_display[piece_id]
        del self.pool_reverse[pool_label]
        self.placed_order.append((piece_id, reverse))
        self.history.append(("place", piece_id, reverse, pool_label))

    def run(self):
        plt.ion()
        plt.show()

        n = len(self.all_pieces)
        print(f"\nBuild the draw order for {n} geometries one command at a time:")
        print("  <#>       place that geometry next")
        print("  <#>r      place it reversed")
        print("  cut <#>   click a point on that geometry to split it in two")
        print("  undo      undo the last place or cut")
        print("  (pool numbers are reassigned to stay simple only when a cut changes the pool)")

        try:
            while self.pool_ids:
                if not plt.fignum_exists(self.fig.number):
                    raise ManualOrderAborted()

                self._render()
                remaining = sorted(self.pool_display.values(), key=int)
                raw = input(f"  {len(self.placed_order)} placed, remaining {remaining}: ").strip()
                if not raw:
                    continue

                if raw.lower() == "undo":
                    self._cmd_undo()
                elif raw.lower().startswith("cut"):
                    parts = raw.split()
                    if len(parts) != 2:
                        print("  Usage: cut <#>")
                    else:
                        self._cmd_cut(parts[1])
                else:
                    self._cmd_place(raw)

            self._render()
        finally:
            if plt.fignum_exists(self.fig.number):
                plt.close(self.fig)

        return build_ordered_path(self.all_pieces, self.placed_order)


def prompt_manual_order(geometries, robot):
    return ManualOrderSession(geometries, robot).run()
