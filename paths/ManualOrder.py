"""Manual draw-order state behind the web app's Manual order tab.

Automatic ordering (PathOrdering.tsp_geometry_order) only decides sequence
and direction. Sometimes the best result requires cutting a geometry in two
and drawing something else in the gap — since the robot can never lift its
pen, every transition between pieces draws a real connector line, so where
that line falls is a drawing-quality judgment call a human has to make.
"""

import numpy as np

MIN_PIECE_LEN = 2


class ManualOrderError(ValueError):
    """An invalid place/cut/undo request (unknown piece, bad cut point, ...).
    The message is suitable to show the user directly."""


def nearest_point_index(points, click):
    pts = np.array(points)
    return int(np.argmin(np.linalg.norm(pts - np.array(click), axis=1)))


def cuttable(points):
    return len(points) >= 2 * MIN_PIECE_LEN


def valid_cut_index_range(points):
    return MIN_PIECE_LEN - 1, len(points) - 1 - MIN_PIECE_LEN


def split_piece(points, idx):
    return points[:idx + 1], points[idx + 1:]


def build_ordered_path(all_pieces, placed_order):
    return [list(reversed(all_pieces[pid])) if reverse else all_pieces[pid]
            for pid, reverse in placed_order]


class ManualOrderState:
    """Pieces are identified by stable integer ids (never reused). The
    original geometries get ids 0..n-1; each cut retires its parent and
    creates two new ids that share the parent's root (for colouring)."""

    def __init__(self, geometries):
        n = len(geometries)
        self.next_id = n
        self.all_pieces = {i: geometries[i] for i in range(n)}
        self.root_of = {i: i for i in range(n)}
        self.pool_ids = set(range(n))
        self.placed_order = []
        self.history = []

    @property
    def is_complete(self):
        return not self.pool_ids

    def _require_pooled(self, piece_id):
        if piece_id not in self.pool_ids:
            raise ManualOrderError(f"Piece {piece_id} is not in the pool.")

    def place(self, piece_id, reverse=False):
        self._require_pooled(piece_id)
        self.pool_ids.discard(piece_id)
        self.placed_order.append((piece_id, bool(reverse)))
        self.history.append(("place", piece_id))

    def place_many(self, placements):
        """Place several (piece_id, reverse) pairs as a single undoable step."""
        ids = [pid for pid, _ in placements]
        if len(set(ids)) != len(ids):
            raise ManualOrderError("A piece can only be placed once.")
        for pid in ids:
            self._require_pooled(pid)
        for pid, reverse in placements:
            self.pool_ids.discard(pid)
            self.placed_order.append((pid, bool(reverse)))
        if placements:
            self.history.append(("batch", ids))

    def cut(self, piece_id, idx):
        """Split a pooled piece after point index `idx`. Returns the two new ids."""
        self._require_pooled(piece_id)
        points = self.all_pieces[piece_id]
        if not cuttable(points):
            raise ManualOrderError("That piece is too short to cut further.")
        lo, hi = valid_cut_index_range(points)
        if idx < lo or idx > hi:
            raise ManualOrderError("Cut point too close to an end of the piece — pick a point further inside.")

        id_a, id_b = self.next_id, self.next_id + 1
        self.next_id += 2
        for child, pts in zip((id_a, id_b), split_piece(points, idx)):
            self.all_pieces[child] = pts
            self.root_of[child] = self.root_of[piece_id]
            self.pool_ids.add(child)
        self.pool_ids.discard(piece_id)
        self.history.append(("cut", piece_id, id_a, id_b))
        return id_a, id_b

    def cut_at_point(self, piece_id, point):
        self._require_pooled(piece_id)
        return self.cut(piece_id, nearest_point_index(self.all_pieces[piece_id], point))

    def undo(self):
        """Reverts the last place, batch place or cut. Returns the kind undone."""
        if not self.history:
            raise ManualOrderError("Nothing to undo.")
        kind, *rest = self.history.pop()
        if kind in ("place", "batch"):
            count = 1 if kind == "place" else len(rest[0])
            for pid, _ in self.placed_order[-count:]:
                self.pool_ids.add(pid)
            del self.placed_order[-count:]
        else:  # "cut"
            parent_id, child_a, child_b = rest
            for child in (child_a, child_b):
                self.pool_ids.discard(child)
                del self.all_pieces[child]
                del self.root_of[child]
            self.pool_ids.add(parent_id)
        return kind

    def pen_position(self, start_xy):
        """Where the pen sits after the pieces placed so far."""
        if not self.placed_order:
            return tuple(start_xy)
        piece_id, reverse = self.placed_order[-1]
        points = self.all_pieces[piece_id]
        return tuple(points[0] if reverse else points[-1])

    def ordered_path(self):
        return build_ordered_path(self.all_pieces, self.placed_order)
