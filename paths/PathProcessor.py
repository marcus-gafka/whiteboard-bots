import math
import os
import pathlib
import numpy as np
from robots.Robot import Robot as r

class PathProcessor:

    INITIAL_SPACING = 0.02

    def __init__(self, dxf_file, spacing):
        self.dxf_file = dxf_file
        self.spacing = spacing
        self.geometries = []

    @staticmethod
    def get_bounding_box(points):
        points = np.array(points)
        min_x, min_y = points.min(axis=0)
        max_x, max_y = points.max(axis=0)
        return min_x, max_x, min_y, max_y

    @staticmethod
    def print_bounding_box(points):
        min_x, max_x, min_y, max_y = PathProcessor.get_bounding_box(points)
        print(f"X: {min_x:.2f} to {max_x:.2f}")
        print(f"Y: {min_y:.2f} to {max_y:.2f}")

    @staticmethod
    def line_to_points(start, end, spacing):
        start = np.array(start)
        end = np.array(end)
        vec = end - start
        length = np.linalg.norm(vec)
        num_segments = max(int(length / spacing), 1)
        t = np.linspace(0, 1, num_segments + 1)
        points = start + np.outer(t, vec)
        return points.tolist()

    @staticmethod
    def arc_to_points(center, radius, start_angle, end_angle, spacing):
        center = np.array(center)
        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        if end_rad < start_rad:
            end_rad += 2 * math.pi
        arc_length = radius * (end_rad - start_rad)
        num_segments = max(int(arc_length / spacing), 1)
        angles = np.linspace(start_rad, end_rad, num_segments + 1)
        x = center[0] + radius * np.cos(angles)
        y = center[1] + radius * np.sin(angles)
        points = np.stack((x, y), axis=-1)
        return points.tolist()

    @staticmethod
    def polyline_to_points(vertices, spacing):
        points = []
        vertices = np.array(vertices)
        for i in range(len(vertices) - 1):
            segment_points = PathProcessor.line_to_points(vertices[i], vertices[i + 1], spacing=spacing)
            points.extend(segment_points)
        return points

    @staticmethod
    def extract_geometries(doc, spacing):
        msp = doc.modelspace()
        geometries = []
        lines, circles, arcs, polylines = 0, 0, 0, 0

        for e in msp:
            if e.dxftype() == 'LINE':
                start = (e.dxf.start.x, e.dxf.start.y)
                end = (e.dxf.end.x, e.dxf.end.y)
                geometries.append(PathProcessor.line_to_points(start, end, spacing))
                lines += 1

            elif e.dxftype() == 'CIRCLE':
                center = (e.dxf.center.x, e.dxf.center.y)
                radius = e.dxf.radius
                geometries.append(PathProcessor.arc_to_points(center, radius, 0, 360, spacing*2))
                circles += 1

            elif e.dxftype() == 'ARC':
                center = (e.dxf.center.x, e.dxf.center.y)
                radius = e.dxf.radius
                geometries.append(PathProcessor.arc_to_points(center, radius, e.dxf.start_angle, e.dxf.end_angle, spacing * 2))
                arcs += 1

            elif e.dxftype() in ['LWPOLYLINE', 'POLYLINE']:
                vertices = []
                if e.dxftype() == 'LWPOLYLINE':
                    vertices = [(pt[0], pt[1]) for pt in e]
                else:  # POLYLINE
                    for v in e.vertices:
                        loc = v.dxf.location
                        vertices.append((loc.x, loc.y))
                geometries.append(PathProcessor.polyline_to_points(vertices, spacing * 3))
                polylines += 1

        return geometries, lines, circles, arcs, polylines

    @staticmethod
    def scale_and_translate_geometries(cls, geometries):
        all_points = np.vstack([np.array(geometry) for geometry in geometries])
        min_x, max_x, min_y, max_y = PathProcessor.get_bounding_box(all_points)
        width = max_x - min_x
        height = max_y - min_y

        scale_factor = (cls.DRAWABLE_WIDTH / width) if (height < r.get_aspect_ratio(cls) * width) else (cls.DRAWABLE_HEIGHT / height)
        tx = cls.DRAWABLE_WIDTH / 2 - ((min_x + max_x) / 2) * scale_factor
        ty = cls.DRAWABLE_HEIGHT / 2 - ((min_y + max_y) / 2) * scale_factor

        scaled_translated = []
        for geometry in geometries:
            pts = np.array(geometry)
            pts = pts * scale_factor
            pts[:, 0] += tx
            pts[:, 1] += ty
            scaled_translated.append(pts.tolist())

        return scaled_translated

    @staticmethod
    def dist_points(p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.linalg.norm(p1 - p2)

    @staticmethod
    def geometry_dist_to_start(p):
        start = np.array(r.START_XY)
        p_arr = np.array(p)
        dist_start = np.linalg.norm(p_arr[0] - start)
        dist_end = np.linalg.norm(p_arr[-1] - start)
        return min(dist_start, dist_end)

    @staticmethod
    def geometry_distance(g1, g2):
        g1 = np.array(g1)
        g2 = np.array(g2)
        start = 0
        end = -1
        candidates = [
            (np.linalg.norm(g1[end] - g2[start]), False, False),
            (np.linalg.norm(g1[end] - g2[end]), False, True),
            (np.linalg.norm(g1[start] - g2[start]), True, False),
            (np.linalg.norm(g1[start] - g2[end]), True, True),
        ]
        return min(candidates, key=lambda x: x[0])

    @staticmethod
    def tsp_geometry_order(geometries):
        n = len(geometries)
        visited = np.zeros(n, dtype=bool)
        path = []

        distances_to_start = np.array([PathProcessor.geometry_dist_to_start(g) for g in geometries])
        current_index = np.argmin(distances_to_start)
        current_geometry = geometries[current_index]
        visited[current_index] = True

        dist_to_start_first = PathProcessor.dist_points(r.START_XY, current_geometry[0])
        dist_to_start_last = PathProcessor.dist_points(r.START_XY, current_geometry[-1])
        if dist_to_start_first > dist_to_start_last:
            current_geometry = list(reversed(current_geometry))

        path.append(current_geometry)

        for _ in range(n - 1):
            best_dist = float('inf')
            best_index = None
            best_geom_reversed = False

            for i in range(n):
                if visited[i]:
                    continue
                dist, _, rev_next = PathProcessor.geometry_distance(path[-1], geometries[i])
                if dist < best_dist:
                    best_dist = dist
                    best_index = i
                    best_geom_reversed = rev_next

            next_geom = geometries[best_index]
            if best_geom_reversed:
                next_geom = list(reversed(next_geom))
            path.append(next_geom)
            visited[best_index] = True

        return path

    @staticmethod
    def reduce_points(points, n):
        points = np.array(points)
        if n >= len(points):
            return points.tolist()

        step = (len(points) - 1) / (n - 1)
        indices = np.unique(np.round(np.arange(n - 1) * step).astype(int))
        indices = np.append(indices, len(points) - 1)
        indices = np.unique(indices)

        reduced = points[indices]
        return reduced.tolist()

    @staticmethod
    def select_dxf_file(folder):
        files = sorted(f for f in os.listdir(folder) if f.endswith(".dxf"))
        if not files:
            raise FileNotFoundError("No DXF files found.")

        print("\nAvailable DXF files:")
        for i, file in enumerate(files):
            print(f"{i + 1}: {file}")

        index = int(input("Enter the number of the DXF file to load: ")) - 1
        return os.path.join(folder, files[index])

    @staticmethod
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