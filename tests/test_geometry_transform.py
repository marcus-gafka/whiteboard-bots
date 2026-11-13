import pytest

from paths import GeometryTransform as gt


def test_get_bounding_box():
    points = [(0, 0), (3, -1), (2, 5)]
    min_x, max_x, min_y, max_y = gt.get_bounding_box(points)
    assert (min_x, max_x, min_y, max_y) == (0, 3, -1, 5)


def test_dist_points():
    assert gt.dist_points((0, 0), (3, 4)) == pytest.approx(5.0)


def test_scale_and_translate_fits_drawable_area():
    class FakeRobot:
        DRAWABLE_WIDTH = 10.0
        DRAWABLE_HEIGHT = 10.0

        def get_aspect_ratio(self):
            return self.DRAWABLE_HEIGHT / self.DRAWABLE_WIDTH

    geometries = [[(0, 0), (20, 20)]]
    scaled = gt.scale_and_translate_geometries(FakeRobot(), geometries)
    min_x, max_x, min_y, max_y = gt.get_bounding_box(scaled[0])
    assert max_x - min_x == pytest.approx(10.0)
    assert max_y - min_y == pytest.approx(10.0)
    assert min_x == pytest.approx(0.0, abs=1e-9)
    assert min_y == pytest.approx(0.0, abs=1e-9)
