from Plotter import Plotter2D
from robots.TwoLink import TwoLink


def _sample_trajectory_and_commands(robot, n=3):
    trajectory, commands = [], []
    x, y = robot.DRAWABLE_WIDTH / 2, robot.DRAWABLE_HEIGHT / 2
    t1, t2 = robot.ik(x, y)
    for i in range(n):
        trajectory.append((x, y, t1, t2))
        commands.append((0, 0, 10))
    return trajectory, commands


def test_plotter_link_count_matches_robot_not_hardcoded():
    robot = TwoLink()
    trajectory, commands = _sample_trajectory_and_commands(robot)
    plotter = Plotter2D(robot, trajectory, commands, show_trace=False,
                         show_manipulability=False)
    expected = len(robot.get_links(*trajectory[0][2:4]))
    assert len(plotter.lines) == expected


def test_plotter_init_func_and_update_func_return_consistent_artists():
    robot = TwoLink()
    trajectory, commands = _sample_trajectory_and_commands(robot)
    plotter = Plotter2D(robot, trajectory, commands, show_trace=True,
                         show_manipulability=False, )
    init_artists = plotter.init_func()
    update_artists = plotter.update_func(0)
    assert len(init_artists) == len(update_artists)
    assert plotter.box_line in init_artists
    assert plotter.trace_line in init_artists


def test_plotter_with_manipulability_heatmap_uses_small_grid():
    robot = TwoLink()
    trajectory, commands = _sample_trajectory_and_commands(robot)
    plotter = Plotter2D(robot, trajectory, commands, show_trace=False,
                         show_manipulability=True, Nx=4, Ny=4)
    assert plotter.manip_grid is not None
    assert plotter.manip_grid.values.shape == (4, 4)
    assert plotter.heatmap is not None
