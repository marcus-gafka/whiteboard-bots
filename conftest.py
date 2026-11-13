# Presence of this file at the repo root makes pytest add the repo root to
# sys.path (no __init__.py anywhere), so tests can `import robots...` etc.
# exactly like main.py does, without needing a package/setup.py.
import matplotlib

# Force a non-interactive backend before any test imports matplotlib.pyplot
# (paths/ManualOrder.py imports it at module level) so the suite runs headless.
matplotlib.use("Agg")
