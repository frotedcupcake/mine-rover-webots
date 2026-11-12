"""Pure-Python autonomy core for the ECE357 mine rover.

Nothing in this package imports Webots, so it runs under pytest and could be ported
to the real robot's computer unchanged.
"""
SAFE, CAUTION, DANGER = "SAFE", "CAUTION", "DANGER"
LEVEL_ORDER = {SAFE: 0, CAUTION: 1, DANGER: 2}
