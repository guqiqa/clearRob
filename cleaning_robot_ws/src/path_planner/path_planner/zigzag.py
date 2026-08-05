#!/usr/bin/env python3
"""
path_planner/zigzag.py — Boustrophedon (弓字形) coverage path generator.

Ported from the horizon1 project (coverage_node.py / test_zigzag.py) — pure
math, no ROS dependencies, so it can be unit-tested on any host.

The algorithm sweeps horizontal lines across a rectangular zone, producing a
left→right then right→left boustrophedon waypoint list.  Sweep spacing matches
the cleaning head width (~0.6 m on the real robot).

Reuses:
  - generate_zigzag_path()  : the sweep-line boustrophedon generator
  - get_yaw_deg()           : heading between two waypoints (for orientation)
"""

import math

# Default sweep spacing = cleaning head width on the real robot (meters)
DEFAULT_SWEEP_SPACING = 0.6


def generate_zigzag_path(polygon_coords, robot_x, sweep_spacing=DEFAULT_SWEEP_SPACING,
                         start_from_top=False):
    """Generate boustrophedon waypoints covering a rectangular zone.

    polygon_coords : [(x0,y0), (x1,y1), ...] convex rectangle corners
    robot_x        : robot's current X (used to pick left/right start)
    sweep_spacing  : spacing between parallel sweeps (cleaning width)
    start_from_top : sweep from the zone's top edge (vs bottom)

    Returns list of (x, y) waypoints ordered for coverage.
    """
    from shapely.geometry import Polygon, LineString

    poly = Polygon(polygon_coords)
    minx, miny, maxx, maxy = poly.bounds
    waypoints = []

    if not start_from_top:
        y = miny + sweep_spacing / 2.0
        step = sweep_spacing
        condition = lambda cur: cur < maxy
    else:
        y = maxy - sweep_spacing / 2.0
        step = -sweep_spacing
        condition = lambda cur: cur > miny

    # Smart start: begin from whichever X edge the robot is nearer.
    left_to_right = abs(robot_x - minx) <= abs(robot_x - maxx)

    while condition(y):
        sweep_line = LineString([(minx - 1, y), (maxx + 1, y)])
        intersection = sweep_line.intersection(poly)
        if not intersection.is_empty and intersection.geom_type == 'LineString':
            p_left = intersection.coords[0]
            p_right = intersection.coords[-1]
            if p_left[0] > p_right[0]:
                p_left, p_right = p_right, p_left
            if left_to_right:
                waypoints.extend([p_left, p_right])
            else:
                waypoints.extend([p_right, p_left])
            left_to_right = not left_to_right
        y += step
    return waypoints


def get_yaw_deg(p_current, p_next):
    """Heading (degrees) from p_current toward p_next."""
    return math.degrees(math.atan2(p_next[1] - p_current[1],
                                   p_next[0] - p_current[0]))


def distance(p1, p2):
    """Euclidean distance between two (x, y) points."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def smart_sweep_direction(zone, robot_y):
    """Pick sweep direction (from_top vs from_bottom) by robot's Y.

    zone   : [(x0,y0),...] polygon corners
    robot_y: robot's current Y
    Returns True if sweeping from the top edge.
    """
    from shapely.geometry import Polygon
    poly = Polygon(zone)
    miny = poly.bounds[1]
    maxy = poly.bounds[3]
    return abs(robot_y - maxy) < abs(robot_y - miny)


# Shapely is only needed at call time; make sure it's importable.
__all__ = [
    'DEFAULT_SWEEP_SPACING',
    'generate_zigzag_path',
    'get_yaw_deg',
    'distance',
    'smart_sweep_direction',
]
