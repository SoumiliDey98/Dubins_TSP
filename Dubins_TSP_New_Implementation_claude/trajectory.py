# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# The genetic algorithm (ga_tour.py) only decides the ORDER in which a robot
# visits its sensors. This file turns that order into the real drive:
#   1. the points the robot must pass ("waypoints"): the sensors, plus the
#      corners it must go round so as not to hit obstacles;
#   2. which way the robot points at each waypoint (its "heading");
#   3. the smooth curved path between every two waypoints (a Dubins path,
#      made by dubins_path.py);
#   4. how long all that takes, including the stops at sensors: T = L / v + sum of tau;
#   5. where the robot is every half second (the "frames"), like the frames
#      of a film. Every robot starts at time t = 0.
# ==============================================================================
"""From a tour (order of sensors) to a timed trajectory (Sections 0.5, 0.7.2-0.7.3).

1. Waypoints: base -> sensors -> base, joined by the geodesic (corner-to-corner)
   paths. Corners closer than R to a neighbouring waypoint are dropped, so the
   Dubins curves do not need small loops, unless the shortcut would pass too
   close to an obstacle.
2. Headings (the decision variables theta of Section 0.5): at every waypoint the
   heading is chosen from a discrete set (16 directions plus the bisector of the
   incoming and outgoing directions) by dynamic programming, so that the total
   Dubins length is as short as possible and every leg keeps r_safe from the
   obstacles (eq. 19).
3. Consecutive poses are joined by a Dubins path, chosen as p["dubins_method"] says
   (by default the shortest one).
4. Time: t = arc length / v, plus a stop of tau_i at sensor i, so that
   T_k = L_k / v + sum of tau_i            (eq. 17)
5. The position is read every dt seconds ("frames"); all sinks start at t = 0.
"""
# "import" borrows a toolbox. "math" has maths for single numbers (sqrt, atan2, ...).
import math

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# shapely: a toolbox for flat shapes (points, lines, polygons) and distances between them.
import shapely
# LineString: a line made of several straight pieces joined end to end.
from shapely.geometry import LineString

# Two files of this project: the Dubins paths, and the shortest ways round obstacles.
import dubins_path
# geodesic.py: the shortest ways round obstacles.
import geodesic

# The 16 headings every waypoint may choose from: 16 directions spread evenly
# round a full circle, from -pi up to (not including) pi, every 22.5 degrees.
HEADINGS = np.linspace(-np.pi, np.pi, 16, endpoint=False)


# "def" starts a recipe (a "function"). This one lists the waypoints of a trip.
# Inputs: "order", the sensors in visiting order; "geo", the shortest-way
# information from geodesic.build; "p", the settings.
def waypoints(order, geo, p):
    """Points the sink must pass, and for each one the sensor index (or -1 for corners and the base).
    A corner closer than R to its neighbours is skipped (it would force a small loop), unless
    the shortcut would pass closer than r_safe + 0.5 m to an obstacle."""
    # The trip as terminal numbers: the base (0), each sensor i as number i + 1,
    # and the base again (0).
    terms = [0] + [i + 1 for i in order] + [0]
    # pts: the waypoints, starting with the base (node 0). sensor: for each
    # waypoint the sensor's number, or -1 if it is not a sensor.
    pts, sensor = [geo["nodes"][0]], [-1]
    # "zip" walks two lists side by side: here every consecutive pair (a, b) of
    # the trip, so (base, first sensor), (first sensor, second sensor), ...
    for a, b in zip(terms[:-1], terms[1:]):
        # The shortest way from a to b round the obstacles, corner by corner.
        leg = geodesic.path(geo, a, b)
        # Go through the corners in the middle of that way (not its two ends).
        for m in range(1, len(leg) - 1):
            # Is this corner closer than R (5 m) to the last waypoint or to the
            # end of the leg? Then the car would need a small loop to pass it.
            close = np.hypot(*(leg[m] - pts[-1])) < p["R"] or np.hypot(*(leg[m] - leg[-1])) < p["R"]
            # Would it be safe to skip the corner, going straight from the last
            # waypoint to the next corner? Safe if there are no obstacles, or if
            # that shortcut stays at least r_safe + 0.5 = 2.5 m from them.
            # (The "\" at the end of the next line means the statement goes on
            # to the line after it.)
            shortcut_ok = geo["obstacles"] is None or \
                shapely.distance(LineString([pts[-1], leg[m + 1]]), geo["obstacles"]) >= p["r_safe"] + 0.5
            # Keep the corner unless it is BOTH too close AND safe to skip.
            if not (close and shortcut_ok):
                # Add the corner as a waypoint, marked -1 (not a sensor).
                pts.append(leg[m])
                # Mark it -1: not a sensor.
                sensor.append(-1)
        # Add the end of the leg: sensor b - 1 (or the base, which gives -1).
        pts.append(leg[-1])
        # Mark it with its sensor number, b - 1 (for the base, 0 - 1 = -1).
        sensor.append(b - 1)
    # Hand back the waypoints as a numpy array, and the sensor marks.
    return np.array(pts), sensor


# A first guess of the heading at every waypoint: halfway between the
# direction the car arrives from and the direction it leaves in.
def bisectors(pts):
    """Heading halfway between the incoming and outgoing directions at every waypoint."""
    # One heading per waypoint, all 0 to begin with.
    h = np.zeros(len(pts))
    # For every waypoint k:
    for k in range(len(pts)):
        # d_in: the step arriving here (none for the first waypoint).
        d_in = pts[k] - pts[k - 1] if k > 0 else None
        # d_out: the step leaving here (none for the last waypoint).
        d_out = pts[k + 1] - pts[k] if k < len(pts) - 1 else None
        # At the first waypoint, just point where the car is going.
        if d_in is None:
            # Use the leaving direction.
            u = d_out
        # At the last waypoint, just point where the car came from.
        elif d_out is None:
            # Use the arriving direction.
            u = d_in
        # Elsewhere: add the two directions, each shrunk to length 1
        # (np.linalg.norm is an arrow's length). The sum points halfway between.
        else:
            # Each direction divided by its length (so it has length 1), then added.
            u = d_in / np.linalg.norm(d_in) + d_out / np.linalg.norm(d_out)
            # If the path turns straight back, the two cancel out to nothing.
            # Then point sideways (the arriving arrow turned by a right angle).
            if np.linalg.norm(u) < 1e-9:              # the path turns back: pass sideways
                # The arriving arrow turned a quarter turn to the left: sideways.
                u = np.array([-d_in[1], d_in[0]])
        # The angle of the arrow u.
        h[k] = math.atan2(u[1], u[0])
    # Hand back all the headings.
    return h


# The "_" at the start of the name means "only used inside this file".
# Checks that the curved leg from pose q0 to pose q1 stays at least r_safe
# (2 m) from every obstacle. "length" is the leg's length; "room" is how far
# the straight line between the two points stays from the obstacles.
def _leg_clear(q0, q1, length, room, obstacles, p):
    """True if the Dubins leg from q0 to q1 keeps r_safe from the obstacles. Quick test
    first: a path of length L between points c apart never leaves the band of half-width
    sqrt(L^2 - c^2) / 2 around the straight segment, which itself has clearance `room`."""
    # No obstacles at all: the leg is clear.
    if obstacles is None:
        # Clear.
        return True
    # c: the straight distance between the two points.
    c = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
    # The quick test: a path of length L between two points c apart can never
    # wander further than sqrt(L^2 - c^2) / 2 from the straight line between
    # them (any point further away would make the path longer than L). If even
    # that furthest wander keeps r_safe, the leg is clear without drawing it.
    if math.sqrt(max(length ** 2 - c ** 2, 0.0)) / 2 <= room - p["r_safe"]:
        # Clear, without drawing the leg.
        return True
    # Otherwise draw the real curved leg. The "*" hands over the pose's three
    # numbers (x, y, heading) one by one.
    xtrajectory, ytrajectory, _, _ = dubins_path.returnDubinsPath(*q0, *q1, p["R"], p["dubins_method"], p["ds"])
    # Clear if its smallest distance to the obstacles is at least r_safe.
    return shapely.distance(LineString(np.column_stack([xtrajectory, ytrajectory])), obstacles) >= p["r_safe"]


# Chooses one heading at every waypoint so that the whole drive is as short as
# possible and never too close to an obstacle. It uses "dynamic programming":
# go from the first waypoint to the last, and for every heading at the current
# waypoint remember only the cheapest way to arrive there. Trying every
# combination would take 17 x 17 x 17 x ... tries; this needs only 17 x 17 per leg.
def choose_headings(pts, geo, p):
    """Headings that minimise the total Dubins length with every leg clear of the
    obstacles (dynamic programming over the candidate headings of each waypoint).
    Falls back to the bisectors if no clear combination exists (reported by build)."""
    # The halfway headings.
    bis = bisectors(pts)
    # The candidates at each waypoint: the 16 directions plus its halfway heading (17 in all).
    cand = [np.append(HEADINGS, bis[k]) for k in range(len(pts))]
    # cost[m]: the shortest total length so far to arrive at the current
    # waypoint with candidate heading m. At the first waypoint nothing has been
    # driven yet, so all are 0.
    cost = np.zeros(len(cand[0]))
    # back remembers, for every leg, which earlier heading gave each best cost.
    back = []
    # For every leg, from waypoint k - 1 to waypoint k:
    for k in range(1, len(pts)):
        # L[i, m]: the Dubins length of this leg from heading i at the start to
        # heading m at the end, for all 17 x 17 pairs at once. [:, None] and
        # [None, :] turn the two lists into a column and a row, so numpy fills a
        # whole table.
        L = dubins_path.returnPathDistances(*pts[k - 1], cand[k - 1][:, None], *pts[k], cand[k][None, :], p["R"], p["dubins_method"])
        # total[i, m]: the best cost to arrive with heading i, plus this leg.
        total = cost[:, None] + L
        # How far the straight line of this leg stays from the obstacles
        # (infinity, math.inf, if there are none). Used by the quick test.
        room = (shapely.distance(LineString(pts[k - 1:k + 1]), geo["obstacles"])
                if geo["obstacles"] is not None else math.inf)
        # new: the new best cost for each end heading, infinity until found.
        # arg: which start heading gave it. np.full makes an array filled with
        # one value; dtype=int means whole numbers.
        new, arg = np.full(len(cand[k]), np.inf), np.zeros(len(cand[k]), dtype=int)
        # For every end heading m:
        for m in range(len(cand[k])):
            # Try the start headings from cheapest to dearest (np.argsort lists
            # the positions in order of increasing cost).
            for i in np.argsort(total[:, m]):                # cheapest predecessor first
                # Infinity means impossible, and the rest are worse: stop.
                if not np.isfinite(total[i, m]):
                    # Stop: all the remaining ones cost even more.
                    break
                # The first one whose leg is clear of obstacles is the best:
                # remember its cost and which start heading it was, and stop.
                if _leg_clear((*pts[k - 1], cand[k - 1][i]), (*pts[k], cand[k][m]), L[i, m], room, geo["obstacles"], p):
                    # Remember its total cost and which start heading gave it.
                    new[m], arg[m] = total[i, m], i
                    # Stop looking: the cheapest clear one is found.
                    break
        # These become the best costs at waypoint k.
        cost = new
        # And remember the choices of this leg.
        back.append(arg)
    # If no heading at the last waypoint could be reached safely, fall back to
    # the halfway headings (build() will then report the clearance problem).
    if not np.isfinite(cost.min()):
        # Hand back the halfway headings.
        return bis
    # The best heading at the last waypoint: the one with the lowest total.
    m = int(np.argmin(cost))
    # Start the list of chosen headings with it.
    h = [cand[-1][m]]
    # Walk backwards through the legs (range(n - 1, 0, -1) counts down),
    # looking up which heading led to the chosen one each time.
    for k in range(len(pts) - 1, 0, -1):
        # The heading at the waypoint before, which led to the chosen one.
        m = back[k - 1][m]
        # Add it to the list.
        h.append(cand[k - 1][m])
    # The list was built backwards; [::-1] turns it round. Hand it back.
    return np.array(h[::-1])


# The main recipe of this file: from an order of sensors to the full timed drive.
def build(order, geo, scen, p):
    """Trajectory of one sink: {"order", "frames" (F, 2), "path" (dense polyline), "L", "T"}."""
    # The waypoints, and which of them are sensors.
    pts, sensor = waypoints(order, geo, p)
    # The best heading at each waypoint.
    h = choose_headings(pts, geo, p)
    # "Knots": moments in time (knot_t) and where the car is then (knot_xy).
    # At time 0 it is at the first waypoint, the base.
    knot_t, knot_xy = [0.0], [pts[0]]
    # t: the time so far; L: the distance driven so far. Both start at 0.
    t = L = 0.0
    # For every leg, from waypoint k to waypoint k + 1:
    for k in range(len(pts) - 1):
        # The start pose and the end pose: position plus heading.
        q0, q1 = (*pts[k], h[k]), (*pts[k + 1], h[k + 1])
        # The curved Dubins path of this leg, with points every ds = 0.25 m.
        xtrajectory, ytrajectory, pathDistance, strajectory = dubins_path.returnDubinsPath(
            *q0, *q1, p["R"], p["dubins_method"], p["ds"])
        # Its points as rows (x, y).
        xy = np.column_stack([xtrajectory, ytrajectory])
        # The time at which the car passes each point: the time so far plus
        # distance along the leg divided by speed (time = distance / speed).
        # [1:] skips the first point, which is already the last knot.
        knot_t.extend(t + strajectory[1:] / p["v"])
        # ... and where the car is at those moments.
        knot_xy.extend(xy[1:])
        # Move the clock on by the leg's driving time (length / speed).
        t += pathDistance / p["v"]
        # Add the leg's length to the distance driven.
        L += pathDistance
        # If the end of this leg is a sensor, the car waits there tau seconds
        # while the data is collected: the clock moves on, the car stays put.
        if sensor[k + 1] >= 0:                         # dwell tau_i at sensor i
            # The clock moves on by tau seconds ...
            t += scen["tau"][sensor[k + 1]]
            # ... so add a knot at the end of the wait ...
            knot_t.append(t)
            # ... at the same place: the sensor (the leg's last point).
            knot_xy.append(xy[-1])
    # Turn the knot lists into numpy arrays.
    knot_t, knot_xy = np.array(knot_t), np.array(knot_xy)
    # The frame times 0, 0.5, 1.0, ... up to the end time t (rounded up so the
    # last frame is at or just after the end). math.ceil rounds up.
    times = np.arange(int(math.ceil(t / p["dt"] - 1e-9)) + 1) * p["dt"]
    # Where the car is at each frame time. np.interp reads between the knots in
    # a straight line ("interpolation"), once for x and once for y.
    frames = np.column_stack([np.interp(times, knot_t, knot_xy[:, 0]), np.interp(times, knot_t, knot_xy[:, 1])])
    # Hand back the order, the frames, the dense path, the length L (metres)
    # and the total time T (seconds), which is L / v plus all the waits.
    return {"order": list(order), "frames": frames, "path": knot_xy, "L": L, "T": t}


# How close a line of points comes to the obstacles (equation 19 needs at least r_safe).
def clearance(polyline, geo):
    """Smallest distance between a polyline and the original obstacles; eq. (19) needs >= r_safe."""
    # No obstacles: infinitely far.
    if geo["obstacles"] is None:
        # Hand back infinity.
        return math.inf
    # The smallest distance between the line and the real (not fattened) obstacles.
    return float(shapely.distance(LineString(polyline), geo["obstacles"]))


# How sharply a line of points bends at its sharpest place. "Curvature" is 1
# divided by the radius of the circle that fits the bend: a gentle bend has a
# big circle and a small curvature. The car can do at most 1 / R = 0.2.
# min_step=1.0: use only points at least 1 m apart.
def max_curvature(points, min_step=1.0):
    """Largest curvature (1 / radius of the circle through three consecutive points).
    Points closer than min_step to the previous one are skipped: at very small
    spacings rounding errors would dominate (and stops have zero spacing)."""
    # keep: which points to use; always the first one.
    keep = [0]
    # Keep a point only if it is at least min_step from the last kept point.
    for i in range(1, len(points)):
        # Is point i at least min_step from the last kept point?
        if np.hypot(*(points[i] - points[keep[-1]])) >= min_step:
            # Then keep it.
            keep.append(i)
    # The kept points.
    q = points[keep]
    # Fewer than 3 points make no bend at all.
    if len(q) < 3:
        # Hand back 0: no bend at all.
        return 0.0
    # For every three points in a row, the lengths of their triangle's sides:
    # a (first to middle), b (middle to last) and c (first to last).
    a = np.hypot(*(q[1:-1] - q[:-2]).T)
    # b: from the middle point to the last.
    b = np.hypot(*(q[2:] - q[1:-1]).T)
    # c: from the first point to the last.
    c = np.hypot(*(q[2:] - q[:-2]).T)
    # Two sides of each triangle as arrows from its first point ...
    u, w = q[1:-1] - q[:-2], q[2:] - q[:-2]
    # ... whose cross product is twice the triangle's area (made positive with np.abs).
    twice_area = np.abs(u[:, 0] * w[:, 1] - u[:, 1] * w[:, 0])
    # The circle through three points has radius a * b * c / (4 * area), so its
    # curvature is 4 * area / (a * b * c) = 2 * twice_area / (a * b * c).
    # np.maximum avoids dividing by 0. Hand back the largest one.
    return float(np.max(2 * twice_area / np.maximum(a * b * c, 1e-12)))
