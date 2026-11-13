# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a short
# description written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# It invents a test world: a square map with obstacles (walls, buildings), a
# base station in the middle, and sensors scattered around. The robots (mobile
# sinks) must later visit every sensor. The map is made with random numbers,
# but "seeded" ones: the same seed number always gives the same map, so every
# result can be repeated exactly.
# ==============================================================================
"""Random test maps: disjoint polygonal obstacles and sensors (Sections 0.2, 0.4).

Obstacles: random polygons, L, C and cross shapes (as in main.py of the original
code), disjoint, away from the base, and far enough apart that the free space
stays connected.

Sensors, layout "obstacles" (default): a share near_share of the sensors is placed
close to the obstacles, near_gap[0]..near_gap[1] metres from their boundary;
a share concave_share of those goes around the concave (inner) corners of the
obstacles: the inside corner of an L, the mouth of a C, the notches of a
cross. Such a sensor is kept only if a sink can really visit it: the round trip
base -> sensor -> base, planned like any tour, must keep r_safe from every
obstacle (a forward-only sink cannot enter a pocket too narrow to turn in).
The other sensors are spread uniformly away from the obstacles (layout
"uniform" does this for all sensors). Everything is seeded.
"""
# "import" borrows a toolbox. numpy (nicknamed np) works with long lists of
# numbers, called "arrays", very quickly.
import numpy as np
# shapely is a toolbox for shapes on a flat map. Point is a single spot (x, y);
# Polygon is a closed shape with straight sides, like a floor plan.
from shapely.geometry import Point, Polygon
# orient puts a polygon's corners in a chosen order: anticlockwise or clockwise.
from shapely.geometry.polygon import orient
# unary_union glues many shapes into one big shape (overlaps are merged).
from shapely.ops import unary_union


# "def" starts a recipe (a "function"). This one makes ONE obstacle.
# Its inputs: rng, the random-number maker; and cx, cy, where the obstacle's
# middle should be. The "_" at the start of the name means "only used inside
# this file".
def _obstacle(rng, cx, cy):
    """Vertices of one random obstacle centred near (cx, cy): random polygon, L, C or cross shape."""
    # Pick one of four shapes at random, each equally likely.
    kind = rng.choice(["random", "L", "C", "cross"])
    # "if" asks a question; the indented lines below run only if the answer is yes.
    if kind == "random":
        # A blob with n corners, where n is a random whole number 4, 5 or 6
        # (rng.integers(4, 7) means "from 4 up to, but not including, 7").
        n = rng.integers(4, 7)
        # n random directions (angles) between 0 and 2*pi radians (a full turn,
        # 360 degrees), sorted so we go round the middle in order.
        ang = np.sort(rng.uniform(0, 2 * np.pi, n))
        # For each direction, a random distance from the middle: 8 to 12 metres.
        rad = rng.uniform(8, 12, n)
        # Turn each (direction, distance) into a corner (x, y):
        # x = cx + distance * cos(angle), y = cy + distance * sin(angle).
        # np.column_stack puts the x list and the y list side by side, giving
        # one row (x, y) per corner. "return" hands the corners back and ends
        # the recipe here.
        return np.column_stack([cx + rad * np.cos(ang), cy + rad * np.sin(ang)])
    # For the other three shapes: a random width w (10-20 m), height h (15-25 m)
    # and arm thickness t (4-8 m).
    w, h, t = rng.uniform(10, 20), rng.uniform(15, 25), rng.uniform(4, 8)
    # An L shape: its 6 corners, listed in order round the outline, starting
    # at the bottom-left (0, 0).
    if kind == "L":
        # Its 6 corners, in order round the outline.
        v = [(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]
    # "elif" means "else, if": checked only when the question above was "no".
    # A C shape: 8 corners; the open mouth faces right.
    elif kind == "C":
        # Its 8 corners, in order round the outline.
        v = [(0, 0), (w, 0), (w, t), (t, t), (t, h - t), (w, h - t), (w, h), (0, h)]
    # "else" catches everything left over: here, the cross (plus sign) shape,
    # with 12 corners. w/2 is the middle across, h/2 the middle up; each arm is
    # t thick, so it reaches t/2 either side of the middle.
    else:
        # Its 12 corners, in order round the outline (they go on over three lines).
        v = [(w/2 - t/2, 0), (w/2 + t/2, 0), (w/2 + t/2, h/2 - t/2), (w, h/2 - t/2), (w, h/2 + t/2),
             (w/2 + t/2, h/2 + t/2), (w/2 + t/2, h), (w/2 - t/2, h), (w/2 - t/2, h/2 + t/2),
             (0, h/2 + t/2), (0, h/2 - t/2), (w/2 - t/2, h/2 - t/2)]
    # Turn the list of corners into a numpy array of decimal numbers ("float").
    v = np.array(v, dtype=float)
    # The centroid is the shape's balance point, its centre of mass.
    c = Polygon(v).centroid
    # A random angle a, so the shape is turned to a random direction.
    a = rng.uniform(0, 2 * np.pi)
    # A "rotation matrix": a 2 x 2 table of numbers that, multiplied with a
    # point, turns it round the origin by the angle a.
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    # Move the shape so its centroid sits on (0, 0), turn it ("@" is matrix
    # multiplication; rot.T is the matrix flipped, as the corners are rows),
    # then move it to its place (cx, cy).
    return (v - [c.x, c.y]) @ rot.T + [cx, cy]


# This recipe makes ALL the obstacles of a map, making sure they do not touch.
def _obstacles(rng, base, p):
    """Disjoint obstacles whose r_plan-inflated versions neither touch each other nor the base."""
    # Two empty lists ([] is an empty list): the obstacles, and each obstacle
    # "grown" (made fatter) by r_plan = 7 m.
    obstacles, grown = [], []
    # "for ... in range(...)" repeats the indented lines a number of times. Here
    # the number is random: between 10 and 15 obstacles. "_" is a name for a
    # counter we do not need.
    for _ in range(rng.integers(p["n_obstacles"][0], p["n_obstacles"][1] + 1)):
        # Try up to 1000 times to place this obstacle in an allowed spot.
        for _ in range(1000):
            # A random middle point anywhere on the 100 m x 100 m map.
            cx, cy = rng.uniform(0, p["map_size"], 2)
            # np.hypot gives the straight distance between two points
            # (Pythagoras). Too close to the base (under 25 m)? Then "continue":
            # skip the rest and try another spot.
            if np.hypot(cx - base[0], cy - base[1]) < 25:
                # Skip this spot and try again.
                continue
            # Make the obstacle shape there.
            poly = Polygon(_obstacle(rng, cx, cy))
            # If the shape is broken (for example its outline crosses itself),
            # try again.
            if not poly.is_valid:
                # Skip this spot and try again.
                continue
            # The obstacle made fatter by r_plan (7 m) all round. "buffer" draws
            # a band of that width around the shape.
            g = poly.buffer(p["r_plan"])
            # If the fat version comes within 5 m of the base, or touches the fat
            # version of any obstacle already placed, try again. "any" is true
            # if at least one of the checks is true. This keeps gaps wide
            # enough for a car to turn in.
            if g.distance(Point(base)) < p["base_radius"] or any(g.intersects(o) for o in grown):
                # Skip this spot and try again.
                continue
            # The spot is good: remember the corners (exterior.coords lists the
            # outline; [:-1] drops the last point, which repeats the first).
            obstacles.append(np.array(poly.exterior.coords[:-1]))
            # And remember its fat version too.
            grown.append(g)
            # "break" leaves the 1000-tries loop: this obstacle is done.
            break
    # Hand back both lists.
    return obstacles, grown


# Checks that a new point is not too close to the points already chosen, so
# sensors are spread out.
def _spaced(x, y, taken, attempt, tries):
    """True if (x, y) keeps the minimum spacing (7 m, shrinking as attempts run out) from the taken points."""
    # The required spacing starts at 7 m and shrinks towards 0 as the number of
    # attempts grows, so the search always finds a spot in the end.
    spacing = 7.0 * (1.0 - attempt / tries)
    # "not taken" is true when the list is still empty: then any spot is fine.
    # Otherwise: the distance to the nearest taken point (np.min of all the
    # distances) must be at least the spacing.
    return not taken or np.min(np.hypot(*(np.array(taken) - [x, y]).T)) >= spacing


# Finds the inner corners of an obstacle: corners that point into the shape,
# like the inside corner of an L.
def _inner_corners(poly):
    """Concave (inner) corners of a polygon: vertices with an interior angle above 180 degrees."""
    # The corners in anticlockwise order (orient(..., 1.0) means anticlockwise).
    v = np.array(orient(poly, 1.0).exterior.coords[:-1])          # counter-clockwise
    # d_in: the step arriving at each corner (this corner minus the one before;
    # np.roll shifts the list by one place). d_out: the step leaving it.
    d_in, d_out = v - np.roll(v, 1, axis=0), np.roll(v, -1, axis=0) - v
    # The "cross product" d_in x d_out says which way the outline turns at a
    # corner: positive is a left turn, negative a right turn. Going
    # anticlockwise, a right turn is an inner corner. Keep only those.
    return v[d_in[:, 0] * d_out[:, 1] - d_in[:, 1] * d_out[:, 0] < 0]


# Picks candidate sensor spots close to the obstacles.
def _near_points(rng, obstacles, base, count, p):
    """Candidate sensors close to the obstacles; a share around their concave (inner) corners."""
    # Each obstacle as a Polygon shape.
    polys = [Polygon(o) for o in obstacles]
    # All obstacles glued into one shape, to measure distances to all at once.
    union = unary_union(polys)
    # The inner corners of every obstacle (an empty list for blobs, which have none).
    corners = [_inner_corners(poly) for poly in polys]                 # none for convex obstacles
    # lo and hi: the allowed distance from an obstacle, 3 m and 7 m.
    lo, hi = p["near_gap"]
    # pts collects the chosen spots; we try up to 500 times for each.
    pts, tries = [], 500
    # Do this "count" times: one new spot each time.
    for _ in range(count):
        # "attempt" counts the tries: 0, 1, 2 ... up to 499.
        for attempt in range(tries):
            # Pick one obstacle at random.
            k = rng.integers(len(polys))
            # If it has inner corners, then half the time (rng.random() gives a
            # number between 0 and 1; under 0.5 is half the time) aim near one
            # of them: a circle of radius 7 m round a random inner corner.
            if len(corners[k]) and rng.random() < p["concave_share"]:
                # The region: a circle of radius 7 m round one randomly chosen inner corner.
                region = Point(corners[k][rng.integers(len(corners[k]))]).buffer(hi)
            # Otherwise aim anywhere within 7 m of the obstacle.
            else:
                # The region: everything within 7 m of the obstacle.
                region = polys[k].buffer(hi)
            # The smallest rectangle holding that region: left, bottom, right, top.
            x0, y0, x1, y1 = region.bounds
            # A random point inside that rectangle.
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            # Reject it unless it is inside the region AND between 3 m and 7 m
            # from the nearest obstacle (not inside one, not too far away).
            if not region.contains(Point(x, y)) or not lo <= union.distance(Point(x, y)) <= hi:
                # Rejected: try again.
                continue
            # Reject it if it is within 6 m of the base (5 m docking zone + 1 m),
            # or too close to a spot already chosen.
            if np.hypot(x - base[0], y - base[1]) < p["base_radius"] + 1 or not _spaced(x, y, pts, attempt, tries):
                # Rejected: try again.
                continue
            # Accept: add the spot to the list ("append" adds to the end) and stop trying.
            pts.append((x, y))
            # Found: stop trying for this spot.
            break
    # Hand back all the chosen spots.
    return pts


# For each candidate spot: can a robot car really drive there and back?
def _visitable(base, pts, obstacles, p):
    """For each point: can a sink go there and back (base -> point -> base) keeping r_safe from obstacles?"""
    # These two toolboxes are borrowed here, inside the recipe, and not at the
    # top of the file, because they themselves use maps made by this file.
    import geodesic                                                      # imported here: these modules use
    import trajectory                                                    # scenarios, not the other way round
    # A small test map holding just the candidate spots, with zero waiting time.
    scen = {"base": base, "xy": np.array(pts), "tau": np.zeros(len(pts)), "obstacles": obstacles}
    # Work out the shortest obstacle-free distances on that test map (see
    # geodesic.py). require_all=False: do not stop with an error if some spot
    # cannot be reached at all.
    geo = geodesic.build(scen, p, require_all=False)
    # ok will hold True (yes) or False (no) for each spot.
    ok = []
    # Go through the spots one by one; i is the spot's number.
    for i in range(len(pts)):
        # D[0, i + 1] is the distance from the base (number 0) to spot i
        # (number i + 1). np.isfinite is false for "infinity", which means
        # "no way through". Such a spot is not visitable.
        if not np.isfinite(geo["D"][0, i + 1]):
            # Mark it as not visitable ...
            ok.append(False)
            # ... and go on to the next spot.
            continue
        # Plan a real drivable trip base -> spot -> base (see trajectory.py).
        tour = trajectory.build([i], geo, scen, p)
        # It is visitable if that trip stays at least r_safe (2 m) from every obstacle.
        ok.append(trajectory.clearance(tour["path"], geo) >= p["r_safe"])
    # Hand back the list of yes/no answers, one per spot.
    return ok


# The main recipe of this file: make the whole map for a given seed number.
def make(seed, p):
    """Return {"base": (2,), "xy": (n, 2) sensor positions, "tau": (n,) dwell times,
    "obstacles": list of (m, 2) vertex arrays}."""
    # The random-number maker, started from the seed. Same seed, same numbers,
    # same map.
    rng = np.random.default_rng(seed)
    # The base station's position (50, 50) as a numpy array.
    base = np.array(p["base"], dtype=float)
    # Place the obstacles.
    obstacles, grown = _obstacles(rng, base, p)
    # How many sensors this map gets: a random whole number from 90 to 110.
    n = int(rng.integers(p["n_sensors"][0], p["n_sensors"][1] + 1))

    # near will hold the sensors placed close to obstacles.
    near = []
    # Only for the "obstacles" layout, and only if there are obstacles at all.
    if p["sensor_layout"] == "obstacles" and obstacles:
        # 60 % of n, rounded to a whole number.
        n_near = int(round(p["near_share"] * n))
        # Ask for 1.5 times as many candidates as needed (plus 5), because some
        # will be rejected as not visitable.
        cand = _near_points(rng, obstacles, base, int(1.5 * n_near) + 5, p)
        # Keep only the visitable candidates ("zip" walks through two lists side
        # by side), and of those only the first n_near ([:n_near]).
        near = [c for c, ok in zip(cand, _visitable(base, cand, obstacles, p)) if ok][:n_near]

    # The rest: outside the inflated obstacles (with a little room), away from the base, spread out.
    # The no-go zones: every fat obstacle made 0.5 m fatter still.
    keep_out = [g.buffer(0.5) for g in grown]
    # Start the full sensor list with the near sensors; up to 1000 tries each.
    xy, tries = list(near), 1000
    # Add sensors until there are n in total.
    for _ in range(n - len(near)):
        # Up to 1000 tries for each sensor; "attempt" counts them.
        for attempt in range(tries):
            # A random spot anywhere on the map.
            x, y = rng.uniform(0, p["map_size"], 2)
            # Not within 6 m of the base.
            if np.hypot(x - base[0], y - base[1]) < p["base_radius"] + 1:
                # Too close to the base: try again.
                continue
            # Not inside a no-go zone, and not too close to another sensor.
            if any(g.contains(Point(x, y)) for g in keep_out) or not _spaced(x, y, xy, attempt, tries):
                # In a no-go zone, or too close to another sensor: try again.
                continue
            # Accept it.
            xy.append((x, y))
            # This sensor is placed: stop trying.
            break
    # All sensor positions as one numpy array: one row (x, y) per sensor.
    xy = np.array(xy)
    # A random waiting time tau, 1 to 5 seconds, for each sensor.
    tau = rng.uniform(p["tau"][0], p["tau"][1], len(xy))
    # Hand back the finished map as a dictionary. n_near records how many
    # sensors ended up close to obstacles.
    return {"base": base, "xy": xy, "tau": tau, "obstacles": obstacles, "n_near": len(near)}
