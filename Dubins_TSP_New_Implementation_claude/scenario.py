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
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


def _obstacle(rng, cx, cy):
    """Vertices of one random obstacle centred near (cx, cy): random polygon, L, C or cross shape."""
    kind = rng.choice(["random", "L", "C", "cross"])
    if kind == "random":
        n = rng.integers(4, 7)
        ang = np.sort(rng.uniform(0, 2 * np.pi, n))
        rad = rng.uniform(8, 12, n)
        return np.column_stack([cx + rad * np.cos(ang), cy + rad * np.sin(ang)])
    w, h, t = rng.uniform(10, 20), rng.uniform(15, 25), rng.uniform(4, 8)
    if kind == "L":
        v = [(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]
    elif kind == "C":
        v = [(0, 0), (w, 0), (w, t), (t, t), (t, h - t), (w, h - t), (w, h), (0, h)]
    else:
        v = [(w/2 - t/2, 0), (w/2 + t/2, 0), (w/2 + t/2, h/2 - t/2), (w, h/2 - t/2), (w, h/2 + t/2),
             (w/2 + t/2, h/2 + t/2), (w/2 + t/2, h), (w/2 - t/2, h), (w/2 - t/2, h/2 + t/2),
             (0, h/2 + t/2), (0, h/2 - t/2), (w/2 - t/2, h/2 - t/2)]
    v = np.array(v, dtype=float)
    c = Polygon(v).centroid
    a = rng.uniform(0, 2 * np.pi)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    return (v - [c.x, c.y]) @ rot.T + [cx, cy]


def _obstacles(rng, base, p):
    """Disjoint obstacles whose r_plan-inflated versions neither touch each other nor the base."""
    obstacles, grown = [], []
    for _ in range(rng.integers(p["n_obstacles"][0], p["n_obstacles"][1] + 1)):
        for _ in range(1000):
            cx, cy = rng.uniform(0, p["map_size"], 2)
            if np.hypot(cx - base[0], cy - base[1]) < 25:
                continue
            poly = Polygon(_obstacle(rng, cx, cy))
            if not poly.is_valid:
                continue
            g = poly.buffer(p["r_plan"])
            if g.distance(Point(base)) < p["base_radius"] or any(g.intersects(o) for o in grown):
                continue
            obstacles.append(np.array(poly.exterior.coords[:-1]))
            grown.append(g)
            break
    return obstacles, grown


def _spaced(x, y, taken, attempt, tries):
    """True if (x, y) keeps the minimum spacing (7 m, shrinking as attempts run out) from the taken points."""
    spacing = 7.0 * (1.0 - attempt / tries)
    return not taken or np.min(np.hypot(*(np.array(taken) - [x, y]).T)) >= spacing


def _inner_corners(poly):
    """Concave (inner) corners of a polygon: vertices with an interior angle above 180 degrees."""
    v = np.array(orient(poly, 1.0).exterior.coords[:-1])          # counter-clockwise
    d_in, d_out = v - np.roll(v, 1, axis=0), np.roll(v, -1, axis=0) - v
    return v[d_in[:, 0] * d_out[:, 1] - d_in[:, 1] * d_out[:, 0] < 0]


def _near_points(rng, obstacles, base, count, p):
    """Candidate sensors close to the obstacles; a share around their concave (inner) corners."""
    polys = [Polygon(o) for o in obstacles]
    union = unary_union(polys)
    corners = [_inner_corners(poly) for poly in polys]                 # none for convex obstacles
    lo, hi = p["near_gap"]
    pts, tries = [], 500
    for _ in range(count):
        for attempt in range(tries):
            k = rng.integers(len(polys))
            if len(corners[k]) and rng.random() < p["concave_share"]:
                region = Point(corners[k][rng.integers(len(corners[k]))]).buffer(hi)
            else:
                region = polys[k].buffer(hi)
            x0, y0, x1, y1 = region.bounds
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if not region.contains(Point(x, y)) or not lo <= union.distance(Point(x, y)) <= hi:
                continue
            if np.hypot(x - base[0], y - base[1]) < p["base_radius"] + 1 or not _spaced(x, y, pts, attempt, tries):
                continue
            pts.append((x, y))
            break
    return pts


def _visitable(base, pts, obstacles, p):
    """For each point: can a sink go there and back (base -> point -> base) keeping r_safe from obstacles?"""
    import geodesic                                                      # imported here: these modules use
    import trajectory                                                    # scenarios, not the other way round
    scen = {"base": base, "xy": np.array(pts), "tau": np.zeros(len(pts)), "obstacles": obstacles}
    geo = geodesic.build(scen, p, require_all=False)
    ok = []
    for i in range(len(pts)):
        if not np.isfinite(geo["D"][0, i + 1]):
            ok.append(False)
            continue
        tour = trajectory.build([i], geo, scen, p)
        ok.append(trajectory.clearance(tour["path"], geo) >= p["r_safe"])
    return ok


def make(seed, p):
    """Return {"base": (2,), "xy": (n, 2) sensor positions, "tau": (n,) dwell times,
    "obstacles": list of (m, 2) vertex arrays}."""
    rng = np.random.default_rng(seed)
    base = np.array(p["base"], dtype=float)
    obstacles, grown = _obstacles(rng, base, p)
    n = int(rng.integers(p["n_sensors"][0], p["n_sensors"][1] + 1))

    near = []
    if p["sensor_layout"] == "obstacles" and obstacles:
        n_near = int(round(p["near_share"] * n))
        cand = _near_points(rng, obstacles, base, int(1.5 * n_near) + 5, p)
        near = [c for c, ok in zip(cand, _visitable(base, cand, obstacles, p)) if ok][:n_near]

    # The rest: outside the inflated obstacles (with a little room), away from the base, spread out.
    keep_out = [g.buffer(0.5) for g in grown]
    xy, tries = list(near), 1000
    for _ in range(n - len(near)):
        for attempt in range(tries):
            x, y = rng.uniform(0, p["map_size"], 2)
            if np.hypot(x - base[0], y - base[1]) < p["base_radius"] + 1:
                continue
            if any(g.contains(Point(x, y)) for g in keep_out) or not _spaced(x, y, xy, attempt, tries):
                continue
            xy.append((x, y))
            break
    xy = np.array(xy)
    tau = rng.uniform(p["tau"][0], p["tau"][1], len(xy))
    return {"base": base, "xy": xy, "tau": tau, "obstacles": obstacles, "n_near": len(near)}
