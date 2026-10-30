"""Random test maps: disjoint polygonal obstacles and sensors (Sections 0.2, 0.4).

Similar to the generator in main.py of the original code (same shapes, map
size and sensor spacing), but seeded and with spacing based on r_plan, so that
every sensor can be reached.
"""
import numpy as np
from shapely.geometry import Point, Polygon


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


def make(seed, p):
    """Return {"base": (2,), "xy": (n, 2) sensor positions, "tau": (n,) dwell times,
    "obstacles": list of (m, 2) vertex arrays}."""
    rng = np.random.default_rng(seed)
    base = np.array(p["base"], dtype=float)
    size = p["map_size"]

    # Obstacles: disjoint, away from the base, and far enough apart that their
    # inflated versions (r_plan) do not touch, so the free space stays connected.
    obstacles, grown = [], []
    for _ in range(rng.integers(p["n_obstacles"][0], p["n_obstacles"][1] + 1)):
        for _ in range(1000):
            cx, cy = rng.uniform(0, size, 2)
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

    # Sensors: outside the inflated obstacles (with a little room), not on top of
    # the base, and spread out (the minimum spacing shrinks if space runs out).
    keep_out = [g.buffer(0.5) for g in grown]
    xy = []
    for _ in range(rng.integers(p["n_sensors"][0], p["n_sensors"][1] + 1)):
        for attempt in range(1000):
            x, y = rng.uniform(0, size, 2)
            if np.hypot(x - base[0], y - base[1]) < p["base_radius"] + 1:
                continue
            if any(g.contains(Point(x, y)) for g in keep_out):
                continue
            spacing = 7.0 * (1.0 - attempt / 1000.0)
            if xy and np.min(np.hypot(*(np.array(xy) - [x, y]).T)) < spacing:
                continue
            xy.append((x, y))
            break
    xy = np.array(xy)
    tau = rng.uniform(p["tau"][0], p["tau"][1], len(xy))
    return {"base": base, "xy": xy, "tau": tau, "obstacles": obstacles}
