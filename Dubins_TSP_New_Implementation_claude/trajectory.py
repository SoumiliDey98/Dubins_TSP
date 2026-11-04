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
import math

import numpy as np
import shapely
from shapely.geometry import LineString

import dubins_path
import geodesic

HEADINGS = np.linspace(-np.pi, np.pi, 16, endpoint=False)


def waypoints(order, geo, p):
    """Points the sink must pass, and for each one the sensor index (or -1 for corners and the base).
    A corner closer than R to its neighbours is skipped (it would force a small loop), unless
    the shortcut would pass closer than r_safe + 0.5 m to an obstacle."""
    terms = [0] + [i + 1 for i in order] + [0]
    pts, sensor = [geo["nodes"][0]], [-1]
    for a, b in zip(terms[:-1], terms[1:]):
        leg = geodesic.path(geo, a, b)
        for m in range(1, len(leg) - 1):
            close = np.hypot(*(leg[m] - pts[-1])) < p["R"] or np.hypot(*(leg[m] - leg[-1])) < p["R"]
            shortcut_ok = geo["obstacles"] is None or \
                shapely.distance(LineString([pts[-1], leg[m + 1]]), geo["obstacles"]) >= p["r_safe"] + 0.5
            if not (close and shortcut_ok):
                pts.append(leg[m])
                sensor.append(-1)
        pts.append(leg[-1])
        sensor.append(b - 1)
    return np.array(pts), sensor


def bisectors(pts):
    """Heading halfway between the incoming and outgoing directions at every waypoint."""
    h = np.zeros(len(pts))
    for k in range(len(pts)):
        d_in = pts[k] - pts[k - 1] if k > 0 else None
        d_out = pts[k + 1] - pts[k] if k < len(pts) - 1 else None
        if d_in is None:
            u = d_out
        elif d_out is None:
            u = d_in
        else:
            u = d_in / np.linalg.norm(d_in) + d_out / np.linalg.norm(d_out)
            if np.linalg.norm(u) < 1e-9:              # the path turns back: pass sideways
                u = np.array([-d_in[1], d_in[0]])
        h[k] = math.atan2(u[1], u[0])
    return h


def _leg_clear(q0, q1, length, room, obstacles, p):
    """True if the Dubins leg from q0 to q1 keeps r_safe from the obstacles. Quick test
    first: a path of length L between points c apart never leaves the band of half-width
    sqrt(L^2 - c^2) / 2 around the straight segment, which itself has clearance `room`."""
    if obstacles is None:
        return True
    c = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
    if math.sqrt(max(length ** 2 - c ** 2, 0.0)) / 2 <= room - p["r_safe"]:
        return True
    word, seg, _ = dubins_path.select(q0, q1, p["R"], p["dubins_method"])
    xy, _, _ = dubins_path.sample(q0, word, seg, p["R"], p["ds"])
    return shapely.distance(LineString(xy), obstacles) >= p["r_safe"]


def choose_headings(pts, geo, p):
    """Headings that minimise the total Dubins length with every leg clear of the
    obstacles (dynamic programming over the candidate headings of each waypoint).
    Falls back to the bisectors if no clear combination exists (reported by build)."""
    bis = bisectors(pts)
    cand = [np.append(HEADINGS, bis[k]) for k in range(len(pts))]
    cost = np.zeros(len(cand[0]))
    back = []
    for k in range(1, len(pts)):
        L = dubins_path.lengths(pts[k - 1], cand[k - 1][:, None], pts[k], cand[k][None, :], p["R"], p["dubins_method"])
        total = cost[:, None] + L
        room = (shapely.distance(LineString(pts[k - 1:k + 1]), geo["obstacles"])
                if geo["obstacles"] is not None else math.inf)
        new, arg = np.full(len(cand[k]), np.inf), np.zeros(len(cand[k]), dtype=int)
        for m in range(len(cand[k])):
            for i in np.argsort(total[:, m]):                # cheapest predecessor first
                if not np.isfinite(total[i, m]):
                    break
                if _leg_clear((*pts[k - 1], cand[k - 1][i]), (*pts[k], cand[k][m]), L[i, m], room, geo["obstacles"], p):
                    new[m], arg[m] = total[i, m], i
                    break
        cost = new
        back.append(arg)
    if not np.isfinite(cost.min()):
        return bis
    m = int(np.argmin(cost))
    h = [cand[-1][m]]
    for k in range(len(pts) - 1, 0, -1):
        m = back[k - 1][m]
        h.append(cand[k - 1][m])
    return np.array(h[::-1])


def build(order, geo, scen, p):
    """Trajectory of one sink: {"order", "frames" (F, 2), "path" (dense polyline), "L", "T"}."""
    pts, sensor = waypoints(order, geo, p)
    h = choose_headings(pts, geo, p)
    knot_t, knot_xy = [0.0], [pts[0]]
    t = L = 0.0
    for k in range(len(pts) - 1):
        q0, q1 = (*pts[k], h[k]), (*pts[k + 1], h[k + 1])
        word, seg, length = dubins_path.select(q0, q1, p["R"], p["dubins_method"])
        xy, s, _ = dubins_path.sample(q0, word, seg, p["R"], p["ds"])
        knot_t.extend(t + s[1:] / p["v"])
        knot_xy.extend(xy[1:])
        t += length / p["v"]
        L += length
        if sensor[k + 1] >= 0:                         # dwell tau_i at sensor i
            t += scen["tau"][sensor[k + 1]]
            knot_t.append(t)
            knot_xy.append(xy[-1])
    knot_t, knot_xy = np.array(knot_t), np.array(knot_xy)
    times = np.arange(int(math.ceil(t / p["dt"] - 1e-9)) + 1) * p["dt"]
    frames = np.column_stack([np.interp(times, knot_t, knot_xy[:, 0]), np.interp(times, knot_t, knot_xy[:, 1])])
    return {"order": list(order), "frames": frames, "path": knot_xy, "L": L, "T": t}


def clearance(polyline, geo):
    """Smallest distance between a polyline and the original obstacles; eq. (19) needs >= r_safe."""
    if geo["obstacles"] is None:
        return math.inf
    return float(shapely.distance(LineString(polyline), geo["obstacles"]))


def max_curvature(points, min_step=1.0):
    """Largest curvature (1 / radius of the circle through three consecutive points).
    Points closer than min_step to the previous one are skipped: at very small
    spacings rounding errors would dominate (and stops have zero spacing)."""
    keep = [0]
    for i in range(1, len(points)):
        if np.hypot(*(points[i] - points[keep[-1]])) >= min_step:
            keep.append(i)
    q = points[keep]
    if len(q) < 3:
        return 0.0
    a = np.hypot(*(q[1:-1] - q[:-2]).T)
    b = np.hypot(*(q[2:] - q[1:-1]).T)
    c = np.hypot(*(q[2:] - q[:-2]).T)
    u, w = q[1:-1] - q[:-2], q[2:] - q[:-2]
    twice_area = np.abs(u[:, 0] * w[:, 1] - u[:, 1] * w[:, 0])
    return float(np.max(2 * twice_area / np.maximum(a * b * c, 1e-12)))
