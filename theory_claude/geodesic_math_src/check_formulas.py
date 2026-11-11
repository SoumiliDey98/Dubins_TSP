"""Check the statements of geodesic_math.tex against geodesic.py.

Run from the repository root:
    python theory_claude/geodesic_math_src/check_formulas.py
Part 1 checks the corner construction of the inflated polygons on random obstacles
made by scenario._obstacle. Part 2 builds the graph of geodesic.py on the maps of
seeds 1-10 and checks the properties of D stated in the text.
"""
import math
import os
import sys

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import orient

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                "Dubins_TSP_New_Implementation_claude"))
import config  # noqa: E402
import geodesic  # noqa: E402
import scenario  # noqa: E402

QUANTUM = math.pi / 4          # pi / (2 * quad_segs), quad_segs = 2


def corners(P):
    """Counter-clockwise vertices, turning angle at each, and whether the corner is convex."""
    w = np.array(orient(P, 1.0).exterior.coords[:-1])
    din, dout = w - np.roll(w, 1, 0), np.roll(w, -1, 0) - w
    cross = din[:, 0] * dout[:, 1] - din[:, 1] * dout[:, 0]
    a_in = np.arctan2(din[:, 1], din[:, 0])
    theta = np.mod(np.arctan2(dout[:, 1], dout[:, 0]) - a_in, 2 * math.pi)
    return w, a_in, theta, cross > 0


def part1(trials=3000):
    """At every convex corner, compare the predicted chord vertices with the buffer's vertices
    that lie on the circle of radius r around that corner, inside the corner's angle."""
    rng = np.random.default_rng(5)
    count = dict(match=0, covered=0, crossing=0, differ=0, swallowed=0)
    max_delta, min_ratio, max_ratio = 0.0, 9.0, 0.0
    for _ in range(trials):
        P = Polygon(scenario._obstacle(rng, 0.0, 0.0))
        if not P.is_valid:
            continue
        w, a_in, theta, convex = corners(P)
        for r in (2.5, 7.0):
            B = P.buffer(r, quad_segs=2)
            bv = np.vstack([np.array(B.exterior.coords[:-1])] + [np.array(h.coords[:-1]) for h in B.interiors])
            for k in np.flatnonzero(convex):
                n_chords = max(1, int(theta[k] / QUANTUM + 0.5))
                step = theta[k] / n_chords
                max_delta = max(max_delta, step)
                n0 = a_in[k] - math.pi / 2                    # outward normal of the incoming edge
                pred = w[k] + r * np.column_stack([np.cos(n0 + step * np.arange(n_chords + 1)),
                                                   np.sin(n0 + step * np.arange(n_chords + 1))])
                on = np.abs(np.hypot(*(bv - w[k]).T) - r) < 1e-7
                rel = np.mod(np.arctan2(*(bv[on] - w[k]).T[::-1]) - n0 + 1e-9, 2 * math.pi) - 1e-9
                found = bv[on][rel <= theta[k] + 1e-7]
                if len(found) == 0:
                    count["swallowed"] += 1                    # inside a closed pocket: not on the boundary
                    continue
                d = np.min(np.hypot(*(found[:, None, :] - pred[None, :, :]).transpose(2, 0, 1)), axis=1)
                d2 = np.min(np.hypot(*(pred[:, None, :] - found[None, :, :]).transpose(2, 0, 1)), axis=1)
                if d2.max() >= 1e-7:                           # some predicted vertices lie inside another arm
                    count["covered"] += 1
                    count["crossing"] += d.max() >= 1e-7      # ... and the boundary has a crossing point on the circle
                elif d.max() >= 1e-7:
                    count["differ"] += 1                       # full arc, but a vertex the rule does not give
                else:
                    count["match"] += 1
            dist = np.array([P.distance(Point(x)) for x in bv])
            min_ratio = min(min_ratio, B.exterior.distance(P) / r)
            max_ratio = max(max_ratio, dist.max() / r)
    print(f"Part 1: convex corners: {count['match']} have exactly the predicted vertices; "
          f"{count['covered']} are partly covered by another arm of the obstacle "
          f"({count['crossing']} of them with a crossing point on the circle); {count['swallowed']} are covered entirely; "
          f"{count['differ']} have the whole arc but a vertex the rule does not predict")
    print(f"  largest chord angle {math.degrees(max_delta):.3f} deg (bound 67.5)")
    print(f"  boundary distance / r: min {min_ratio:.5f} (bound cos(3pi/16) = {math.cos(3 * math.pi / 16):.5f}), "
          f"max over vertices {max_ratio:.12f}")


def all_pairs(nodes, n_t, polys):
    """Visibility graph in which every pair is tested against the same polygons."""
    i, j = np.triu_indices(len(nodes), 1)
    lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
    ok = ~shapely.intersects(lines, geodesic._inside(polys))
    i, j = i[ok], j[ok]
    g = csr_matrix((np.hypot(*(nodes[i] - nodes[j]).T), (i, j)), shape=(len(nodes),) * 2)
    return dijkstra(g, directed=False, indices=np.arange(n_t))[:, :n_t]


def part2(seeds=range(1, 11)):
    p = config.default_params()
    print("Part 2: seed, n_t, corner nodes, pairs tested, edges (terminal / corner-corner), "
          "sensors inside r_plan polygons, base paths through another terminal")
    tot = dict(sym=0.0, tri=0.0, euclid=0.0, low=0.0, up=0.0, through=0, paths=0, inside=0, sensors=0)
    for seed in seeds:
        scen = scenario.make(seed, p)
        geo = geodesic.build(scen, p)
        D, n_t, nodes = geo["D"], geo["n_terminals"], geo["nodes"]
        term = nodes[:n_t]
        # graph size, rebuilt as geodesic.build does
        grown = geodesic.inflate(scen["obstacles"], p["r_plan"])
        small = geodesic.inflate(scen["obstacles"], p["r_safe"] + 0.5)
        i, j = np.triu_indices(len(nodes), 1)
        lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
        is_t = (i < n_t) | (j < n_t)
        vis = np.empty(len(i), bool)
        vis[is_t] = ~shapely.intersects(lines[is_t], geodesic._inside(small))
        vis[~is_t] = ~shapely.intersects(lines[~is_t], geodesic._inside(grown))
        inside = int(np.sum(shapely.contains(geodesic._inside(grown), shapely.points(term[1:]))))
        # metric properties
        tot["sym"] = max(tot["sym"], float(np.max(np.abs(D - D.T))))
        tri = D[:, :, None] - (D[:, None, :] + D[None, :, :].transpose(0, 2, 1))     # D[a,c] - D[a,b] - D[b,c]
        tot["tri"] = max(tot["tri"], float(np.max(tri)))
        E = np.hypot(*(term[:, None, :] - term[None, :, :]).transpose(2, 0, 1))
        tot["euclid"] = max(tot["euclid"], float(np.max(E - D)))
        # bounds against single-margin graphs
        n_small = np.vstack([term] + [np.array(g.exterior.coords[:-1]) for g in small])
        n_big = np.vstack([term] + [np.array(g.exterior.coords[:-1]) for g in grown])
        D_low = all_pairs(n_small, n_t, small)
        D_up = all_pairs(n_big, n_t, grown)
        out = ~shapely.contains(geodesic._inside(grown), shapely.points(term))
        both = out[:, None] & out[None, :]
        tot["low"] = max(tot["low"], float(np.max(D_low - D)))
        tot["up"] = max(tot["up"], float(np.max((D - D_up)[both])))
        # shortest paths from the base that pass through another terminal
        through = 0
        for b in range(1, n_t):
            seq = geodesic.path(geo, 0, b)
            inner = [np.flatnonzero((np.abs(term - q) < 1e-12).all(1)) for q in seq[1:-1]]
            through += any(len(x) for x in inner)
        tot["through"] += through
        tot["paths"] += n_t - 1
        tot["inside"] += inside
        tot["sensors"] += n_t - 1
        print(f"  {seed:2d}  {n_t:3d}  {len(nodes) - n_t:4d}  {len(i):6d}  {int(vis[is_t].sum()):5d} / "
              f"{int(vis[~is_t].sum()):5d}  {inside:3d}  {through:3d}")
    print(f"  totals: {tot['inside']} of {tot['sensors']} sensors inside r_plan polygons; "
          f"{tot['through']} of {tot['paths']} base paths pass through another terminal")
    print(f"  max |D - D^T| {tot['sym']:.2e}; max triangle excess {tot['tri']:.2e}; max (Euclid - D) {tot['euclid']:.2e}")
    print(f"  max (D_low - D) {tot['low']:.2e} (<= 0 expected); max (D - D_up) where both outside {tot['up']:.2e} (<= 0 expected)")


if __name__ == "__main__":
    part1()
    part2()
