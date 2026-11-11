"""Figures of geodesic_math.tex, drawn with geodesic.py itself.

Run from the repository root:
    python theory_claude/geodesic_math_src/make_figures.py
It writes fig_inflate.pdf, fig_graph.pdf and fig_seed1.pdf next to this file and prints
the numbers quoted in the captions. Needs LaTeX and cm-super (text.usetex).
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import shapely
from shapely.geometry import Point, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Dubins_TSP_New_Implementation_claude"))
import config  # noqa: E402
import geodesic  # noqa: E402
import scenario  # noqa: E402

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 9,
    "lines.linewidth": 1.0,
    "pdf.fonttype": 42,
})

P = config.default_params()
R_PLAN, R_TERM = P["r_plan"], P["r_safe"] + 0.5
OBST = "0.78"           # obstacle fill
PLAN = "#1f4e9c"        # r_plan polygons and paths
TERM = "#c0661a"        # r_safe + 0.5 polygons
CON = "0.45"            # construction lines
EDGE = "0.80"           # graph edges


def unit(a):
    return np.array([math.cos(a), math.sin(a)])


def ring(ax, poly, **kw):
    x, y = poly.exterior.xy
    ax.plot(x, y, **kw)


def fill(ax, poly):
    x, y = poly.exterior.xy
    ax.fill(x, y, color=OBST, lw=0)


def finish(ax):
    ax.set_aspect("equal")
    ax.axis("off")


def text(ax, p, s, offset=(0, 0), color="black", **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(p[0] + offset[0], p[1] + offset[1], s, color=color, **kw)


# ---------------------------------------------------------------------------
# Figure 1: how an obstacle is inflated
# ---------------------------------------------------------------------------

def inflate_figure():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6.5, 3.0), gridspec_kw=dict(width_ratios=[1.15, 1]))

    # (a) an L-shaped obstacle and its two inflations
    L = Polygon([(0, 0), (16, 0), (16, 6), (6, 6), (6, 20), (0, 20)])
    big, small = geodesic.inflate([np.array(L.exterior.coords[:-1])], R_PLAN)[0], \
        geodesic.inflate([np.array(L.exterior.coords[:-1])], R_TERM)[0]
    fill(ax1, L)
    ring(ax1, L.buffer(R_PLAN, quad_segs=64), color=CON, lw=0.5, ls=(0, (1, 1.5)))
    ring(ax1, big, color=PLAN, lw=1.1)
    ring(ax1, small, color=TERM, lw=1.0, ls=(0, (4, 2)))
    bv = np.array(big.exterior.coords[:-1])
    ax1.plot(*bv.T, "o", ms=2.6, color=PLAN, zorder=5)
    text(ax1, (3, 10), r"$O_k$")
    text(ax1, (15.0, 25.0), r"$r_{\rm plan} = 7$\,m", color=PLAN, ha="left")
    text(ax1, (15.0, 22.2), r"$r_{\rm safe} + 0.5 = 2.5$\,m", color=TERM, ha="left")
    # the reflex corner: offset lines meet
    text(ax1, (6 + R_PLAN, 6 + R_PLAN), r"reflex corner", (3.0, 3.6), ha="left", fontsize=8, color=CON)
    ax1.annotate("", xy=(6 + R_PLAN + 0.3, 6 + R_PLAN + 0.3), xytext=(6 + R_PLAN + 2.8, 6 + R_PLAN + 3.0),
                 arrowprops=dict(arrowstyle="-|>", color=CON, lw=0.6, mutation_scale=6))
    ax1.set_title(r"(a) $O_k$ and its two inflated polygons", fontsize=9)
    finish(ax1)

    # (b) one convex corner: chords with their ends on the circle of radius r
    r = R_PLAN
    theta_deg = 100.0
    a_in = 0.0                                                      # incoming edge along +x
    c = np.array([0.0, 0.0])
    pts = [c - 20 * unit(a_in), c, c + 20 * unit(a_in + math.radians(theta_deg))]
    # a polygon that turns left by theta at c (counter-clockwise), closed far away
    poly = Polygon([pts[0], pts[1], pts[2], pts[0] + 20 * unit(a_in + math.pi / 2 + 0.6)])
    if not poly.is_valid or poly.exterior.is_ccw is False:
        poly = Polygon(poly.exterior.coords[::-1])
    B = poly.buffer(r, quad_segs=2)
    bv = np.array(B.exterior.coords[:-1])
    on = bv[np.abs(np.hypot(*(bv - c).T) - r) < 1e-7]
    rel = np.mod(np.arctan2(*(on - c).T[::-1]) - (a_in - math.pi / 2), 2 * math.pi)
    on = on[np.argsort(rel)][rel[np.argsort(rel)] <= math.radians(theta_deg) + 1e-9]
    n_chords = len(on) - 1
    delta = math.radians(theta_deg) / n_chords

    ax2.fill(*np.array(poly.exterior.coords).T, color=OBST, lw=0)
    t = np.linspace(-math.pi, math.pi, 400)
    ax2.plot(c[0] + r * np.cos(t), c[1] + r * np.sin(t), color=CON, lw=0.5, ls=(0, (1, 1.5)))
    ax2.plot(*on.T, color=PLAN, lw=1.3)
    # offset edges leading into the corner
    ax2.plot([on[0][0] - 6, on[0][0]], [on[0][1], on[0][1]], color=PLAN, lw=1.3)
    e = on[-1] + 6 * unit(a_in + math.radians(theta_deg))
    ax2.plot([on[-1][0], e[0]], [on[-1][1], e[1]], color=PLAN, lw=1.3)
    ax2.plot(*on.T, "o", ms=3, color=PLAN, zorder=5)
    for q in on:
        ax2.plot([c[0], q[0]], [c[1], q[1]], color=CON, lw=0.5)
    n0 = a_in - math.pi / 2                                         # outward normal of the incoming edge
    # theta: from the normal of the incoming edge to that of the outgoing edge
    tt = np.linspace(n0, n0 + math.radians(theta_deg), 60)
    ax2.plot(c[0] + 1.8 * np.cos(tt), c[1] + 1.8 * np.sin(tt), color="black", lw=0.6)
    text(ax2, c + 2.7 * unit(n0 + math.radians(theta_deg) - 0.35), r"$\theta$")
    # Delta: one chord step, shown on the second chord
    tt = np.linspace(n0 + delta, n0 + 2 * delta, 40)
    ax2.plot(c[0] + 4.4 * np.cos(tt), c[1] + 4.4 * np.sin(tt), color="black", lw=0.6)
    text(ax2, c + 5.2 * unit(n0 + 1.5 * delta), r"$\Delta$")
    # distance from the corner to the first chord
    mid = 0.5 * (on[0] + on[1])
    ax2.annotate("", xy=tuple(mid), xytext=tuple(c),
                 arrowprops=dict(arrowstyle="-|>", color=TERM, lw=0.8, mutation_scale=7, shrinkA=0, shrinkB=0))
    text(ax2, (-0.5, -4.2), r"$r\cos\frac{\Delta}{2}$", ha="right", color=TERM, fontsize=8)
    text(ax2, c + r * unit(n0 + math.radians(theta_deg) + 0.55), r"$r$", (0, 0), color=CON)
    text(ax2, c, r"corner of $O_k$", (1.2, 1.6), ha="left", fontsize=8)
    ax2.plot(*c, "o", ms=2.5, color="black")
    ax2.set_xlim(-9, 9)
    ax2.set_ylim(-9.5, 8)
    ax2.set_title(rf"(b) a convex corner: $\theta = {theta_deg:.0f}^\circ$, {n_chords} chords", fontsize=9)
    finish(ax2)
    fig.tight_layout(w_pad=1.0)
    fig.savefig(os.path.join(HERE, "fig_inflate.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"fig_inflate: corner theta={theta_deg} deg -> {n_chords} chords, Delta={math.degrees(delta):.1f} deg, "
          f"sag ratio {math.cos(delta / 2):.4f}; L polygon vertices: r_plan {len(bv)}")


# ---------------------------------------------------------------------------
# Figure 2: the visibility graph of a small map
# ---------------------------------------------------------------------------

SMALL_OBST = [
    np.array([(14, 4), (30, 4), (30, 10), (20, 10), (20, 26), (14, 26)], float),      # L
    np.array([(40, 18), (50, 14), (54, 26), (44, 30)], float),                          # quadrilateral
]
SMALL_BASE = np.array([0.0, 14.0])
SMALL_SENSORS = np.array([(24.0, 14.5), (36.0, -6.0), (62.0, 24.0), (34.0, 34.0)])


def graph_figure():
    scen = {"base": SMALL_BASE, "xy": SMALL_SENSORS, "tau": np.zeros(len(SMALL_SENSORS)),
            "obstacles": SMALL_OBST}
    geo = geodesic.build(scen, P)
    nodes, n_t = geo["nodes"], geo["n_terminals"]
    grown = geodesic.inflate(SMALL_OBST, R_PLAN)
    small = geodesic.inflate(SMALL_OBST, R_TERM)
    i, j = np.triu_indices(len(nodes), 1)
    lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
    is_t = (i < n_t) | (j < n_t)
    vis = np.empty(len(i), bool)
    vis[is_t] = ~shapely.intersects(lines[is_t], geodesic._inside(small))
    vis[~is_t] = ~shapely.intersects(lines[~is_t], geodesic._inside(grown))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6.5, 3.0))
    for ax in (ax1, ax2):
        for o in SMALL_OBST:
            fill(ax, Polygon(o))
        for g in grown:
            ring(ax, g, color=PLAN, lw=0.8)
        for g in small:
            ring(ax, g, color=TERM, lw=0.8, ls=(0, (4, 2)))
        ax.plot(*nodes[n_t:].T, "o", ms=1.8, color=PLAN, zorder=4)
        ax.plot(*SMALL_BASE, "s", ms=5, color="black", zorder=6)
        ax.plot(*SMALL_SENSORS.T, "o", ms=4, color="black", zorder=6)
        text(ax, SMALL_BASE, r"$S_0$", (0, -3.2))
        for k, (s, off) in enumerate(zip(SMALL_SENSORS, [(3.2, -0.6), (2.8, 1.8), (2.4, 2.2), (2.8, -2.2)])):
            text(ax, s, rf"$S_{k + 1}$", off)
        finish(ax)
    for a, b, t in zip(i[vis], j[vis], is_t[vis]):
        ax1.plot(*nodes[[a, b]].T, color=TERM if t else EDGE, lw=0.35 if t else 0.4, zorder=1,
                 alpha=0.55 if t else 1.0)
    ax1.set_title(rf"(a) the {int(vis.sum())} edges of $G$", fontsize=9)
    for b in range(1, n_t):
        seq = geodesic.path(geo, 0, b)
        ax2.plot(*seq.T, color=PLAN, lw=1.5, zorder=5)
    ax2.set_title(r"(b) shortest paths from $S_0$", fontsize=9)
    fig.tight_layout(w_pad=0.6)
    fig.savefig(os.path.join(HERE, "fig_graph.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    D = geo["D"]
    E = np.hypot(*(SMALL_SENSORS - SMALL_BASE).T)
    inside = shapely.contains(geodesic._inside(grown), shapely.points(SMALL_SENSORS))
    print(f"fig_graph: {n_t} terminals, {len(nodes) - n_t} corner nodes, {len(i)} pairs, "
          f"{int(vis[is_t].sum())} terminal edges, {int(vis[~is_t].sum())} corner-corner edges")
    print("  D[0, i] =", np.round(D[0, 1:], 2), " Euclid =", np.round(E, 2), " inside r_plan:", inside)


# ---------------------------------------------------------------------------
# Figure 3: seed 1, every shortest path from the base
# ---------------------------------------------------------------------------

def seed1_figure():
    scen = scenario.make(1, P)
    geo = geodesic.build(scen, P)
    nodes, n_t = geo["nodes"], geo["n_terminals"]
    term = nodes[:n_t]
    grown = geodesic.inflate(scen["obstacles"], R_PLAN)
    inside = shapely.contains(geodesic._inside(grown), shapely.points(term[1:]))

    fig, ax = plt.subplots(figsize=(4.6, 4.6))
    for o in scen["obstacles"]:
        fill(ax, Polygon(o))
    for g in grown:
        ring(ax, g, color=PLAN, lw=0.5, alpha=0.6)
    through, allpts = 0, [term]
    for b in range(1, n_t):
        seq = geodesic.path(geo, 0, b)
        allpts.append(seq)
        hit = any((np.abs(term - q) < 1e-12).all(1).any() for q in seq[1:-1])
        through += hit
        ax.plot(*seq.T, color=TERM if hit else PLAN, lw=0.9 if hit else 0.6, zorder=3 if hit else 2)
    ax.plot(*term[1:][~inside].T, "o", ms=2.6, color="black", zorder=5)
    ax.plot(*term[1:][inside].T, "o", ms=2.8, mfc="white", mec="black", mew=0.7, zorder=5)
    ax.plot(*term[0], "s", ms=6, color="black", zorder=6)
    lo, hi = np.vstack(allpts).min(0) - 2, np.vstack(allpts).max(0) + 2
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    finish(ax)
    fig.savefig(os.path.join(HERE, "fig_seed1.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"fig_seed1: {n_t - 1} sensors, {len(scen['obstacles'])} obstacles, {len(nodes) - n_t} corner nodes, "
          f"{int(inside.sum())} sensors inside r_plan polygons, {through} paths through another terminal")


if __name__ == "__main__":
    inflate_figure()
    graph_figure()
    seed1_figure()
