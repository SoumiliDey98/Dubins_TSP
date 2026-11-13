# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
# 
# WHAT THIS FILE IS FOR
# It draws still pictures (PNG files) that explain the algorithm one step at a
# time: a flow diagram of the whole planner, and one four-panel picture each
# for geodesic.py, ga_tour.py, trajectory.py, partition.py and conflict_game.py.
# Every picture uses the planner's own results and checks that it reproduces
# them exactly. Drawing words (figure, axes, plot, scatter, patch, zorder,
# alpha ...) are explained at the top of plot.py. Here, pictures are laid out in
# INCHES: _axes(fig, x, top, w, h) puts a drawing area x inches from the left
# edge and "top" inches below the top edge, w wide and h high.
# ==============================================================================
"""Still figures that explain the algorithm one step (module) at a time, on one map.

    python module_figures.py --seed 1                    # every figure below (about 1.5 minutes)
    python module_figures.py --only flow geodesic        # these two plan nothing (about 15 s)
    python module_figures.py --only trajectory --sink 3  # another sink; --ga-sink, --split N, --game N likewise

Writes to output/:

    pipeline_flow.png               planner.run as a flow diagram: every step with its module, its step in
                                    Section 0.11, and the data passed between steps (needs no map)
    seed1_geodesic_graph.png        geodesic.py: (a) the input map; (b) the visibility graph, with the lines one
                                    sensor can and cannot use; (c) Dijkstra from the base: the shortest-path tree
                                    and r of eq. 9; (d) part of the output D, against straight-line distances
    seed1_sink14_ga_moves.png       ga_tour.py on one sink's sensors: (a) a start tour (Section 0.8.2);
                                    (b) ordered crossover; (c) swap mutation; (d) one generation, and the
                                    sink's real GA run
    seed1_sink8_trajectory.png      trajectory.py: (a) waypoints and the heading chosen at each; (b) the Dubins
                                    legs, by turn type; (c) the output frames, with the stops at sensors;
                                    (d) distance against time: T = L / v + sum of tau, against B
    seed1_split1_bisection.png      partition.py: (a) the affinity W; (b) the Fiedler vector; (c) the threshold
                                    sweep, min max(T_hat(A_t), T_hat(B_t)); (d) the two halves and their sweep tours
    seed1_game6_pair.png            conflict_game.py, one two-player game: (a) the conflict as planned; (b) each
                                    player's strategies; (c) the cost table and the equilibrium; (d) the distance
                                    between the two before and after

The ga, trajectory, bisection and game figures plan the map once, exactly as planner.py does, and draw the
planner's own GA runs, tours, splits and games; each figure checks that it reproduces them.
"""
# "import" borrows a toolbox. argparse reads options typed on the command line.
import argparse
# os: folders and file names.
import os
# textwrap: breaks long text into lines of a given width.
import textwrap

# matplotlib draws charts and pictures.
import matplotlib

# "Agg": draw into files only, without opening a window.
matplotlib.use("Agg")
# pyplot (nicknamed plt): matplotlib's drawing commands.
import matplotlib.pyplot as plt
# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# LineCollection: many separate lines drawn in one go (fast for thousands).
from matplotlib.collections import LineCollection
# Colour ramps (a smooth run of colours for numbers), and Normalize, which
# turns numbers into positions 0 to 1 along a ramp.
from matplotlib.colors import LinearSegmentedColormap, Normalize
# Line2D: sample lines for legends.
from matplotlib.lines import Line2D
# A filled polygon for drawing.
from matplotlib.patches import Polygon as PolygonPatch
# A filled rectangle.
from matplotlib.patches import Rectangle
# A table that is mostly empty, stored without waste (for the graph).
from scipy.sparse import csr_matrix
# Dijkstra's shortest-route method.
from scipy.sparse.csgraph import dijkstra

# This project's distances round obstacles.
import geodesic
# This project's map maker.
import scenario
# This project's settings.
from config import default_params
# Colours shared with plot.py.
from plot import COLORS, DASHES, GRID, INK, MUTED, QUIET, SURFACE

# The names of the six pictures.
KINDS = ("flow", "geodesic", "ga", "trajectory", "bisection", "game")
# The grey of obstacles.
OBSTACLE = "#9a9ea6"
EDGE = "#b4b3ab"                                   # graph edges: recessive, there are thousands
HIGHLIGHT = COLORS[1]                              # orange: the one sensor followed through (b), (c), (d)
BLOCKED = "#d03b3b"                                # a line that is not an edge; always labelled
TINT, PLAIN = "#fbe3d6", "#f3f2ee"                 # table cells where an obstacle lengthens D, and the others
# Sequential blue (steps 250-700 of the dataviz reference ramp): distance from the base.
RAMP = LinearSegmentedColormap.from_list("r", ["#86b6ef", "#3987e5", "#256abf", "#184f95", "#0d366b"])


# ---------------------------------------------------------------------------
# Shared pieces
# ---------------------------------------------------------------------------

# "def" starts a recipe (a "function"). This one adds a drawing area to the
# page, placed in inches. The "_" at the start means "only used inside this file".
def _axes(fig, x, top, w, h):
    """Axes placed in inches: left edge x, top edge `top` below the top of the figure, size w by h."""
    # The page's width and height in inches.
    W, H = fig.get_size_inches()
    # matplotlib wants page fractions (0 to 1) measured from the bottom-left, so
    # divide by the page size and turn "from the top" into "from the bottom".
    return fig.add_axes([x / W, 1 - (top + h) / H, w / W, h / H])


# Writes a panel's title, with a grey note under it, just above the drawing
# area; room is kept for "lines" lines of note, so titles line up.
def _heading(ax, title, note, width=96, lines=5):
    """Panel title at a fixed height above the axes (room for `lines` lines of note), the wrapped grey note below it."""
    # Break the note into lines of at most 96 letters.
    note = textwrap.fill(note, width)
    # Stop with an error if it needs more lines than there is room for.
    assert note.count("\n") < lines, "the note is longer than the room left for it"
    # How high above the drawing area the title goes, in points (1/72 inch).
    top = 13 + 12.2 * lines
    # The title, that high above the area's top-left corner.
    ax.annotate(title, (0, 1), xycoords="axes fraction", xytext=(0, top), textcoords="offset points",
                fontsize=11.5, fontweight="bold", color=INK, va="bottom")
    # The note just under the title.
    ax.annotate(note, (0, 1), xycoords="axes fraction", xytext=(0, top - 5), textcoords="offset points",
                fontsize=8.5, color=QUIET, va="top", linespacing=1.4)


# Sets up a map panel: equal scales, the area to show, a plain look, a heading.
def _panel(ax, title, note, lims):
    """A map panel: equal axes, recessive grid and ticks, its heading."""
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # One metre across = one metre up.
    ax.set_aspect("equal")
    # If the area shrinks to keep equal scales, keep it in the top-left corner.
    ax.set_anchor("NW")
    # The area to show, across ...
    ax.set_xlim(lims[0][0], lims[1][0])
    # ... and up and down.
    ax.set_ylim(lims[0][1], lims[1][1])
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=7.5, colors=MUTED)
    # The border lines ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # A faint grid ...
    ax.grid(color=GRID, lw=0.5)
    # ... behind everything.
    ax.set_axisbelow(True)
    # The heading.
    _heading(ax, title, note)


# A legend just under a drawing area, in ncol columns.
def _legend(ax, handles, ncol=1):
    # (its top-left corner just below the area's bottom-left corner)
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0, -0.035), ncol=ncol, frameon=False,
              fontsize=8.5, labelcolor=INK, columnspacing=1.6, handlelength=2.2)


# Draws the obstacles as grey shapes.
def _obstacles(ax, scen):
    # Every obstacle ...
    for o in scen["obstacles"]:
        # ... as a filled shape with no outline.
        ax.add_patch(PolygonPatch(o, closed=True, color=OBSTACLE, lw=0, zorder=2))


# Draws a shape's outline; **kw: drawing options handed on.
def _ring(ax, poly, **kw):
    # The outline's x and y values.
    x, y = poly.exterior.xy
    # Draw them.
    ax.plot(x, y, **kw)


# Writes a small label near a point, shifted by "offset" points, on a
# faint background box so it can be read over lines.
def _label(ax, xy, text, offset, color=INK, weight="normal"):
    # (the box: rounded, page-coloured, a little see-through)
    ax.annotate(text, xy, xytext=offset, textcoords="offset points", fontsize=8.5, color=color,
                fontweight=weight, zorder=12, ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", fc=SURFACE, ec="none", alpha=0.85))


# ---------------------------------------------------------------------------
# geodesic.py
# ---------------------------------------------------------------------------

# Chooses five terminals for the table in the geodesic picture.
def _pick_terminals(D, T):
    """Terminals for panel (d): the base; X, the sensor whose geodesic distance from the base exceeds the straight
    line most; Y, the sensor whose distance from X exceeds the straight line most; Z, the most lengthened sensor
    (from the base) more than 15 m from X; A, the sensor furthest from X that the base sees directly."""
    # The straight-line distance between every two terminals (a whole table at once).
    E = np.hypot(*(T[:, None] - T[None]).transpose(2, 0, 1))
    # How many times longer the way round obstacles is than the straight line.
    ratio = np.where(E > 0, D / np.maximum(E, 1e-9), 1.0)
    # X: the sensor with the biggest ratio from the base (+1: sensors start at terminal 1).
    X = 1 + int(np.argmax(ratio[0, 1:]))
    # Y: the sensor with the biggest ratio from X (not X itself).
    Y = 1 + int(np.argmax(np.where(np.arange(1, len(T)) == X, 0, ratio[X, 1:])))
    # Terminals more than 15 m from X ...
    far = E[X] > 15
    # ... but not the base, X or Y ...
    far[[0, X, Y]] = False
    # Z: of those, the biggest ratio from the base.
    Z = int(np.argmax(np.where(far, ratio[0], 0)))
    # Terminals the base sees in a straight line ...
    clear = np.abs(D[0] - E[0]) < 1e-9
    # ... not counting the ones already chosen ...
    clear[[0, X, Y, Z]] = False
    # A: of those, the one furthest from X.
    A = int(np.argmax(np.where(clear, E[X], -1)))
    # Hand back the five (the base first) and the straight-line table.
    return [0, X, Y, Z, A], E


# Panel (d) of the geodesic picture: a 5 x 5 piece of D, and what it is used for.
def _distance_table(ax, D, E, picks, n_t):
    """Panel (d): D for the picked terminals, the straight line in brackets where D is longer, and what
    build returns and who uses it."""
    # Page-coloured background ...
    ax.set_facecolor(SURFACE)
    # ... no axes.
    ax.axis("off")
    # The panel's heading.
    _heading(ax, "(d) Output: part of D [m]",
             f"D is {n_t} × {n_t}: row and column t belong to terminal St (S0 is the base; St is sensor t − 1 in the "
             "code's 0-based ids). Here are five terminals, the ones labelled in (c). Shaded: the straight line "
             "is blocked (an obstacle with its 2.5 m margin is in the way), and its length is in brackets.")
    # The followed sensor X, and the table's size (5).
    X, m = picks[1], len(picks)
    # The row and column names.
    names = ["S0 base"] + [f"S{t}" for t in picks[1:]]
    # The table's top-left corner and each cell's size, in fractions of the panel.
    x0, y0, w, h = 0.15, 0.955, 0.17, 0.1
    # For each row a:
    for a in range(m):
        # the name above column a ...
        ax.text(x0 + (a + 0.5) * w, y0 + 0.012, names[a], transform=ax.transAxes, ha="center", va="bottom",
                fontsize=9, fontweight="bold", color=INK)
        # ... and beside row a;
        ax.text(x0 - 0.02, y0 - (a + 0.5) * h, names[a], transform=ax.transAxes, ha="right", va="center",
                fontsize=9, fontweight="bold", color=INK)
        # for each column b:
        for b in range(m):
            # the two terminals;
            ta, tb = picks[a], picks[b]
            # is the way round longer than the straight line (by over 5 cm)?
            longer = D[ta, tb] > E[ta, tb] + 0.05
            # the cell: tinted if longer, plain if not;
            cell = Rectangle((x0 + b * w + 0.003, y0 - (a + 1) * h + 0.003), w - 0.006, h - 0.006,
                             transform=ax.transAxes, facecolor=TINT if longer else PLAIN, lw=0)
            # the base-to-X cells ...
            if {ta, tb} == {0, X}:
                # ... get an orange border ...
                cell.set_edgecolor(HIGHLIGHT)
                # (thick).
                cell.set_linewidth(1.8)
            # Draw the cell.
            ax.add_patch(cell)
            # Its middle.
            cx, cy = x0 + (b + 0.5) * w, y0 - (a + 0.5) * h
            # On the diagonal (a terminal to itself):
            if a == b:
                # 0.
                ax.text(cx, cy, "0", transform=ax.transAxes, ha="center", va="center", fontsize=10, color=MUTED)
            # Longer than the straight line:
            elif longer:
                # the distance in bold ...
                ax.text(cx, cy + 0.016, f"{D[ta, tb]:.1f}", transform=ax.transAxes, ha="center", va="center",
                        fontsize=10, fontweight="bold", color=INK)
                # ... and the straight line in brackets under it.
                ax.text(cx, cy - 0.021, f"({E[ta, tb]:.1f})", transform=ax.transAxes, ha="center", va="center",
                        fontsize=8.5, color=QUIET)
            # Otherwise:
            else:
                # just the distance.
                ax.text(cx, cy, f"{D[ta, tb]:.1f}", transform=ax.transAxes, ha="center", va="center",
                        fontsize=10, color=INK)
    # The explanation under the table: (name, text) pairs; text None is a heading.
    rows = [
        ("What geodesic.build returns", None),
        ("D[a, b]", f"shortest obstacle-free distance between terminals a and b ({n_t} × {n_t});"),
        ("", "symmetric, and never shorter than the straight line"),
        ("pred[a, v]", "the node before v on the shortest path from terminal a"),
        ("nodes", "where the nodes are; geodesic.path(geo, a, b) follows pred back"),
        ("", "from b to a and returns the corner-to-corner polyline"),
        ("Where the planner uses it", None),
        ("r = D[0, 1:]", "each sensor's distance from the base, eq. 9: the GA's peak count"),
        ("", "m(π) and radially unimodal start tours (ga_tour.py); partition.py"),
        ("Ds = D[1:, 1:]", "sensor to sensor: spectral bisection and workload (partition.py)"),
        ("geodesic.path", "the corners a tour must pass round obstacles (trajectory.waypoints)"),
    ]
    # Start a little below the table.
    y = y0 - m * h - 0.05
    # For each line:
    for key, text in rows:
        # a heading:
        if text is None:
            # a little gap ...
            y -= 0.015
            # ... the heading in bold ...
            ax.text(0.0, y, key, transform=ax.transAxes, fontsize=9.5, fontweight="bold", color=INK, va="top")
            # ... and move down;
            y -= 0.042
        # an entry:
        else:
            # the name in typewriter letters ...
            ax.text(0.02, y, key, transform=ax.transAxes, fontsize=8.8, family="monospace", color=INK, va="top")
            # ... its explanation beside it ...
            ax.text(0.25, y, text, transform=ax.transAxes, fontsize=8.8, color=QUIET, va="top")
            # ... and move down.
            y -= 0.036


# The geodesic picture, in four panels, for map number "seed".
def geodesic_figure(seed, p, path):
    """Save the four-panel figure of geodesic.py and return the numbers it shows."""
    # Make the map.
    scen = scenario.make(seed, p)
    # The distances round obstacles.
    geo = geodesic.build(scen, p)
    # The visibility graph itself: its points and its lines (edges).
    nodes, i, j, length = geodesic.visibility_graph(scen, p)
    # How many terminals, and how many corner points.
    n_t, n_corner = geo["n_terminals"], len(nodes) - geo["n_terminals"]
    # The terminals' positions, the base, and the distance table.
    T, base, D = nodes[:n_t], scen["base"], geo["D"]

    # Dijkstra from the base over every node (corners too), on the very graph that build() used.
    graph = csr_matrix((length, (i, j)), shape=(len(nodes), len(nodes)))
    # (with the graph stored as a table, on the line above)
    dist, pred = dijkstra(graph, directed=False, indices=0, return_predecessors=True)
    # It must give exactly build()'s answers (else stop with an error).
    assert np.array_equal(dist[:n_t], D[0]) and np.array_equal(pred, geo["pred"][0]), "not the graph build() used"

    # The five terminals for the table, and the straight-line distances.
    picks, E = _pick_terminals(D, T)
    # X: the sensor followed through panels (b), (c) and (d).
    X = picks[1]
    # The obstacles grown by 7 m ...
    grown = geodesic.inflate(scen["obstacles"], p["r_plan"])
    # ... and by 2.5 m.
    term = geodesic.inflate(scen["obstacles"], p["r_safe"] + 0.5)
    # Which edges touch a terminal.
    is_term = (i < n_t) | (j < n_t)
    # Which edges touch X.
    mine = (i == X) | (j == X)
    # The area of map to show: all points and the whole 100 m square, 3 m to spare.
    lims = (np.minimum(nodes.min(0), 0) - 3, np.maximum(nodes.max(0), p["map_size"]) + 3)

    # The page height in inches.
    H = 17.5
    # A 14 by 17.5 inch page, 150 pixels per inch.
    fig = plt.figure(figsize=(14, H), dpi=150, facecolor=SURFACE)
    # The title, 0.5 inch from the left and 0.3 inch from the top.
    fig.text(0.5 / 14, 1 - 0.3 / H, f"geodesic.py on seed {seed}: from the map to the geodesic distances D (eq. 9)",
             fontsize=14, fontweight="bold", color=INK, va="top")
    # The subtitle, broken into lines of 150 letters.
    fig.text(0.5 / 14, 1 - 0.62 / H, textwrap.fill("geodesic.build builds the visibility graph (b), runs Dijkstra on it from "
             "every terminal (c), and returns the distances D (d) and pred, from which geodesic.path rebuilds each "
             "path. The graph itself is not kept.", 150), fontsize=9.5, linespacing=1.4, color=QUIET, va="top")
    # Four 6 x 6 inch drawing areas: (a) top left ...
    axa = _axes(fig, 0.55, 2.35, 6.0, 6.0)
    # ... (b) top right ...
    axb = _axes(fig, 7.5, 2.35, 6.0, 6.0)
    # ... (c) bottom left ...
    axc = _axes(fig, 0.55, 10.55, 6.0, 6.0)
    # ... (d) bottom right.
    axd = _axes(fig, 7.5, 10.55, 6.0, 6.0)

    # (a) Input ------------------------------------------------------------------------------------------
    _panel(axa, "(a) Input: the map",
           f"{len(scen['obstacles'])} obstacles, {n_t - 1} sensors and the base station. With the two margins "
           f"r_plan = r_safe + R = {p['r_plan']:g} m and r_safe + 0.5 = {p['r_safe'] + 0.5:g} m, this is all "
           "that geodesic.build reads.", lims)
    # Panel (a), set up above: the obstacles ...
    _obstacles(axa, scen)
    # ... the sensors ...
    axa.scatter(*scen["xy"].T, s=16, color=INK, zorder=5)
    # ... the base ...
    axa.plot(*base, marker="^", ms=12, color=INK, zorder=10)
    # ... and the legend.
    _legend(axa, [Line2D([], [], marker="^", ms=9, color=INK, ls="none", label="base station S0"),
                  Line2D([], [], marker="o", ms=5, color=INK, ls="none", label="sensor"),
                  Rectangle((0, 0), 1, 1, color=OBSTACLE, lw=0, label="obstacle")], ncol=3)

    # (b) Visibility graph -------------------------------------------------------------------------------
    _panel(axb, "(b) The visibility graph",
           f"Nodes: the {n_t} terminals (base and sensors) and the {n_corner} corners of the obstacles grown by "
           f"{p['r_plan']:g} m (blue). An edge joins two nodes that see each other, with its straight-line length "
           f"as cost: {len(i)} edges. A line from a terminal may cross the blue outline but not the dashed 2.5 m one; "
           "a line between two corners must not enter the blue one.", lims)
    # Panel (b), set up above: the obstacles ...
    _obstacles(axb, scen)
    # ... every edge not touching X, as thin faint grey lines ...
    axb.add_collection(LineCollection(np.stack([nodes[i[~mine]], nodes[j[~mine]]], 1), colors=EDGE, lw=0.25,
                                      alpha=0.55, zorder=1))
    # ... the 7 m outlines ...
    for g in grown:
        # (in blue)
        _ring(axb, g, color=COLORS[0], lw=0.9, zorder=3)
    # ... the 2.5 m outlines ...
    for g in term:
        # (grey, dashed)
        _ring(axb, g, color=QUIET, lw=0.8, ls=DASHES, zorder=3)
    # ... X's edges in orange ...
    axb.add_collection(LineCollection(np.stack([nodes[i[mine]], nodes[j[mine]]], 1), colors=HIGHLIGHT, lw=0.9,
                                      zorder=4))
    # ... X's blocked straight line to the base, red dashed ...
    axb.plot(*np.stack([T[X], base]).T, color=BLOCKED, lw=1.6, ls=DASHES, zorder=6)
    # ... labelled half way ...
    _label(axb, 0.5 * (T[X] + base), "blocked: no edge", (0, 9), color=BLOCKED, weight="bold")
    # ... the corner points (blue) ...
    axb.scatter(*nodes[n_t:].T, s=7, color=COLORS[0], zorder=5)
    # ... the sensors ...
    axb.scatter(*scen["xy"].T, s=12, color=INK, zorder=5)
    # ... the base ...
    axb.plot(*base, marker="^", ms=12, color=INK, zorder=10)
    # ... X as a big orange dot ...
    axb.scatter(*T[X], s=70, color=HIGHLIGHT, edgecolor=SURFACE, lw=1.2, zorder=11)
    # ... with its name ...
    _label(axb, T[X], f"S{X}", (0, 11), weight="bold")
    # ... and the legend.
    _legend(axb, [Line2D([], [], color=EDGE, lw=1.2,
                         label=f"edge: {int(is_term.sum())} at a terminal, {int((~is_term).sum())} corner to corner"),
                  Line2D([], [], color=HIGHLIGHT, lw=1.6, label=f"the {int(mine.sum())} edges of sensor S{X}"),
                  Line2D([], [], color=COLORS[0], lw=1.2, marker="o", ms=3.5,
                         label=f"obstacle grown by {p['r_plan']:g} m; corners are nodes"),
                  Line2D([], [], color=QUIET, lw=1.2, ls=DASHES, label=f"obstacle grown by {p['r_safe'] + 0.5:g} m")],
            ncol=2)

    # (c) Dijkstra from the base -------------------------------------------------------------------------
    r = D[0, 1:]
    # Panel (c): its heading (r, the line above, is the base's row of D).
    _panel(axc, "(c) Dijkstra from the base: the shortest-path tree, and r = D[0, 1:]",
           "Dijkstra settles the nodes one at a time, nearest to the base first. From each settled node it tries "
           "every edge, and where that gives a neighbour a shorter distance it keeps it and notes where it came "
           f"from (pred). Following pred back from a sensor gives its shortest path; together they form this tree. "
           f"build runs the same from all {n_t} terminals, one row of D each; this is the run from S0.", lims)
    # The obstacles ...
    _obstacles(axc, scen)
    # ... and the 7 m outlines ...
    for g in grown:
        # (faint).
        _ring(axc, g, color=MUTED, lw=0.5, alpha=0.6, zorder=3)
    # Distances from 0 to the largest are mapped onto the blue ramp.
    norm = Normalize(0, float(np.max(r)))
    # Every point reached from somewhere (pred = -1 means "none").
    kids = np.array([v for v in range(1, len(nodes)) if pred[v] >= 0])
    # The corner points that lie on some shortest path.
    corners = np.intersect1d(np.concatenate([kids, pred[kids]]), np.arange(n_t, len(nodes)))
    # Every tree line, from where a point was reached to the point, coloured
    # by its distance from the base.
    axc.add_collection(LineCollection(np.stack([nodes[pred[kids]], nodes[kids]], 1),
                                      colors=RAMP(norm(dist[kids])), lw=1.3, zorder=4))
    # Those corners, small and grey.
    axc.scatter(*nodes[corners].T, s=6, color=MUTED, zorder=5)
    # X's straight line from the base, dashed.
    axc.plot(*np.stack([base, T[X]]).T, color=INK, lw=1.3, ls=DASHES, zorder=6)
    # X's shortest path, thick orange.
    axc.plot(*geodesic.path(geo, 0, X).T, color=HIGHLIGHT, lw=3.0, zorder=7, solid_capstyle="round")
    # Every sensor, coloured by its distance from the base.
    sc = axc.scatter(*scen["xy"].T, s=30, c=r, cmap=RAMP, norm=norm, edgecolor=SURFACE, lw=0.8, zorder=8)
    # The base.
    axc.plot(*base, marker="^", ms=12, color=INK, zorder=10)
    # The table's four sensors ...
    for t in picks[1:]:
        # ... get their names.
        _label(axc, T[t], f"S{t}", (0, 11), weight="bold")
    # X as a big orange dot.
    axc.scatter(*T[X], s=80, color=HIGHLIGHT, edgecolor=SURFACE, lw=1.2, zorder=11)
    # A thin strip under the panel, for the colour key.
    cax = axc.inset_axes([0.6, -0.085, 0.4, 0.022])
    # The colour key (a "colour bar"), lying flat.
    cb = fig.colorbar(sc, cax=cax, orientation="horizontal")
    # No border round it.
    cb.outline.set_visible(False)
    # Small grey numbers, no marks.
    cb.ax.tick_params(length=0, labelsize=7.5, colors=MUTED)
    # Its label.
    cb.set_label("geodesic distance from the base [m]", fontsize=8, color=MUTED, labelpad=2)
    # The legend.
    _legend(axc, [Line2D([], [], color=RAMP(0.6), lw=1.6, label="tree edge, pred → node"),
                  Line2D([], [], color=HIGHLIGHT, lw=3, label=f"shortest path to S{X}: {D[0, X]:.1f} m"),
                  Line2D([], [], color=INK, lw=1.3, ls=DASHES, label=f"straight line, blocked: {E[0, X]:.1f} m")])

    # (d) Part of the output D ---------------------------------------------------------------------------
    _distance_table(axd, D, E, picks, n_t)

    # Save the picture (panel (d) was drawn on the line above).
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)
    # Hand back the numbers shown, for the printed summary.
    return {"terminals": n_t, "corners": n_corner, "edges": len(i), "terminal_edges": int(is_term.sum()),
            "X": X, "X_edges": int(mine.sum()), "r_X": float(D[0, X]), "straight_X": float(E[0, X]),
            "picks": picks}


# ---------------------------------------------------------------------------
# The whole sequence (planner.py)
# ---------------------------------------------------------------------------

# The flow diagram's box colour and box border colour.
BOX, BOX_EDGE = "#f3f2ee", "#c3c2b7"
PLAN_BAND, GAME_BAND = "#eef4fc", "#fdf1ea"            # the two stages: planning (blue) and conflicts (orange)


# Draws one step of the flow diagram as a rounded box, in the diagram's own
# units (the page is 125 units across and 175 up).
def _box(ax, x, y, w, h, badge, head, ref, lines, fc=BOX):
    """A step: rounded box centred at (x, y), a numbered badge, the function in monospace, its Section 0.11
    step in grey, then plain lines."""
    # A box with rounded corners.
    from matplotlib.patches import FancyBboxPatch
    # The box, centred at (x, y).
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0,rounding_size=1.2",
                                fc=fc, ec=BOX_EDGE, lw=1.0, zorder=3))
    # Where the text starts: 1.6 units below the box's top.
    top = y + h / 2 - 1.6
    # If the step has a number ...
    if badge:
        # ... a black circle ...
        ax.add_patch(plt.Circle((x - w / 2 + 2.4, top - 0.4), 1.5, color=INK, zorder=4))
        # ... with the number in it, in white.
        ax.text(x - w / 2 + 2.4, top - 0.45, badge, color=SURFACE, fontsize=8.5, fontweight="bold", ha="center",
                va="center", zorder=5)
    # The function's name in bold typewriter letters.
    ax.text(x - w / 2 + (4.6 if badge else 1.6), top, head, fontsize=9.5, family="monospace", fontweight="bold",
            color=INK, va="top", zorder=5)
    # If there is a paper reference ...
    if ref:
        # ... it goes on the right, in grey.
        ax.text(x + w / 2 - 1.4, top, ref, fontsize=8, color=QUIET, va="top", ha="right", zorder=5)
    # Each line of description, one under another ...
    for n, line in enumerate(lines):
        # ... the first in black, the rest in grey.
        ax.text(x - w / 2 + 1.6, top - 3.0 - 2.3 * n, line, fontsize=8.8, color=INK if n == 0 else QUIET,
                va="top", zorder=5)


# A diamond: a yes/no question of the flow diagram.
def _diamond(ax, x, y, w, h, text, sub):
    # Its four points: left, top, right, bottom.
    ax.add_patch(PolygonPatch([(x - w / 2, y), (x, y + h / 2), (x + w / 2, y), (x, y - h / 2)], closed=True,
                              fc=SURFACE, ec=INK, lw=1.1, zorder=3))
    # The question ...
    ax.text(x, y + 0.9, text, fontsize=9.5, fontweight="bold", color=INK, ha="center", va="center", zorder=5)
    # ... and a small note under it.
    ax.text(x, y - 1.5, sub, fontsize=8, color=QUIET, ha="center", va="center", zorder=5)


# An arrow along a bent line through the points pts, with an optional label.
def _arrow(ax, pts, color=INK, label=None, at=None, ha="left", lw=1.3):
    """A polyline arrow through pts, its head at the last point; an optional label at `at`."""
    # The points as numbers.
    pts = np.asarray(pts, float)
    # If it bends (more than 2 points), draw all but the last piece as a line.
    ax.plot(*pts[:-1].T, color=color, lw=lw, zorder=2, solid_capstyle="butt") if len(pts) > 2 else None
    # The last piece as an arrow, with its head at the last point.
    ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=2,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=13, shrinkA=0, shrinkB=0))
    # If there is a label ...
    if label:
        # ... write it at "at", on a page-coloured box.
        ax.text(*at, label, fontsize=8.6, color=INK, ha=ha, va="center", zorder=5, linespacing=1.35,
                bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE, ec="none"))


# The flow diagram of the whole planner.
def flow_figure(p, path):
    """The sequence of planner.run, step by step, with what goes in and comes out of each step."""
    # A 12.5 by 17.5 inch page.
    fig = plt.figure(figsize=(12.5, 17.5), dpi=150, facecolor=SURFACE)
    # One drawing area covering the whole page.
    ax = fig.add_axes([0, 0, 1, 1])
    # 125 units across (10 per inch) ...
    ax.set_xlim(0, 125)
    # ... and 175 up.
    ax.set_ylim(0, 175)
    # No axes.
    ax.axis("off")
    # The title ...
    ax.text(3, 172.5, "How planner.run works: each step, with what goes in and what comes out", fontsize=14,
            fontweight="bold", color=INK, va="top")
    # ... and the subtitle.
    ax.text(3, 169.2, "Numbers 0–6 are the steps used in our discussion; grey labels are the steps of Section 0.11 "
            "of Dubins_TSP.pdf. On the arrows: the data passed on.", fontsize=9.5, color=QUIET, va="top")
    X, W = 36, 50                                                      # the main column
    # A light blue band behind the planning steps ...
    ax.add_patch(Rectangle((1.5, 49), 122, 75.6, fc=PLAN_BAND, ec="none", zorder=0))
    # ... and a light orange band behind the conflict steps.
    ax.add_patch(Rectangle((1.5, 17), 122, 30, fc=GAME_BAND, ec="none", zorder=0))
    # The planning band's name ...
    ax.text(122, 123.2, "Plan the routes, one group of sensors at a time\n(Section 0.11, Steps 1–7)", fontsize=9.5,
            fontweight="bold", color=COLORS[0], ha="right", va="top")
    # ... and the conflict band's.
    ax.text(122, 45.5, "Resolve conflicts between sinks\n(Steps 8–12)", fontsize=9.5, fontweight="bold",
            color=COLORS[1], ha="right", va="top")

    # Input and steps 0-2
    ax.text(X, 163.5, f"seed  +  config.py (B = {p['B']:g} s, R = {p['R']:g} m, v = {p['v']:g} m/s, "
            f"d_safe = {p['d_safe']:g} m, ...)", fontsize=9, family="monospace", color=INK, ha="center",
            va="center", bbox=dict(boxstyle="round,pad=0.5", fc=SURFACE, ec=INK, lw=0.8))
    # (The input box is on the line above.) An arrow down to ...
    _arrow(ax, [(X, 161.5), (X, 157.6)])
    # ... step 0, making the map.
    _box(ax, X, 152.5, W, 10, "0", "scenario.make", "input, not in 0.11",
         ["Make the map", "100 m square, 10–15 obstacles, 90–110 sensors, dwell τ 1–5 s"])
    # An arrow labelled with what scenario.make hands on.
    _arrow(ax, [(X, 147.5), (X, 140.6)], label="scen: base (x, y), xy (n × 2 sensor positions),\n"
           "tau (n dwell times), obstacles (polygons)", at=(X + 1.5, 144))
    # Step 1: distances round obstacles.
    _box(ax, X, 135.5, W, 10, "1", "geodesic.build", "Step 2",
         ["Visibility graph, then Dijkstra from every terminal",
          f"margins r_plan = {p['r_plan']:g} m and r_safe + 0.5 = {p['r_safe'] + 0.5:g} m"])
    # An arrow labelled with what geodesic.build hands on.
    _arrow(ax, [(X, 130.5), (X, 123.6)], label="geo: D ((n+1) × (n+1) distances), pred, nodes\n"
           "r = D[0, 1:] (from the base), Ds = D[1:, 1:]", at=(X + 1.5, 127))
    # Step 2: the queue of sensor groups.
    _box(ax, X, 119.5, W, 9, "2", "planner.plan_routes", "Step 1",
         ["Queue of sensor groups", "it starts with one group of all n sensors: one sink"])

    # Step 3: GA, trajectory, budget
    _arrow(ax, [(X, 115), (X, 109.6)], label="the next group in the queue: its sensor ids", at=(X + 1.5, 112.5))
    # (The arrow into it is on the line above.) Step 3a: the GA.
    _box(ax, X, 104.5, W, 10, "3a", "ga_tour.optimize", "Steps 3, 6",
         ["GA: the visiting order of the group",
          f"fitness = straight-line length + m(π)·π·R; {p['pop_size']} tours, {p['generations']} generations"])
    # An arrow labelled with what the GA hands on.
    _arrow(ax, [(X, 99.5), (X, 92.6)], label="order: the group's sensor ids in visiting order", at=(X + 1.5, 96))
    # Step 3b: the drivable path and its time.
    _box(ax, X, 87.5, W, 10, "3b", "trajectory.build", "Steps 2, 4, 6",
         ["Waypoints round obstacles, headings, Dubins legs",
          f"(dubins_path for each leg); R = {p['R']:g} m, v = {p['v']:g} m/s, dt = {p['dt']:g} s"])
    # An arrow labelled with what trajectory.build hands on.
    _arrow(ax, [(X, 82.5), (X, 77.2)], label="tour: path, frames (one position every 0.5 s),\n"
           "L [m], T = L / v + Σ τ [s]", at=(X + 1.5, 79.8))
    # Step 3c: the question "does it fit the budget?".
    _diamond(ax, X, 71.5, 26, 11, "3c  T ≤ B ?", f"B = {p['B']:g} s · Steps 5, 7")
    # no: split, back to the queue
    _box(ax, 101, 71.5, 40, 10, None, "partition.bisect", "Steps 5–7",
         ["Split the group in two", "spectral bisection, min of the larger T̂"], fc=SURFACE)
    # (The split box is on the lines above.) "No": an arrow right to it ...
    _arrow(ax, [(X + 13, 71.5), (81, 71.5)], color=COLORS[0])
    # ... labelled "no: split".
    ax.text(X + 15, 73.2, "no: split", fontsize=8.8, fontweight="bold", color=COLORS[0])
    # From the split box, up and back into the queue.
    _arrow(ax, [(101, 76.5), (101, 119.5), (X + W / 2, 119.5)], color=COLORS[0],
           label="halves A and B (sensor ids)\njoin the back of the queue", at=(102.5, 100))
    # yes: keep, then the next group or the conflict stage
    _arrow(ax, [(X, 66), (X, 61.1)])
    # (The arrow down is on the line above.) It is labelled "yes".
    ax.text(X + 1.2, 63.8, "yes", fontsize=8.8, fontweight="bold", color=INK)
    # Keep the tour: one more robot.
    _box(ax, X, 56.5, W, 9, None, "keep the tour", "",
         ["one more sink", f"(stop if more than K_max = {p['K_max']} sinks would be needed)"])
    # From there, a loop on the left back up to the queue ...
    _arrow(ax, [(X - W / 2, 56.5), (4, 56.5), (4, 119.5), (X - W / 2, 119.5)], color=COLORS[0])
    # ... labelled sideways (rotation=90 turns the text upright).
    ax.text(5.2, 88, "queue not empty:\nnext group", fontsize=8.6, color=COLORS[0], rotation=90, ha="left",
            va="center", fontweight="bold")
    # When the queue is empty: an arrow down to the conflict stage.
    _arrow(ax, [(X, 52), (X, 43.1)], label="queue empty: K tours\n(frames, T, L of each sink)", at=(X + 1.5, 49.5))

    # Steps 4-6: conflicts, repair, check
    _box(ax, X, 38, W, 10, "4", "conflict_game.resolve", "Steps 8–10",
         [f"Players: sinks closer than d_safe = {p['d_safe']:g} m", "each picks GO, WAIT or DETOUR, never past B "
          f"(--resolver simple: take turns)"])
    # (Step 4's box is on the lines above.) An arrow down from it ...
    _arrow(ax, [(X, 33), (X, 28.6)], label="new frames, T, L; one record per game", at=(X + 1.5, 30.8))
    # ... to step 5: "any conflicts left?".
    _diamond(ax, X, 23.5, 30, 10, "5  conflicts left?", f"at most {p['repair_rounds']} repairs · Step 12")
    # The repair box, on the right.
    _box(ax, 101, 23.5, 40, 12, None, "planner.reoptimize", "Step 12",
         ["Re-plan the sinks involved", "reversed order or a new GA run,", "kept if T ≤ B and less risky"],
         fc=SURFACE)
    # "Yes": an arrow right to it ...
    _arrow(ax, [(X + 15, 23.5), (81, 23.5)], color=COLORS[1])
    # ... labelled "yes: repair".
    ax.text(X + 16.5, 25.2, "yes: repair", fontsize=8.8, fontweight="bold", color=COLORS[1])
    # From the repair box, back up to the game.
    _arrow(ax, [(101, 29.5), (101, 38), (X + W / 2, 38)], color=COLORS[1],
           label="new tours for those sinks;\nplay again", at=(102.5, 34))
    # "No": an arrow down to ...
    _arrow(ax, [(X, 18.5), (X, 13.6)], label="no: the round with the least conflict time", at=(X + 1.5, 16))
    # ... step 6, checking every rule.
    _box(ax, X, 9, W, 9, "6", "planner.check", "Step 11",
         ["Measure every constraint on the final result", "(Section 0.7, eqs. 11–20)"])
    # An arrow right to ...
    _arrow(ax, [(X + W / 2, 9), (67, 9)])
    # ... the final output, in a box.
    ax.text(68, 9, "K, T_max, L_total\nevery sensor visited? · every T ≤ B?\nclearance ≥ r_safe? · curvature ≤ 1/R?"
            "\nconflict pairs left, closest approach", fontsize=8.6, color=INK, va="center", linespacing=1.4,
            bbox=dict(boxstyle="round,pad=0.5", fc=SURFACE, ec=INK, lw=0.8))
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)


# ---------------------------------------------------------------------------
# ga_tour.py
# ---------------------------------------------------------------------------

OUT_C, IN_C = COLORS[0], COLORS[1]                  # legs going out (r rises) and coming back (r falls)
OUT_T, IN_T = "#dbe8f9", "#fbe3d6"                  # their light tints, for the gene boxes


# The area to show round some points: padded, with a chosen width-to-height ratio.
def _zoom(points, pad=6.0, aspect=1.0):
    """Limits around the points, padded, with the given width / height ratio."""
    # The points' range, padded.
    lo, hi = points.min(0) - pad, points.max(0) + pad
    # Its middle and size.
    c, size = (lo + hi) / 2, hi - lo
    # A width big enough for both directions ...
    w = max(size[0], size[1] * aspect)
    # ... and the matching height.
    h = w / aspect
    # The two corners, centred on the middle.
    return (c - [w / 2, h / 2], c + [w / 2, h / 2])


# Draws one tour as a row of boxes (one per sensor, like genes on a
# chromosome), at height y, with a label on the left. fills: each box's
# colour; edges, texts: optional border and text colours; sub: optional
# small text under each number.
def _genes(ax, y, seq, label, fills, edges=None, texts=None, sub=None):
    """One tour as a row of boxes (one per sensor), with a label on the left."""
    # The label.
    ax.text(-0.3, y, label, ha="right", va="center", fontsize=9, color=INK)
    # For each sensor g at position n:
    for n, g in enumerate(seq):
        # its border colour (the normal one unless given);
        ec = (edges or {}).get(n, BOX_EDGE)
        # its box (a thicker border if highlighted);
        ax.add_patch(Rectangle((n + 0.06, y - 0.36), 0.88, 0.72, fc=fills[n], ec=ec,
                               lw=2.0 if ec != BOX_EDGE else 0.8, zorder=2))
        # its number, in bold;
        ax.text(n + 0.5, y + (0.08 if sub else 0), str(g), ha="center", va="center", fontsize=9.5, zorder=3,
                color=(texts or {}).get(n, INK), fontweight="bold")
        # and, if given ...
        if sub:
            # ... the small text under it.
            ax.text(n + 0.5, y - 0.2, sub[n], ha="center", va="center", fontsize=7, color=QUIET, zorder=3)


# Draws a tour on a small map: straight arrows, blue going out, orange coming back.
def _tour_map(ax, scen, order, r, lims, title):
    """The tour as straight arrows from the base and back (what the fitness measures): blue where r rises,
    orange where it falls; its radial peaks ringed."""
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # Equal scales.
    ax.set_aspect("equal")
    # The area to show, across ...
    ax.set_xlim(lims[0][0], lims[1][0])
    # ... and up and down.
    ax.set_ylim(lims[0][1], lims[1][1])
    # No numbers across ...
    ax.set_xticks([])
    # ... or up.
    ax.set_yticks([])
    # The border lines ...
    for side in ax.spines:
        # ... light grey.
        ax.spines[side].set_color(GRID)
    # The obstacles ...
    for o in scen["obstacles"]:
        # ... faint (the fitness ignores them).
        ax.add_patch(PolygonPatch(o, closed=True, color=OBSTACLE, alpha=0.45, lw=0, zorder=1))
    # Sensor positions and the base.
    xy, base = scen["xy"], scen["base"]
    # The trip's points: base, sensors, base.
    pts = np.vstack([base, xy[order], base])
    # Their distances from the base.
    rr = np.concatenate([[0.0], r[order], [0.0]])
    # For every leg:
    for a in range(len(pts) - 1):
        # an arrow, blue if the distance grows, orange if it shrinks.
        ax.annotate("", xy=pts[a + 1], xytext=pts[a], zorder=3, arrowprops=dict(
            arrowstyle="-|>", color=OUT_C if rr[a + 1] > rr[a] else IN_C, lw=1.5, mutation_scale=10,
            shrinkA=3, shrinkB=4))
    # The peaks: stops further out than both neighbours.
    peak = [a for a in range(1, len(rr) - 1) if rr[a] > rr[a - 1] and rr[a] > rr[a + 1]]
    # The sensors as black dots.
    ax.scatter(*xy[order].T, s=22, color=INK, zorder=4)
    # A ring round each peak.
    ax.scatter(*pts[peak].T, s=130, facecolor="none", edgecolor=INK, lw=1.3, zorder=4)
    # Each sensor's number ...
    for n, g in enumerate(order):
        # ... just beside it.
        ax.annotate(str(g), xy[g], xytext=(5, 5), textcoords="offset points", fontsize=7.5, color=QUIET, zorder=5)
    # The base.
    ax.plot(*base, marker="^", ms=10, color=INK, zorder=6)
    # The title above the map.
    ax.set_title(title, fontsize=8.5, color=INK, loc="left", pad=3)


# Draws a tour's distance from the base, stop by stop, with its peaks ringed.
def _radial(ax, order, r, top):
    """r along the tour, base to base; peaks ringed; m(pi) in the corner."""
    # This project's GA (for counting peaks).
    import ga_tour
    # The distances, with 0 at the base at both ends.
    rr = np.concatenate([[0.0], r[order], [0.0]])
    # Stop numbers 0, 1, 2 ...
    x = np.arange(len(rr))
    # A plain chart.
    _quiet(ax)
    # Each step from one stop to the next ...
    for a in range(len(rr) - 1):
        # ... a line, blue going up, orange going down.
        ax.plot(x[a:a + 2], rr[a:a + 2], color=OUT_C if rr[a + 1] > rr[a] else IN_C, lw=1.8)
    # A dot at each sensor.
    ax.scatter(x[1:-1], rr[1:-1], s=16, color=INK, zorder=3)
    # The peaks.
    peak = [a for a in range(1, len(rr) - 1) if rr[a] > rr[a - 1] and rr[a] > rr[a + 1]]
    # A ring round each.
    ax.scatter(x[peak], rr[peak], s=110, facecolor="none", edgecolor=INK, lw=1.3, zorder=3)
    # From 0 to a little above the furthest sensor.
    ax.set_ylim(0, top * 1.12)
    # "base" written at both ends.
    ax.set_xticks([0, len(rr) - 1], ["base", "base"])
    # The label up the side.
    ax.set_ylabel("r [m]", fontsize=8, color=MUTED)
    # The number of peaks, top right.
    ax.text(0.98, 0.95, f"m(π) = {ga_tour.peaks(list(order), r)}", transform=ax.transAxes, ha="right", va="top",
            fontsize=9, fontweight="bold", color=INK)


# Makes a chart plain: no border, small grey numbers, light horizontal grid.
def _quiet(ax):
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # The border lines ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=7.5, colors=MUTED)
    # Light horizontal grid lines ...
    ax.grid(axis="y", color=GRID, lw=0.6)
    # ... behind everything.
    ax.set_axisbelow(True)


# A tour's fitness, written out in full, and as a number.
def _fitness_text(order, scen, r, R):
    # This project's GA.
    import ga_tour
    # Its straight-line length and its number of peaks.
    L, m = ga_tour.euclid_length(list(order), scen["xy"], scen["base"]), ga_tour.peaks(list(order), r)
    # For example "fitness = 125.3 + 1·π·5 = 141.0 m", and 141.0.
    return f"fitness = {L:.1f} + {m}·π·{R:g} = {L + m * np.pi * R:.1f} m", L + m * np.pi * R


# The GA picture, on robot k's sensors.
def ga_figure(out, log, k, path):
    """How ga_tour.py makes tours, on the sensors of sink k: a start tour (Section 0.8.2), ordered crossover,
    swap mutation, and one generation, with this group's real GA run. The parents, the cut and the swap are
    chosen for the example (from a start population drawn with its own random numbers); the GA draws them."""
    # This project's GA.
    import ga_tour
    # Short names for the map, the distances and the settings.
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    # Each sensor's distance from the base, and the turning radius.
    r, R = geo["D"][0, 1:], p["R"]
    # Robot k's sensors, in number order.
    ids = np.array(sorted(out["result"]["tours"][k]["order"]))
    # How many.
    n = len(ids)
    # The GA run that made robot k's tour ("next" takes the first match).
    run = next(e for e in log if e.get("kept") and sorted(e["order"]) == list(ids))

    # The example: two start tours, a cut and a swap, picked so that each move changes something visible.
    pop = ga_tour.initial_population(ids, r, p["pop_size"], np.random.default_rng(0))
    # (The example population is made on the line above.) Its first tour, and
    # a second random-number maker started the same way ...
    start, coin_rng = pop[0], np.random.default_rng(0)
    out_set = set((ids[coin_rng.random(n) < 0.5]).tolist())         # the same draw that built pop[0]
    # ... which must rebuild that first tour exactly (else stop with an error).
    # (The "\" continues the statement on the next line.)
    assert start == sorted([g for g in ids.tolist() if g in out_set], key=lambda g: r[g]) + \
        sorted([g for g in ids.tolist() if g not in out_set], key=lambda g: -r[g])
    # Search for a good crossover example: none yet.
    best = None
    # Every pair of different tours (i, j) ...
    for i in range(len(pop)):
        # (j is the second parent)
        for j in range(len(pop)):
            # Two identical parents ...
            if pop[i] == pop[j]:
                # ... are skipped.
                continue
            # ... and every cut from 2 to n - 2:
            for cut in range(2, n - 1):
                # the child;
                child = ga_tour.ordered_crossover(pop[i], pop[j], cut)
                # keep it only if it has 2 or more peaks and is new;
                if ga_tour.peaks(child, r) >= 2 and child not in (pop[i], pop[j]):
                    # prefer a cut near the middle, and early tours;
                    score = abs(cut - n / 2) + 0.1 * i + 0.1 * j
                    # the best score so far ...
                    if best is None or score < best[0]:
                        # ... is remembered.
                        best = (score, i, j, cut, child)
    # The chosen example.
    _, i, j, cut, child = best
    # Its two parents.
    P1, P2 = pop[i], pop[j]
    # Every possible swap that does not just recreate a parent.
    swaps = [(a, b) for a in range(n) for b in range(a + 1, n) if ga_tour.swap(child, a, b) not in (P1, P2)]
    # The swap giving the best fitness.
    a, b = min(swaps, key=lambda s: _fitness_text(ga_tour.swap(child, *s), scen, r, R)[1])
    # The child after that swap.
    mutant = ga_tour.swap(child, a, b)
    # Did it get better?
    better = _fitness_text(mutant, scen, r, R)[1] < _fitness_text(child, scen, r, R)[1]

    # The area to show on the small maps.
    lims = _zoom(np.vstack([scen["xy"][ids], scen["base"][None]]), pad=5)
    # The furthest sensor's distance.
    top = float(r[ids].max())
    # A 14 by 17.6 inch page.
    fig = plt.figure(figsize=(14, 17.6), dpi=150, facecolor=SURFACE)
    # The title.
    fig.text(0.4 / 14, 1 - 0.3 / 17.6, f"ga_tour.py: how the GA builds and changes tours (sink {k}'s {n} sensors, "
             f"seed {out.get('seed', 1)})", fontsize=14, fontweight="bold", color=INK, va="top")
    # The subtitle, broken into lines of 165 letters.
    fig.text(0.4 / 14, 1 - 0.62 / 17.6, textwrap.fill(
        "A tour is a visiting order of the sensors, from the base and back. Lower fitness is better: fitness = "
        "straight-line length + m(π)·π·R (Section 0.8.1), where m(π) is the number of radial peaks, the times "
        "the tour turns back towards the base. Blue legs go away from the base (r rises), orange legs come back; "
        "peaks are ringed. Obstacles are ignored by the fitness, except through r, which is geodesic. Start tours "
        "follow Section 0.8.2. The paper does not say how tours are crossed or mutated, so (b) and (c) are the "
        "code's choices. The parents, cut and swap here are picked for the example; the GA draws them at random.",
        165), fontsize=9.3, color=QUIET, va="top", linespacing=1.4)

    # Writes a row's title and note, top_in inches from the top.
    def heading(top_in, title, note):
        # The title ...
        fig.text(0.4 / 14, 1 - top_in / 17.6, title, fontsize=11.5, fontweight="bold", color=INK, va="top")
        # ... and the note 0.3 inch below it.
        fig.text(0.4 / 14, 1 - (top_in + 0.3) / 17.6, note, fontsize=9, color=QUIET, va="top")

    tops = [2.35, 6.15, 9.95]                                             # top of each row's panels [in]
    # (The rows' tops are on the line above.) Row (a)'s heading ...
    heading(tops[0] - 0.62, "(a) A start tour (Section 0.8.2)", "Each sensor is put at random in S_out or S_in. "
            "S_out is visited by increasing r, then S_in by decreasing r, so every start tour has exactly one peak.")
    # ... row (b)'s ...
    heading(tops[1] - 0.62, "(b) Ordered crossover (ga_tour.ordered_crossover)", f"The first {cut} sensors of "
            "parent 1, then the others in the order they have in parent 2. Two one-peak parents can give a child "
            "with more peaks.")
    # ... and row (c)'s.
    heading(tops[2] - 0.62, f"(c) Swap mutation (ga_tour.swap; each child, probability {p['mutation']:g})",
            "Two sensors change places. Most swaps make a tour worse; " +
            ("the one shown here makes this child better." if better else "the one shown here is the best of them."))

    # A drawing area for the gene boxes in each row.
    gene_axes = [_axes(fig, 1.4, t, 6.0, 2.9) for t in tops]
    # (a)
    ga = gene_axes[0]
    # Row (a), area ga (line above): the sensors sorted by distance.
    by_r = sorted(ids.tolist(), key=lambda g: r[g])
    # The sorted sensors, tinted by the coin toss (S_out blue, S_in orange) ...
    _genes(ga, 2.9, by_r, "sensors by r", [OUT_T if g in out_set else IN_T for g in by_r],
           sub=[f"{r[g]:.0f} m" for g in by_r])
    # ... and the start tour made from them.
    _genes(ga, 1.2, start, "start tour", [OUT_T if g in out_set else IN_T for g in start],
           sub=[f"{r[g]:.0f} m" for g in start])
    # An arrow and the rule between the two rows.
    ga.text(n / 2, 2.05, "↓  S_out by increasing r, then S_in by decreasing r", ha="center", va="center",
            fontsize=8.5, color=QUIET)
    # The legend.
    ga.legend(handles=[Rectangle((0, 0), 1, 1, fc=OUT_T, ec=BOX_EDGE, label="in S_out (drawn at random)"),
                       Rectangle((0, 0), 1, 1, fc=IN_T, ec=BOX_EDGE, label="in S_in")],
              loc="lower left", bbox_to_anchor=(0.02, 0.0), ncol=2, frameon=False, fontsize=8.5)
    # (b)
    gb = gene_axes[1]
    # Row (b), area gb (line above): the first cut sensors of parent 1.
    head = set(P1[:cut])
    # Parent 1, with its first cut sensors in black ...
    _genes(gb, 3.1, P1, "parent 1", [INK if m < cut else BOX for m in range(n)],
           texts={m: SURFACE for m in range(cut)})
    # ... parent 2, with those sensors faded out ...
    _genes(gb, 1.9, P2, "parent 2", [BOX if g in head else "#cfcdc4" for g in P2],
           texts={m: "#b4b3ab" for m, g in enumerate(P2) if g in head})
    # ... and the child: black from parent 1, grey from parent 2.
    _genes(gb, 0.7, child, "child", [INK if m < cut else "#cfcdc4" for m in range(n)],
           texts={m: SURFACE for m in range(cut)})
    # A dashed orange line at the cut ...
    gb.plot([cut, cut], [0.25, 3.55], color=COLORS[1], lw=1.4, ls=DASHES)
    # ... labelled.
    gb.text(cut, 3.62, f"cut = {cut}", ha="center", va="bottom", fontsize=8.5, color=COLORS[1], fontweight="bold")
    # The legend.
    gb.legend(handles=[Rectangle((0, 0), 1, 1, fc=INK, label="from parent 1"),
                       Rectangle((0, 0), 1, 1, fc="#cfcdc4", label="the rest, in parent 2's order")],
              loc="lower left", bbox_to_anchor=(0.02, -0.08), ncol=2, frameon=False, fontsize=8.5)
    # (c)
    gc = gene_axes[2]
    # Row (c), area gc (line above): the child, with the two swapped boxes outlined ...
    _genes(gc, 2.9, child, "child", [BOX] * n, edges={a: COLORS[1], b: COLORS[1]})
    # ... and after the swap.
    _genes(gc, 1.2, mutant, "after swap", [BOX] * n, edges={a: COLORS[1], b: COLORS[1]})
    # Two curved arrows showing the swap:
    for x0, x1 in ((a, b), (b, a)):
        # from each box's old place to its new one.
        gc.annotate("", xy=(x1 + 0.5, 1.62), xytext=(x0 + 0.5, 2.48), arrowprops=dict(
            arrowstyle="-|>", color=COLORS[1], lw=1.2, connectionstyle="arc3,rad=0.15"))
    # Every gene area:
    for g_ax in gene_axes:
        # room for n boxes across ...
        g_ax.set_xlim(-0.2, n + 0.2)
        # ... and the rows up ...
        g_ax.set_ylim(-0.2, 4.0)
        # ... with no axes.
        g_ax.axis("off")

    # For each row (its top t, and the tours it shows):
    for t, tours in zip(tops, [[("start tour", start)], [("parent 1", P1), ("parent 2", P2), ("child", child)],
                               [("child", child), ("after swap", mutant)]]):
        # the last tour is the one drawn;
        name, order = tours[-1]
        # its fitness in words;
        txt, _ = _fitness_text(order, scen, r, R)
        # a small map area ...
        m_ax = _axes(fig, 7.75, t + 0.25, 2.9, 2.9)
        # ... showing it;
        _tour_map(m_ax, scen, order, r, lims, f"{name}: {txt}")
        # a radial chart beside it;
        _radial(_axes(fig, 11.3, t + 0.4, 2.4, 2.5), order, r, top)
        # and the other tours' fitness ...
        if len(tours) > 1:
            # ... written under the map.
            fig.text(7.75 / 14, 1 - (t + 3.25) / 17.6, "\n".join(f"{nm}: {_fitness_text(o, scen, r, R)[0]}"
                                                                for nm, o in tours[:-1]),
                     fontsize=8.3, color=QUIET, va="top", linespacing=1.4)

    # (d) One generation, and this group's real run
    gens = run["generations"]
    # Row (d) (its run's generations: line above): the best fitness of every generation ...
    bestf = [g["fitness"][0] for g in gens]
    # ... and the average.
    meanf = [g["fitness"].mean() for g in gens]
    # Row (d)'s heading.
    heading(13.75, f"(d) One generation, repeated {p['generations']} times (ga_tour.optimize)", textwrap.fill(
        f"Sort the {p['pop_size']} tours by fitness, keep the better half, and refill the other half with children "
        f"of two kept tours picked at random, each mutated with probability {p['mutation']:g}. The best tour never "
        f"gets worse, and after {p['generations']} generations it is the answer. Left: generation 0 of this "
        "group's real run, one bar per tour. Right: the best and the mean fitness of every generation.", 165))
    # The bar chart's drawing area.
    pa = _axes(fig, 0.9, 15.0, 6.2, 2.0)
    # Generation 0's fitness values, best first.
    f0 = gens[0]["fitness"]
    # Half the population: 25.
    half = p["pop_size"] // 2
    # A plain chart.
    _quiet(pa)
    # The better 25 as black bars (kept) ...
    pa.bar(np.arange(half), f0[:half], width=0.8, color=INK, label=f"kept: the better {half}")
    # ... the other 25 as grey bars (replaced).
    pa.bar(np.arange(half, len(f0)), f0[half:], width=0.8, color="#cfcdc4", label=f"replaced by {half} children")
    # The range up.
    pa.set_ylim(f0.min() * 0.9, f0.max() * 1.06)
    # No numbers across.
    pa.set_xticks([])
    # Labels up the side ...
    pa.set_ylabel("fitness [m]", fontsize=8, color=MUTED)
    # ... and along the bottom.
    pa.set_xlabel(f"the {len(f0)} tours of generation 0, best first", fontsize=8, color=MUTED)
    # The legend.
    pa.legend(loc="upper left", frameon=False, fontsize=8, ncol=2)
    # The line chart's drawing area.
    ca = _axes(fig, 7.9, 15.0, 5.8, 2.0)
    # A plain chart.
    _quiet(ca)
    # The average fitness per generation ...
    ca.plot(meanf, color=MUTED, lw=1.2, label="mean of the population")
    # ... and the best.
    ca.plot(bestf, color=INK, lw=2.0, label="best tour")
    # The range up.
    ca.set_ylim(min(bestf) * 0.95, max(meanf) * 1.08)
    # Labels along the bottom ...
    ca.set_xlabel("generation", fontsize=8, color=MUTED)
    # ... and up the side.
    ca.set_ylabel("fitness [m]", fontsize=8, color=MUTED)
    # The legend.
    ca.legend(loc="upper right", frameon=False, fontsize=8)
    # Did the best never change?
    same = abs(bestf[-1] - bestf[0]) < 1e-9
    # The chart's title (saying so if it never changed).
    ca.set_title(f"best: {bestf[0]:.1f} m in generation 0, {bestf[-1]:.1f} m at the end" +
                 (" (the start population already held it)" if same else ""), fontsize=8.5, color=INK, loc="left",
                 pad=4)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)
    # Hand back the numbers, for the printed summary.
    return {"sink": k, "n": n, "cut": cut, "swap": (a, b), "best0": bestf[0], "best_end": bestf[-1]}


# ---------------------------------------------------------------------------
# trajectory.py (with dubins_path.py)
# ---------------------------------------------------------------------------

PART = {"L": COLORS[0], "S": "#52514e", "R": COLORS[1]}   # left arc, straight, right arc


# Sets up a close-up map panel: obstacles, their 2 m safety band, the base, a heading.
def _map_panel(ax, scen, p, lims, title, note, width=96):
    """A close-up map: obstacles, their r_safe boundary, the base; with its heading."""
    # shapely's Polygon, to grow obstacles by r_safe.
    from shapely.geometry import Polygon
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # Equal scales.
    ax.set_aspect("equal")
    # The area to show, across ...
    ax.set_xlim(lims[0][0], lims[1][0])
    # ... and up and down.
    ax.set_ylim(lims[0][1], lims[1][1])
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=7.5, colors=MUTED)
    # The border lines ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # A faint grid ...
    ax.grid(color=GRID, lw=0.5)
    # ... behind everything.
    ax.set_axisbelow(True)
    # Every obstacle:
    for o in scen["obstacles"]:
        # filled grey;
        ax.add_patch(PolygonPatch(o, closed=True, color=OBSTACLE, lw=0, zorder=1))
        # its outline grown by r_safe ...
        x, y = Polygon(o).buffer(p["r_safe"]).exterior.xy
        # ... as a thin dashed line.
        ax.plot(x, y, color=MUTED, lw=0.6, ls=DASHES, zorder=1)
    # The base.
    ax.plot(*scen["base"], marker="^", ms=11, color=INK, zorder=9)
    # The heading, with room for 4 lines of note.
    _heading(ax, title, note, width=width, lines=4)


# The trajectory picture, for robot k.
def trajectory_figure(out, k, path):
    """What trajectory.build does with one sink's visiting order, and what it returns."""
    # This project's Dubins paths ...
    import dubins_path
    # ... and drivable trajectories.
    import trajectory
    # Short names for the map, the distances and the settings.
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    # Robot k's planned tour.
    tour = out["result"]["tours"][k]
    # Its visiting order.
    order = tour["order"]
    # Build it again ...
    again = trajectory.build(order, geo, scen, p)
    # ... which must give exactly the planner's tour (else stop with an error).
    assert np.array_equal(again["frames"], tour["frames"]) and again["L"] == tour["L"], "not the planner's tour"
    # Its waypoints, and which are sensors.
    pts, sensor = trajectory.waypoints(order, geo, p)
    # The heading chosen at each.
    h = trajectory.choose_headings(pts, geo, p)
    # The halfway headings, for comparison.
    bis = trajectory.bisectors(pts)
    # Turning radius, speed, and how shapes are chosen.
    R, v, method = p["R"], p["v"], p["dubins_method"]

    legs, t, curve = [], 0.0, [(0.0, 0.0)]             # curve: (time, distance travelled) at every knot
    # (The leg list, clock and time-distance curve start on the line above.)
    # For every leg a:
    for a in range(len(pts) - 1):
        # its start and end poses;
        q0, q1 = (*pts[a], h[a]), (*pts[a + 1], h[a + 1])
        # its shape and the lengths of its three pieces;
        word, S1, Sm, S2 = dubins_path.findDubinsPath(*q0, *q1, R, method)
        # its points, total length, and each point's distance along it;
        xs, ys, length, s = dubins_path.returnDubinsPath(*q0, *q1, R, method, p["ds"])
        # kept together;
        legs.append({"word": word, "parts": (S1, Sm, S2), "xy": np.column_stack([xs, ys]), "s": s, "L": length})
        # the clock moves on by the driving time;
        t += length / v
        # a point of the curve: (time, distance so far);
        curve.append((t, curve[-1][1] + length))
        # if the leg ends at a sensor ...
        if sensor[a + 1] >= 0:
            # ... the clock moves on by its waiting time ...
            t += scen["tau"][sensor[a + 1]]
            # ... while the distance stays the same.
            curve.append((t, curve[-1][1]))
    # The curve as an array.
    curve = np.array(curve)
    # The legs must add up to the tour's L and T exactly (else stop with an error).
    assert abs(sum(g["L"] for g in legs) - tour["L"]) < 1e-9 and abs(t - tour["T"]) < 1e-9
    # The total length if every heading were the halfway one.
    L_bis = sum(float(dubins_path.returnPathDistances(*pts[a], bis[a], *pts[a + 1], bis[a + 1], R, method))
                for a in range(len(pts) - 1))
    # The total waiting time.
    dwell = float(scen["tau"][order].sum())
    # The straight-line length the GA scored.
    straight = float(np.sum(np.hypot(*np.diff(np.vstack([scen["base"], scen["xy"][order], scen["base"]]),
                                              axis=0).T)))

    # All points to fit on the maps.
    allp = np.vstack([pts, tour["path"]])
    # The area to show, 6 m to spare, shaped like the panels.
    lims = _zoom(allp, pad=6, aspect=6.0 / 5.2)
    # A 14 by 15.8 inch page.
    fig = plt.figure(figsize=(14, 15.8), dpi=150, facecolor=SURFACE)
    # The title.
    fig.text(0.4 / 14, 1 - 0.3 / 15.8, f"trajectory.py: from sink {k}'s visiting order to the path it drives and its "
             f"time T (seed {out.get('seed', 1)})", fontsize=14, fontweight="bold", color=INK, va="top")
    # The subtitle.
    fig.text(0.4 / 14, 1 - 0.62 / 15.8, textwrap.fill(
        f"Input: the GA's order of {len(order)} sensors ({', '.join(map(str, order))}), the geodesic paths of "
        f"geodesic.py and the dwell times τ. Output: path (dense points), frames (one position every {p['dt']:g} s), "
        f"L and T. The GA scored this order as {straight:.1f} m of straight lines; the drivable path is "
        f"{tour['L']:.1f} m.", 165), fontsize=9.3, color=QUIET, va="top", linespacing=1.4)
    # Four 6 x 5.2 inch panels: (a) top left ...
    axa = _axes(fig, 0.55, 2.35, 6.0, 5.2)
    # ... (b) top right ...
    axb = _axes(fig, 7.6, 2.35, 6.0, 5.2)
    # ... (c) bottom left ...
    axc = _axes(fig, 0.55, 9.45, 6.0, 5.2)
    # ... (d) bottom right.
    axd = _axes(fig, 7.6, 9.45, 6.0, 5.2)

    # (a) waypoints and headings
    _map_panel(axa, scen, p, lims, "(a) Waypoints, and the heading chosen at each",
               "The sensors in order, joined by their geodesic paths, whose corners are waypoints too (one closer "
               f"than R = {R:g} m to a neighbour is dropped if the shortcut is clear). At each waypoint, dynamic "
               f"programming picks one of 17 headings "
               f"(16 directions and the turn's bisector) so that the legs are shortest in total and each keeps "
               f"r_safe = {p['r_safe']:g} m from obstacles.")
    # Panel (a), set up above: the GA's straight lines (base, sensors, base) ...
    gx = np.vstack([scen["base"], scen["xy"][order], scen["base"]])
    # ... dotted grey.
    axa.plot(*gx.T, color=MUTED, lw=1.0, ls=(0, (1.5, 2)), zorder=2)
    # The waypoints joined by thin black lines.
    axa.plot(*pts.T, color=INK, lw=0.9, zorder=3)
    # (The 16 directions plus one more; kept for reference.)
    cand = np.append(trajectory.HEADINGS, 0.0)
    # At every waypoint q:
    for a, q in enumerate(pts):
        # each of its 17 candidate headings ...
        for c in np.append(trajectory.HEADINGS, bis[a]):
            # ... as a short thin grey line;
            axa.annotate("", xy=q + 3.2 * np.array([np.cos(c), np.sin(c)]), xytext=q, zorder=4,
                         arrowprops=dict(arrowstyle="-", color="#b4b3ab", lw=0.6, shrinkA=0, shrinkB=0))
        # the chosen heading as a longer blue arrow.
        axa.annotate("", xy=q + 5.5 * np.array([np.cos(h[a]), np.sin(h[a])]), xytext=q, zorder=6,
                     arrowprops=dict(arrowstyle="-|>", color=COLORS[0], lw=2.0, mutation_scale=12, shrinkA=0,
                                     shrinkB=0))
    # Which waypoints are not sensors ...
    corner = np.array([s < 0 for s in sensor])
    # ... not counting the base at both ends: the corners.
    corner[[0, -1]] = False
    # The corners as hollow squares.
    axa.scatter(*pts[corner].T, s=40, marker="s", color=SURFACE, edgecolor=INK, lw=1.2, zorder=7)
    # Count the visits.
    visit = 0
    # Every waypoint:
    for a, s_id in enumerate(sensor):
        # if it is a sensor ...
        if s_id >= 0:
            # ... one more visit ...
            visit += 1
            # ... a black dot ...
            axa.scatter(*pts[a], s=40, color=INK, zorder=7)
            # ... labelled with the visit number and the sensor.
            _label(axa, pts[a], f"{visit}: s{s_id}", (0, -12))
    # The legend.
    _legend(axa, [Line2D([], [], color=MUTED, lw=1.0, ls=(0, (1.5, 2)), label="the GA's straight lines"),
                  Line2D([], [], color=INK, lw=0.9, label="geodesic paths between them"),
                  Line2D([], [], marker="s", ms=6, color=SURFACE, mec=INK, ls="none", label="corner waypoint"),
                  Line2D([], [], color="#b4b3ab", lw=1.0, label="the 17 candidate headings"),
                  Line2D([], [], color=COLORS[0], lw=2.0, label="the heading chosen")], ncol=2)

    # (b) Dubins legs
    _map_panel(axb, scen, p, lims, "(b) One Dubins path per leg (dubins_path.py)",
               f"Each pair of consecutive waypoints, with their headings, is joined by the shortest of the six Dubins "
               f"types (LSL, RSR, LSR, RSL, RLR, LRL): arcs of radius R = {R:g} m and a straight part. With every "
               f"heading set to the bisector instead, the legs would total {L_bis:.1f} m; the chosen headings give "
               f"{tour['L']:.1f} m.")
    # Panel (b), set up above. Every leg:
    for n_leg, g in enumerate(legs):
        # where its pieces start and end (running totals of their lengths);
        bounds = np.cumsum((0.0,) + tuple(g["parts"]))
        # for each of its three pieces:
        for part in range(3):
            # the points that lie on it;
            sel = (g["s"] >= bounds[part] - 1e-9) & (g["s"] <= bounds[part + 1] + 1e-9)
            # if there are at least two ...
            if sel.sum() >= 2:
                # ... draw the piece in its colour (left arc blue, straight grey, right arc orange).
                axb.plot(*g["xy"][sel].T, color=PART[g["word"][part]], lw=2.2, zorder=4, solid_capstyle="butt")
        at = g["xy"][int(len(g["xy"]) * (0.35 if n_leg % 2 else 0.65))]       # alternate, so labels do not meet
        # (The label's place is on the line above.) The leg's shape and length.
        _label(axb, at, f"{g['word']} {g['L']:.1f} m", (0, 11 if n_leg % 2 else -11))
    # The waypoints.
    axb.scatter(*pts[1:-1].T, s=18, color=INK, zorder=6)
    # The legend.
    _legend(axb, [Line2D([], [], color=PART["L"], lw=2.2, label="left arc (L)"),
                  Line2D([], [], color=PART["S"], lw=2.2, label="straight (S)"),
                  Line2D([], [], color=PART["R"], lw=2.2, label="right arc (R)")], ncol=3)

    # (c) frames
    fr = tour["frames"]
    # Panel (c) (its frames: line above): its heading.
    _map_panel(axc, scen, p, lims, f"(c) Output: frames, one position every {p['dt']:g} s",
               f"The sink drives at v = {v:g} m/s, so frames are {v * p['dt']:g} m apart while it moves. At sensor i it "
               f"stops for its dwell time τ_i, so the same position repeats for τ_i / {p['dt']:g} frames. The planner's "
               f"conflict check (Step 4) compares all sinks frame by frame. This tour has {len(fr)} frames.")
    # The dense path, as a wide light line.
    axc.plot(*tour["path"].T, color=GRID, lw=3.0, zorder=2)
    # Every frame as a small black dot.
    axc.scatter(*fr.T, s=9, color=INK, zorder=4)
    # Every sensor of the tour:
    for i in order:
        # how many frames are exactly at it (the frames of its stop);
        n_f = int(np.sum(np.all(np.abs(fr - scen["xy"][i]) < 1e-9, axis=1)))
        # an orange ring;
        axc.scatter(*scen["xy"][i], s=170, facecolor="none", edgecolor=COLORS[1], lw=1.6, zorder=5)
        away = scen["xy"][i] - scen["xy"][order].mean(0)                       # label away from the group
        # (the direction away from the group, on the line above, made length 1)
        away = away / max(np.hypot(*away), 1e-9)
        # a label, placed that way: its waiting time and its number of frames.
        _label(axc, scen["xy"][i], f"τ = {scen['tau'][i]:.1f} s ({n_f} frames)", tuple(28 * away))
    # The legend.
    _legend(axc, [Line2D([], [], marker="o", ms=3.5, color=INK, ls="none", label=f"a frame (every {p['dt']:g} s)"),
                  Line2D([], [], marker="o", ms=10, mfc="none", mec=COLORS[1], ls="none", label="stop at a sensor"),
                  Line2D([], [], color=GRID, lw=3.0, label="path (dense points)")], ncol=3)

    # (d) time
    axd.set_facecolor(SURFACE)
    # Panel (d) (background: line above): a plain chart ...
    _quiet(axd)
    # ... with vertical grid lines too.
    axd.grid(axis="x", color=GRID, lw=0.6)
    # Its heading.
    _heading(axd, "(d) Output: the time T, checked against the budget B",
             f"Distance travelled against time. While the sink drives, the slope is v = {v:g} m/s; at each sensor it "
             f"stands still for τ_i. So T = L / v + Σ τ_i (eq. 17), and the tour is kept only if T ≤ B (eq. 18).",
             lines=4)
    # The distance-against-time curve.
    axd.plot(curve[:, 0], curve[:, 1], color=INK, lw=2.0)
    # The flat parts (waiting at sensors): start time, end time, distance.
    stops = [(curve[a - 1, 0], curve[a, 0], curve[a, 1]) for a in range(1, len(curve)) if curve[a, 1] == curve[a - 1, 1]]
    # Each one ...
    for t0, t1, s in stops:
        # ... drawn as a thick orange bar.
        axd.plot([t0, t1], [s, s], color=COLORS[1], lw=4, solid_capstyle="butt")
    # A dashed line at the budget B ...
    axd.axvline(p["B"], color=MUTED, lw=1.2, ls=DASHES)
    # ... labelled, upright.
    axd.text(p["B"] - 0.8, tour["L"] * 0.06, f"budget B = {p['B']:g} s", rotation=90, ha="right", va="bottom",
             fontsize=8.5, color=MUTED)
    # A line at the mission time T.
    axd.axvline(tour["T"], color=INK, lw=1.0)
    # Time from 0 to just past B ...
    axd.set_xlim(0, p["B"] * 1.05)
    # ... distance from 0 to just past L.
    axd.set_ylim(0, tour["L"] * 1.12)
    # Labels along the bottom ...
    axd.set_xlabel("time t [s]", fontsize=8.5, color=MUTED)
    # ... and up the side.
    axd.set_ylabel("distance travelled [m]", fontsize=8.5, color=MUTED)
    # The sum written out, T = L / v + waiting = ..., at the top.
    axd.text(tour["T"] - 1, tour["L"] * 1.06, f"T = {tour['L']:.1f} / {v:g} + {dwell:.1f} = {tour['T']:.1f} s ≤ B",
             fontsize=9.5, fontweight="bold", color=INK, va="center", ha="right")
    # The legend: driving time and waiting time.
    axd.legend(handles=[Line2D([], [], color=INK, lw=2.0, label=f"driving: L / v = {tour['L'] / v:.1f} s"),
                        Line2D([], [], color=COLORS[1], lw=4, label=f"stopped at sensors: Σ τ = {dwell:.1f} s")],
               loc="upper left", bbox_to_anchor=(0, 0.95), frameon=False, fontsize=8.5, labelcolor=INK)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)
    # Hand back the numbers, for the printed summary.
    return {"sink": k, "waypoints": len(pts), "legs": [g["word"] for g in legs], "L": tour["L"], "L_bisector": L_bis,
            "T": tour["T"], "frames": len(fr)}


# ---------------------------------------------------------------------------
# partition.py
# ---------------------------------------------------------------------------

SIDE_A, SIDE_B = COLORS[0], COLORS[1]               # f <= t (group A) and f > t (group B)
# A colour ramp from blue through grey to orange, for the Fiedler values.
FIEDLER = LinearSegmentedColormap.from_list("f", [SIDE_A, "#d9d8d2", SIDE_B])


# The bisection picture, for the planner's split number "split".
def bisection_figure(out, log, split, path):
    """One split of partition.bisect (Section 0.9), step by step, on the group of the planner's split number
    `split` (0 = the first: all sensors)."""
    # TwoSlopeNorm: maps numbers onto a ramp with a chosen middle (here, the cut).
    from matplotlib.colors import TwoSlopeNorm
    # This project's bisection.
    import partition
    # Short names for the map, the distances and the settings.
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    # Sensor positions, the base, the waiting times.
    xy, base, tau = scen["xy"], scen["base"], scen["tau"]
    # Sensor-to-sensor distances, and distances from the base.
    Ds, r = geo["D"][1:, 1:], geo["D"][0, 1:]
    # The GA runs whose group was split, in order.
    runs = [e for e in log if "halves" in e]
    # The chosen one.
    entry = runs[split]
    # Its sensors.
    ids = np.asarray(entry["ids"])
    # An empty dictionary for the bisection's in-between results.
    tr = {}
    # Do the split again, recording its steps.
    A, B = partition.bisect(ids, xy, base, Ds, r, tau, p, trace=tr)
    # It must give exactly the planner's halves (else stop with an error).
    assert np.array_equal(A, entry["halves"][0]) and np.array_equal(B, entry["halves"][1]), "not the planner's split"
    # The Fiedler values, the affinity, every threshold tried, both halves'
    # times for each, and the best one's number.
    f, W, t_all, wl, best = tr["f"], tr["W"], tr["thresholds"], tr["workloads"], tr["best"]
    # The chosen threshold.
    t_star = t_all[best]
    # The distances within the group ...
    sub = Ds[np.ix_(ids, ids)]
    # ... and the average distance to the nearest neighbour.
    nn = np.where(np.eye(len(ids), dtype=bool), np.inf, sub).min(axis=1).mean()
    # The estimated time of each half.
    wA, wB = partition.workload(A, xy, base, Ds, r, tau, p["v"]), partition.workload(B, xy, base, Ds, r, tau, p["v"])
    # The halves' own GA runs, if they happened (looked up by group size).
    later = {len(e["ids"]): e for e in log if any(np.array_equal(e["ids"], h) for h in (A, B))}

    # The group's sensors and the base ...
    pts = np.vstack([xy[ids], base[None]])
    # ... give the area to show, 8 m to spare.
    lims = _zoom(pts, pad=8)
    # A 14 by 17.9 inch page ...
    fig = plt.figure(figsize=(14, 17.9), dpi=150, facecolor=SURFACE)
    # ... (its height).
    H = 17.9
    # The title.
    fig.text(0.4 / 14, 1 - 0.3 / H, f"partition.py: how a group is split in two (split {split + 1} of {len(runs)} on "
             f"seed {out.get('seed', 1)}: {len(ids)} sensors whose tour took T = {entry['T']:.0f} s > B = "
             f"{p['B']:g} s)", fontsize=14, fontweight="bold", color=INK, va="top")
    # The subtitle.
    fig.text(0.4 / 14, 1 - 0.62 / H, textwrap.fill(
        "Section 0.9.1, steps 1–7. The split uses geodesic distances, so two sensors on opposite sides of a wall "
        "count as far apart. It looks for a line that cuts few strong links (spectral bisection) and, among the "
        "cuts along the Fiedler vector, keeps the one whose larger half has the smallest estimated time T̂.", 170),
        fontsize=9.3, color=QUIET, va="top", linespacing=1.4)
    # Four panels: (a) top left ...
    axa = _axes(fig, 0.55, 2.2, 6.0, 6.0)
    # ... (b) top right ...
    axb = _axes(fig, 7.6, 2.2, 6.0, 6.0)
    # ... (c) bottom left ...
    axc = _axes(fig, 0.9, 10.95, 5.7, 5.4)
    # ... (d) bottom right.
    axd = _axes(fig, 7.6, 10.95, 6.0, 6.0)

    # (a) affinity
    _map_panel(axa, scen, p, lims, "(a) Steps 1–2: affinity W between every two sensors",
               f"W_ij = exp(−(d_geo(i, j) / σ)²), with d_geo from geodesic.py. σ = max(mean nearest-neighbour "
               f"distance {nn:.1f} m, R = {p['R']:g} m) = {tr['sigma']:.1f} m. Lines show pairs with W_ij > 0.05; "
               "thicker is stronger. Sensors near each other but on two sides of an obstacle get a weak link.")
    # Panel (a), set up above. Every pair of sensors once.
    i, j = np.triu_indices(len(ids), 1)
    # Pairs whose link is stronger than 0.05.
    strong = W[i, j] > 0.05
    # A line for each such pair ...
    segs = np.stack([xy[ids[i[strong]]], xy[ids[j[strong]]]], 1)
    # ... drawn thicker for stronger links.
    axa.add_collection(LineCollection(segs, colors=INK, lw=0.3 + 2.2 * W[i, j][strong], alpha=0.55, zorder=3))
    # The sensors.
    axa.scatter(*xy[ids].T, s=14, color=INK, zorder=5)
    # The legend.
    _legend(axa, [Line2D([], [], color=INK, lw=2.4, alpha=0.6, label="W ≈ 1 (very close)"),
                  Line2D([], [], color=INK, lw=0.5, alpha=0.6, label="W = 0.05"),
                  Line2D([], [], marker="o", ms=4, color=INK, ls="none", label="sensor")], ncol=3)

    # (b) Fiedler vector, coloured by rank: step 7 uses only the order of the values
    rank = np.argsort(np.argsort(f))
    # Panel (b). (Each sensor's rank in f is on the line above.) Ranks onto the
    # ramp, with grey exactly at the cut.
    norm = TwoSlopeNorm(vcenter=len(A) - 0.5, vmin=0, vmax=len(ids) - 1)
    # The two sensors whose f is furthest from the middle value.
    outl = np.argsort(-np.abs(f - np.median(f)))[:2]
    # The group's distances again (a copy, to change) ...
    sub = Ds[np.ix_(ids, ids)].copy()
    # ... with each sensor's distance to itself set to infinity.
    np.fill_diagonal(sub, np.inf)
    # The share of sensors whose nearest neighbour is in the same half.
    same = float(np.mean([(a in set(A.tolist())) == (b in set(A.tolist())) for a, b in zip(ids, ids[np.argmin(sub, 1)])]))
    # Panel (b)'s map and heading.
    _map_panel(axb, scen, p, lims, "(b) Steps 3–6: the Fiedler vector",
               "Degrees d_i = Σ_j W_ij, normalised Laplacian I − D^(−1/2) W D^(−1/2), and its eigenvectors. The one "
               f"for the second-smallest eigenvalue (λ₂ = {tr['eigenvalues'][1]:.3f}) gives each sensor a value f_i; "
               "linked sensors get similar values. Colour: the order of f_i, which is all step 7 uses; grey where (c) "
               "cuts.")
    # Each sensor coloured by its rank in f.
    sc = axb.scatter(*xy[ids].T, c=rank, cmap=FIEDLER, norm=norm, s=46, edgecolor=SURFACE, lw=0.8, zorder=5)
    # The two far-out sensors ...
    for n_q, q in enumerate(outl):
        # ... labelled with their f (one above, one below).
        _label(axb, xy[ids[q]], f"s{ids[q]}: f = {f[q]:.2f}", (0, 12) if n_q == 0 else (0, -12), weight="bold")
    # A strip under the panel for the colour key.
    cax = axb.inset_axes([0.45, -0.085, 0.55, 0.022])
    # The colour key, lying flat.
    cb = fig.colorbar(sc, cax=cax, orientation="horizontal")
    # No border.
    cb.outline.set_visible(False)
    # Three labels: lowest f, the cut, highest f.
    cb.set_ticks([0, len(A) - 0.5, len(ids) - 1], labels=[f"lowest f\n{f.min():.3f}", f"cut\nt = {t_star:.3f}",
                                                         f"highest f\n{f.max():.2f}"])
    # Small grey labels, no marks.
    cb.ax.tick_params(length=0, labelsize=7, colors=MUTED)
    # A note under the key about the far-out pair.
    axb.text(0.0, -0.16, textwrap.fill(
        f"Here f is far from the rest only for the two labelled sensors, a weakly linked pair; the others have small "
        f"values, but their order still groups neighbours: {100 * same:.0f} % of the sensors have their nearest "
        "neighbour in the same half.", 105), transform=axb.transAxes, fontsize=8.3, color=QUIET, va="top",
             linespacing=1.4)

    # (c) threshold sweep, against the number of sensors in A_t
    size = np.array([np.sum(f <= t) for t in t_all])
    # Panel (c). (Each threshold's |A_t| is on the line above.) A plain chart ...
    _quiet(axc)
    # ... with vertical grid lines too.
    axc.grid(axis="x", color=GRID, lw=0.6)
    # Its heading.
    _heading(axc, "(c) Step 7: the threshold t, chosen by min max(T̂(A_t), T̂(B_t))",
             "Every value of f is tried as a threshold t: A_t = {f_i ≤ t}, B_t = {f_i > t}, so A_t gains one sensor "
             "at a time. T̂ is a quick estimate of a group's time: a sweep round the base by angle, with geodesic "
             "legs, divided by v, plus Σ τ (drawn in (d)). The larger of the two T̂ is smallest near the crossing.")
    # A's estimated time for each threshold (blue) ...
    axc.plot(size, wl[:, 0], color=SIDE_A, lw=1.6, label="T̂(A_t): sensors with f ≤ t")
    # ... B's (orange) ...
    axc.plot(size, wl[:, 1], color=SIDE_B, lw=1.6, label="T̂(B_t): sensors with f > t")
    # ... and the larger of the two (thick black).
    axc.plot(size, wl.max(1), color=INK, lw=2.6, label="the larger of the two")
    # A dashed line at the chosen cut ...
    axc.axvline(size[best], color=INK, lw=1.0, ls=DASHES)
    # ... a dot at its larger time ...
    axc.scatter([size[best]], [wl[best].max()], s=60, color=INK, zorder=5)
    # ... and a note about it, joined by a thin line.
    axc.annotate(f"chosen: t = {t_star:.3f}\n|A| = {len(A)}, |B| = {len(B)}\nmax T̂ = {wl[best].max():.0f} s",
                 (size[best], wl[best].max()), xytext=(0.05 * len(ids), 0.6 * wl.max()), textcoords="data",
                 fontsize=8.8, color=INK, va="top",
                 arrowprops=dict(arrowstyle="-", color=INK, lw=0.8))
    # Times up ...
    axc.set_ylim(0, wl.max() * 1.08)
    # ... sizes of A across.
    axc.set_xlim(0, len(ids))
    # Labels along the bottom ...
    axc.set_xlabel("sensors in A_t (those with the lowest f)", fontsize=8.5, color=MUTED)
    # ... and up the side.
    axc.set_ylabel("estimated time T̂ [s]", fontsize=8.5, color=MUTED)
    # The legend.
    axc.legend(loc="upper center", frameon=False, fontsize=8.5, labelcolor=INK)

    # (d) result
    _map_panel(axd, scen, p, lims, "(d) Result: the two halves, and the sweep tours behind T̂",
               f"A: {len(A)} sensors, T̂ = {wA:.0f} s; B: {len(B)} sensors, T̂ = {wB:.0f} s. Each half joins the back "
               "of the queue; its own GA run and Dubins path give its real T" +
               (": " + ", ".join(f"T = {later[len(h)]['T']:.0f} s for {nm}" for nm, h in (("A", A), ("B", B))
                                 if len(h) in later) if later else "") + ". A half with T > B is split again.")
    # Panel (d), set up above. Each half, in its colour:
    for grp, col in ((A, SIDE_A), (B, SIDE_B)):
        # its sensors in clock-hand order;
        seq = partition.sweep_order(np.asarray(grp), xy, base)
        # the ways round obstacles: base to first, each to the next, last to base
        # (the "\" continues the line);
        legs = [geodesic.path(geo, 0, seq[0] + 1)] + \
               [geodesic.path(geo, a + 1, b + 1) for a, b in zip(seq[:-1], seq[1:])] + [geodesic.path(geo, seq[-1] + 1, 0)]
        # each one ...
        for leg in legs:
            # ... drawn in the half's colour;
            axd.plot(*leg.T, color=col, lw=1.1, alpha=0.75, zorder=3)
        # its sensors.
        axd.scatter(*xy[grp].T, s=40, color=col, edgecolor=SURFACE, lw=0.8, zorder=5)
    # The legend.
    _legend(axd, [Line2D([], [], marker="o", ms=6, color=SIDE_A, lw=1.1, label=f"A: f ≤ t ({len(A)} sensors)"),
                  Line2D([], [], marker="o", ms=6, color=SIDE_B, lw=1.1, label=f"B: f > t ({len(B)} sensors)")],
            ncol=2)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)
    # Hand back the numbers, for the printed summary.
    return {"split": split, "n": len(ids), "sigma": tr["sigma"], "lambda2": float(tr["eigenvalues"][1]),
            "t": float(t_star), "A": len(A), "B": len(B), "T_hat": (wA, wB)}


# ---------------------------------------------------------------------------
# conflict_game.py: one game between two sinks
# ---------------------------------------------------------------------------

PLAYER = (COLORS[0], COLORS[1])                     # the two players, in the order of the game record
CONFLICT = "#d03b3b"                                # closer than d_safe; always labelled


# Chooses which two-player game to show by default.
def pick_pair_game(games):
    """The two-player game to show by default: one whose equilibrium uses DETOUR, else the one with the most
    strategies, else the first."""
    # The games with exactly two players.
    two = [gi for gi, g in enumerate(games) if len(g["players"]) == 2]
    # None at all?
    if not two:
        # Then there is nothing to show.
        return None
    # Prefer one ending in DETOUR, then the most strategies, then the earliest.
    return max(two, key=lambda gi: ("DETOUR" in games[gi]["equilibrium"].values(),
                                    sum(len(o) for o in games[gi]["options"].values()), -gi))


# The game picture, for game number gi. replay: the games, if already replayed.
def game_figure(out, gi, path, replay=None):
    """One two-player game of conflict_game.resolve, round 0 on seed 1, as the planner played it: the conflict,
    each player's strategies, the cost table with its equilibrium, and the distance before and after."""
    # This project's animations (for replaying the games).
    import animations
    # This project's conflict game.
    import conflict_game as cg
    # The settings.
    p = out["params"]
    # Replay the games (unless already done).
    games, trace = animations.game_replay_data(out) if replay is None else replay
    # The game to show.
    gi = pick_pair_game(games) if gi is None else gi
    # Its record and its recording.
    g, tr = games[gi], trace[gi]
    # Not a two-player game?
    if len(g["players"]) != 2:
        # Then stop with this message.
        raise SystemExit(f"game {gi} has {len(g['players'])} players; this figure needs a two-player game")
    # The two players.
    ka, kb = g["players"]
    # Their strategies, everyone's trajectories before, the decision frame,
    # and the conflict frame.
    acts, before, s, f = tr["acts"], tr["trajs"], g["decision_frame"], tr["f"]
    # The time step, and the price of danger.
    dt, alpha = p["dt"], g["alpha"]
    # The three cost weights.
    beta, gamma, delta = p["weights"]
    # The robots not playing.
    others = [m for m in before if m not in (ka, kb)]

    # Recompute every cell of the cost table from its parts, and check it against the game record.
    def parts(k, a, other_traj):
        # (This recipe, started above, splits a cost into its parts.) The danger
        # between the two players ...
        pair = cg.exposure(a["traj"], other_traj, s, p)
        # ... the danger with everyone else ...
        rest = sum(cg.exposure(a["traj"], before[m], s, p) for m in others)
        # ... and the player's own cost.
        own = beta * a["dT"] + gamma * a["dD"] / p["v"] + delta * a["W"]
        # Hand back the three parts.
        return pair, rest, own
    # The cost table, built below.
    cells = {}
    # For every strategy x of the first player ...
    for x, a in enumerate(acts[ka]):
        # ... and every strategy y of the second:
        for y, b in enumerate(acts[kb]):
            # the first player's parts;
            pa, ra, oa = parts(ka, a, b["traj"])
            # the second player's parts;
            pb, rb, ob = parts(kb, b, a["traj"])
            # both costs, and all the parts;
            cells[x, y] = (alpha * (pa + ra) + oa, alpha * (pb + rb) + ob, pa, ra, rb, oa, ob)
            # which must equal the game's own table (else stop with an error).
            assert np.allclose(cells[x, y][:2], g["payoff"][x][y]), "cost table differs from the game record"
    # The row and column of the equilibrium.
    eq = (g["options"][ka].index(g["equilibrium"][ka]), g["options"][kb].index(g["equilibrium"][kb]))

    # The players' names.
    names = {ka: f"sink {ka}", kb: f"sink {kb}"}
    # Both trajectories under GO (the plan) ...
    go_a, go_b = acts[ka][0]["traj"], acts[kb][0]["traj"]
    # ... and at the equilibrium.
    eq_a, eq_b = acts[ka][eq[0]]["traj"], acts[kb][eq[1]]["traj"]
    # Their closest distance in every step, as planned.
    d_go, dock_go = cg.closest(go_a, go_b, p)
    # The steps in conflict.
    bad = np.flatnonzero((d_go < p["d_safe"]) & ~dock_go)
    # The last of them.
    f_end = int(bad[bad >= f].max()) if len(bad) else f
    # The frames to show: from a little before the decision to 15 s after the conflict.
    lo_f, hi_f = max(0, s - 8), min(max(len(go_a), len(go_b)), f_end + 30)
    # This conflict's steps.
    run_ = bad[(bad >= f) & (bad <= f_end)]
    n_c = int(run_[np.argmin(d_go[run_])])                          # the closest moment of the conflict
    # (The closest step n_c is on the line above.) The arrow between the two at its start ...
    r0 = go_a[n_c] - go_b[n_c]
    # ... and how it changes during the step.
    dr = (go_a[n_c + 1] - go_b[n_c + 1]) - r0
    tau_c = float(np.clip(-r0 @ dr / max(dr @ dr, 1e-12), 0.0, 1.0))  # between two frames, as closest() does
    # (The fraction tau_c is on the line above.) Where each player is at the closest moment.
    at = {k: P[n_c] + tau_c * (P[n_c + 1] - P[n_c]) for k, P in ((ka, go_a), (kb, go_b))}
    # The time of that moment.
    t_c = (n_c + tau_c) * dt
    # Every strategy's trajectory over the frames shown ...
    window = [t["traj"][lo_f:hi_f] for k in (ka, kb) for t in acts[k]]
    # ... give the area to show.
    lims = _zoom(np.vstack(window), pad=7, aspect=6.0 / 5.0)

    # The page height in inches.
    H = 16.6
    # A 14 by 16.6 inch page.
    fig = plt.figure(figsize=(14, H), dpi=150, facecolor=SURFACE)
    # The title.
    fig.text(0.4 / 14, 1 - 0.3 / H, f"conflict_game.py: one game between two sinks (game {gi + 1} of {len(games)} in "
             f"round 0, seed {out.get('seed', 1)}: sinks {ka} and {kb} at t = {g['time']:.1f} s)", fontsize=14,
             fontweight="bold", color=INK, va="top")
    # The subtitle, explaining the game.
    fig.text(0.4 / 14, 1 - 0.62 / H, textwrap.fill(
        f"Two sinks are in conflict when they come closer than d_safe = {p['d_safe']:g} m (eq. 20). The sinks in "
        f"the first conflict become the players. Each chooses GO, WAIT or DETOUR (Section 0.10.1); its cost is "
        f"J = α·(seconds closer than d_safe to any sink) + β·extra time + γ·extra distance / v + δ·waiting time, "
        f"with α = {alpha:g} here, so that any danger outweighs any delay. Players switch to their best answer one "
        "at a time until neither can lower its cost: a Nash equilibrium.", 165), fontsize=9.3, color=QUIET,
        va="top", linespacing=1.4)
    # Four panels: (a) top left ...
    axa = _axes(fig, 0.55, 2.35, 6.0, 5.0)
    # ... (b) top right ...
    axb = _axes(fig, 7.6, 2.35, 6.0, 5.0)
    # ... (c) bottom left ...
    axc = _axes(fig, 0.55, 9.55, 6.0, 5.6)
    # ... (d) bottom right.
    axd = _axes(fig, 7.9, 9.55, 5.7, 5.0)

    # Draws one trajectory P over the frames shown, with small arrowheads
    # showing the direction of travel.
    def draw(ax, k, P, col, lw=2.0, ls="-", label=None, z=4):
        # The frames shown.
        seg = P[lo_f:hi_f]
        # The line.
        ax.plot(*seg.T, color=col, lw=lw, ls=ls, zorder=z, label=label)
        for q in range(4, len(seg) - 1, 8):                                # direction of travel
            # (Every 8th frame, from the line above:) an arrowhead with no tail.
            ax.annotate("", xy=seg[q + 1], xytext=seg[q], zorder=z, arrowprops=dict(
                arrowstyle="-|>", color=col, lw=0, mutation_scale=11, shrinkA=0, shrinkB=0))

    # (a) the conflict as planned
    _map_panel(axa, out["scenario"], p, lims, "(a) The conflict, as planned (both GO)",
               f"Both sinks drive their planned paths (arrows: direction). At t = {t_c:.2f} s they are only "
               f"{d_go[n_c]:.2f} m apart (between two frames), closer than d_safe = {p['d_safe']:g} m, for "
               f"{dt * len(bad[(bad >= f) & (bad <= f_end)]):.1f} s. The game decides at t = {s * dt:.1f} s, "
               "a little earlier, so that a sink still has room to react.")
    # Panel (a), set up above. Each player, as planned:
    for k, P, col in ((ka, go_a, PLAYER[0]), (kb, go_b, PLAYER[1])):
        # its path;
        draw(axa, k, P, col, label=names[k])
        # a dot where it is at the closest moment;
        axa.scatter(*at[k], s=60, color=col, edgecolor=SURFACE, lw=1.2, zorder=7)
        # a hollow square where it is when the game decides.
        axa.scatter(*P[min(s, len(P) - 1)], s=36, marker="s", color=SURFACE, edgecolor=col, lw=1.6, zorder=7)
    # The two positions at the closest moment.
    pa_, pb_ = at[ka], at[kb]
    # 100 angles round a full turn ...
    t_ = np.linspace(0, 2 * np.pi, 100)
    # ... make a red dashed circle of radius d_safe (3 m) round the first player.
    axa.plot(pa_[0] + p["d_safe"] * np.cos(t_), pa_[1] + p["d_safe"] * np.sin(t_), color=CONFLICT, lw=1.0,
             ls=DASHES, zorder=6)
    # A red line between the two ...
    axa.plot(*np.stack([pa_, pb_]).T, color=CONFLICT, lw=2.0, zorder=8)
    # ... labelled with their distance.
    _label(axa, (pa_ + pb_) / 2, f"conflict: {np.hypot(*(pa_ - pb_)):.2f} m < {p['d_safe']:g} m", (0, 18),
           color=CONFLICT, weight="bold")
    # The legend.
    _legend(axa, [Line2D([], [], color=PLAYER[0], lw=2, label=names[ka]),
                  Line2D([], [], color=PLAYER[1], lw=2, label=names[kb]),
                  Line2D([], [], marker="o", ms=6, color=MUTED, ls="none", label=f"at t = {t_c:.2f} s (closest)"),
                  Line2D([], [], marker="s", ms=5, mfc=SURFACE, mec=MUTED, ls="none",
                         label=f"at t = {s * dt:.1f} s (decision)"),
                  Line2D([], [], color=CONFLICT, lw=1.0, ls=DASHES, label=f"d_safe = {p['d_safe']:g} m")], ncol=3)

    # (b) strategies
    _map_panel(axb, out["scenario"], p, lims, "(b) Each player's strategies",
               "GO keeps the plan. WAIT stops at the decision point for the shortest time that clears the "
               f"conflict (at most {p['max_wait']:g} s, and only if T stays ≤ B). DETOUR steps "
               f"{p['d_safe'] + p['detour_margin']:g} m to the right of the own path and rejoins it, if the turn is "
               "feasible and obstacles stay r_safe away. Strategies a player does not have are missing.")
    # Panel (b), set up above. Each player ...
    for k, col in ((ka, PLAYER[0]), (kb, PLAYER[1])):
        # ... and each of its strategies:
        for a in acts[k]:
            # GO:
            if a["name"] == "GO":
                # its planned path.
                draw(axb, k, a["traj"], col, lw=1.6)
            # DETOUR:
            elif a["name"] == "DETOUR":
                # its side-step path, dashed;
                draw(axb, k, a["traj"], col, lw=2.2, ls=(0, (3, 2)), z=5)
                # comparing it with the plan, frame by frame ...
                m_ = min(len(a["traj"]), len(before[k]))
                # ... the frame where it is furthest aside;
                far = int(np.argmax(np.hypot(*(a["traj"][:m_] - before[k][:m_]).T)))
                # a label there: extra distance and extra time.
                _label(axb, a["traj"][far], f"{names[k]} DETOUR: +{a['dD']:.1f} m, +{a['dT']:.1f} s", (75, 6),
                       color=INK)
            # WAIT:
            else:
                # where it would stop (its position at the decision frame);
                q = before[k][min(s, len(before[k]) - 1)]
                # a diamond there ...
                axb.scatter(*q, s=110, marker="D", color=col, edgecolor=SURFACE, lw=1.2, zorder=7)
                # ... labelled with how long.
                _label(axb, q, f"{names[k]} WAIT {a['W']:.1f} s here", (0, 14), color=INK)
    # The legend.
    _legend(axb, [Line2D([], [], color=MUTED, lw=1.6, label="GO (the plan)"),
                  Line2D([], [], marker="D", ms=7, color=MUTED, ls="none", label="WAIT: stop here"),
                  Line2D([], [], color=MUTED, lw=2.2, ls=(0, (3, 2)), label="DETOUR")], ncol=3)

    # (c) the cost table
    axc.set_xlim(0, 1)
    # Panel (c): positions from 0 to 1 across (line above) and up ...
    axc.set_ylim(0, 1)
    # ... with no axes.
    axc.axis("off")
    # Its heading.
    _heading(axc, "(c) The cost table, and the equilibrium",
             f"Each cell: the cost J of {names[ka]} (left) and of {names[kb]} (right) if they play that row and "
             f"column, from conflict_game.play. Below: seconds of conflict between the two and with other sinks, "
             f"and each one's own cost (extra time, distance, waiting). The game starts at GO/GO; each switch is one player's best "
             "answer; it stops where neither can do better (outlined).", lines=4)
    # The rows (first player's strategies) and columns (second player's).
    ra, rb = g["options"][ka], g["options"][kb]
    # The table's top-left corner and each cell's width and height.
    x0, y0, cw, ch = 0.2, 0.86, 0.8 / len(rb), 0.62 / len(ra)
    # Above each column ...
    for y, nm in enumerate(rb):
        # ... the second player and that strategy.
        axc.text(x0 + (y + 0.5) * cw, y0 + 0.03, f"{names[kb]}\n{nm}", ha="center", va="bottom", fontsize=9,
                 fontweight="bold", color=PLAYER[1])
    # For each row:
    for x, nm in enumerate(ra):
        # the first player and that strategy, on the left;
        axc.text(x0 - 0.02, y0 - (x + 0.5) * ch, f"{names[ka]}\n{nm}", ha="right", va="center", fontsize=9,
                 fontweight="bold", color=PLAYER[0])
        # for each cell in the row:
        for y in range(len(rb)):
            # both costs and all their parts;
            Ja, Jb, pair, rest_a, rest_b, oa, ob = cells[x, y]
            # is it the equilibrium?
            is_eq = (x, y) == eq
            # the cell: tinted if the two still conflict, outlined if it is the equilibrium;
            axc.add_patch(Rectangle((x0 + y * cw + 0.004, y0 - (x + 1) * ch + 0.004), cw - 0.008, ch - 0.008,
                                    fc=TINT if pair > 0 else BOX, ec=INK if is_eq else "none", lw=2.2 if is_eq else 0))
            # its middle;
            cx, cy = x0 + (y + 0.5) * cw, y0 - (x + 0.5) * ch
            # the first player's cost, left of the middle ...
            axc.text(cx - 0.012, cy + 0.045, f"{Ja:.1f}", ha="right", va="center", fontsize=12, fontweight="bold",
                     color=INK)
            # ... a comma ...
            axc.text(cx, cy + 0.045, ",", ha="center", va="center", fontsize=12, color=MUTED)
            # ... the second player's cost, right of it;
            axc.text(cx + 0.012, cy + 0.045, f"{Jb:.1f}", ha="left", va="center", fontsize=12, fontweight="bold",
                     color=INK)
            # small lines under them: the conflict between the two ...
            lines_ = [f"between them: {pair:.1f} s"]
            # ... the conflict with others, if any ...
            if rest_a + rest_b:
                # (both players' shares)
                lines_.append(f"with others: {rest_a:.1f} | {rest_b:.1f} s")
            # ... and both own costs;
            lines_.append(f"own: {oa:.1f} | {ob:.1f}")
            # each small line ...
            for n_l, l_ in enumerate(lines_):
                # ... written one under another;
                axc.text(cx, cy - 0.03 - 0.032 * n_l, l_, ha="center", va="center", fontsize=7.6, color=QUIET)
            # and in the equilibrium's cell ...
            if is_eq:
                # ... the word "equilibrium" at the bottom.
                axc.text(cx, y0 - (x + 1) * ch + 0.015, "equilibrium", ha="center", va="bottom", fontsize=8,
                         fontweight="bold", color=INK)
    # The switches, in words: GO/GO → sink ... switches to ... (gains ...).
    moves = " → ".join(["GO/GO"] + [f"{names[k]} switches to {nm} (gains {gain:.1f})" for k, nm, gain in g["moves"]])
    # Written under the table.
    axc.text(0.0, y0 - len(ra) * ch - 0.05, textwrap.fill("Best-response switches: " + moves, 95), fontsize=8.6,
             color=INK, va="top", linespacing=1.4)
    # A note explaining the shading and the costs.
    axc.text(0.0, y0 - len(ra) * ch - 0.13, textwrap.fill(
        f"Shaded: the two still come closer than d_safe. Costs are in seconds: J = α·risk + own, α = {alpha:g}; "
        f"own = {beta:g}·extra time + {gamma:g}·extra distance / v + {delta:g}·waiting.", 95), fontsize=8.3,
        color=QUIET, va="top", linespacing=1.4)

    # (d) distance before and after
    _quiet(axd)
    # Panel (d) (made plain on the line above): its heading.
    _heading(axd, "(d) Distance between the two sinks, before and after",
             f"The closest distance in every {dt:g} s step (conflict_game.closest), as planned and at the "
             f"equilibrium ({g['equilibrium'][ka]} / {g['equilibrium'][kb]}). Below the dashed line the two are in "
             "conflict.", lines=4)
    # Their closest distance in every step at the equilibrium.
    d_eq, dock_eq = cg.closest(eq_a, eq_b, p)
    # As planned (grey) and at the equilibrium (black):
    for d, dock, col, lw, lab in ((d_go, dock_go, MUTED, 1.6, "as planned (GO / GO)"),
                                  (d_eq, dock_eq, INK, 2.2, f"equilibrium ({g['equilibrium'][ka]} / "
                                                            f"{g['equilibrium'][kb]})")):
        # the step numbers;
        n = np.arange(len(d))
        # the steps shown;
        sel = (n >= lo_f) & (n < hi_f)
        # distance against time.
        axd.plot(n[sel] * dt, d[sel], color=col, lw=lw, label=lab)
    # The planned steps in conflict ...
    sel = (np.arange(len(d_go)) >= lo_f) & (np.arange(len(d_go)) < hi_f) & (d_go < p["d_safe"]) & ~dock_go
    # ... as red dots.
    axd.scatter(np.flatnonzero(sel) * dt, d_go[sel], s=26, color=CONFLICT, zorder=5,
                label=f"conflict: closer than {p['d_safe']:g} m")
    # A red dashed line at d_safe.
    axd.axhline(p["d_safe"], color=CONFLICT, lw=1.0, ls=DASHES)
    # A dotted line at the decision time ...
    axd.axvline(s * dt, color=MUTED, lw=0.8, ls=":")
    # ... labelled.
    axd.text(s * dt, 0, " decision", fontsize=8, color=MUTED, va="bottom")
    # Distances from 0 up (the top is chosen automatically).
    axd.set_ylim(0, None)
    # Labels along the bottom ...
    axd.set_xlabel("time t [s]", fontsize=8.5, color=MUTED)
    # ... and up the side.
    axd.set_ylabel("distance between the sinks [m]", fontsize=8.5, color=MUTED)
    # The legend.
    axd.legend(loc="center right", frameon=False, fontsize=8.5, labelcolor=INK)
    # Save the picture.
    fig.savefig(path, facecolor=SURFACE)
    # Close the page.
    plt.close(fig)
    # Hand back the numbers, for the printed summary.
    return {"game": gi, "players": (ka, kb), "equilibrium": g["equilibrium"], "payoff": g["payoff"],
            "closest_before": float(d_go[lo_f:hi_f].min()), "closest_after": float(d_eq[lo_f:hi_f].min())}


# What happens when the file is started from the command line.
def main():
    # Describe the options the user may type:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --seed: which map;
    ap.add_argument("--seed", type=int, default=1)
    # --only: which pictures;
    ap.add_argument("--only", nargs="+", choices=KINDS, default=list(KINDS), help="which figures to make")
    # --sink: the robot for the trajectory picture;
    ap.add_argument("--sink", type=int, default=None,
                    help="sink for the trajectory figure (default: the one whose path is longest against its straight lines)")
    # --ga-sink: the robot for the GA picture;
    ap.add_argument("--ga-sink", type=int, default=None, help="sink for the GA figure (default: the one with nearest to 8 sensors)")
    # --split: which split to show;
    ap.add_argument("--split", type=int, default=1, help="split for the bisection figure, counted from 1 (1: all sensors)")
    # --game: which two-player game to show.
    ap.add_argument("--game", type=int, default=None,
                    help="two-player game for the game figure, counted from 1 (default: one that ends in DETOUR)")
    # Read what the user typed.
    args = ap.parse_args()
    # The normal settings.
    p = default_params()
    # The "output" folder next to this file ...
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    # ... made if it is missing.
    os.makedirs(folder, exist_ok=True)
    # The start of every file name, e.g. "seed1".
    name = f"seed{args.seed}"
    # No plan yet.
    out = log = None
    # Four of the pictures need a planned map. If any of them was asked for:
    if set(args.only) & {"ga", "trajectory", "bisection", "game"}:
        # borrow animations.py (for choosing a robot and replaying games) ...
        import animations
        # ... and the planner;
        from planner import run
        # an empty log for the GA runs;
        log = []
        out = run(args.seed, p, ga_log=log)                 # the planner's own run, with every GA run recorded
        # (the plan is made on the line above) note the seed in it;
        out["seed"] = args.seed
        # and print a summary.
        c = out["checks"]
        # (robots, longest time, total length, GA runs)
        print(f"seed {args.seed}: K = {c['K']}, T_max = {c['T_max']:.1f} s, L_total = {c['L_total']:.0f} m, "
              f"{len(log)} GA runs")
    # Go through the six pictures in order:
    for kind in KINDS:
        # not asked for?
        if kind not in args.only:
            # Skip it.
            continue
        # The flow diagram:
        if kind == "flow":
            # its file name ...
            path = os.path.join(folder, "pipeline_flow.png")
            # ... and drawing it.
            flow_figure(p, path)
        # The geodesic picture:
        elif kind == "geodesic":
            # its file name;
            path = os.path.join(folder, f"{name}_geodesic_graph.png")
            # draw it;
            s = geodesic_figure(args.seed, p, path)
            # print its numbers.
            print(f"geodesic: {s['terminals']} terminals + {s['corners']} corners, {s['edges']} edges "
                  f"({s['terminal_edges']} at a terminal); S{s['X']} has {s['X_edges']} edges, r = {s['r_X']:.1f} m "
                  f"against {s['straight_X']:.1f} m straight; table: S{', S'.join(map(str, s['picks']))}")
        # The GA picture:
        elif kind == "ga":
            # the robots' tours;
            tours = out["result"]["tours"]
            # the robot with nearest to 8 sensors, unless one was chosen ("\" continues the line);
            k = min(range(len(tours)), key=lambda q: (abs(len(tours[q]["order"]) - 8), q)) \
                if args.ga_sink is None else args.ga_sink
            # its file name;
            path = os.path.join(folder, f"{name}_sink{k}_ga_moves.png")
            # draw it;
            s = ga_figure(out, log, k, path)
            # print its numbers.
            print(f"ga: sink {k}, {s['n']} sensors, cut {s['cut']}, swap {s['swap']}; best fitness "
                  f"{s['best0']:.1f} m at generation 0, {s['best_end']:.1f} m at the end")
        # The trajectory picture:
        elif kind == "trajectory":
            # the robot (the default, or the one chosen);
            k = animations.pick_refine_sink(out) if args.sink is None else args.sink
            # its file name;
            path = os.path.join(folder, f"{name}_sink{k}_trajectory.png")
            # draw it;
            s = trajectory_figure(out, k, path)
            # print its numbers.
            print(f"trajectory: sink {k}, {s['waypoints']} waypoints, legs {' '.join(s['legs'])}, "
                  f"L = {s['L']:.1f} m (bisector headings: {s['L_bisector']:.1f} m), T = {s['T']:.1f} s, "
                  f"{s['frames']} frames")
        # The bisection picture:
        elif kind == "bisection":
            # its file name;
            path = os.path.join(folder, f"{name}_split{args.split}_bisection.png")
            # draw it (the code counts splits from 0, the user from 1);
            s = bisection_figure(out, log, args.split - 1, path)
            # print its numbers.
            print(f"bisection: split {args.split}, {s['n']} sensors -> {s['A']} + {s['B']}, sigma = {s['sigma']:.1f} m, "
                  f"lambda2 = {s['lambda2']:.4f}, t = {s['t']:.4f}, T_hat = {s['T_hat'][0]:.0f} s and {s['T_hat'][1]:.0f} s")
        # The game picture:
        else:
            replay = animations.game_replay_data(out)              # round 0 of the game, as the planner played it
            # (the games are replayed on the line above) the game to show;
            gi = pick_pair_game(replay[0]) if args.game is None else args.game - 1
            # its file name;
            path = os.path.join(folder, f"{name}_game{gi + 1}_pair.png")
            # draw it;
            s = game_figure(out, gi, path, replay)
            # print its numbers.
            print(f"game: game {s['game'] + 1}, sinks {s['players']}, equilibrium {s['equilibrium']}, closest "
                  f"{s['closest_before']:.2f} m as planned, {s['closest_after']:.2f} m after")
        # Say where the picture was saved.
        print(f"figure: {path}")


# True only when this file is started directly, not when it is imported.
if __name__ == "__main__":
    # Run the main recipe.
    main()
