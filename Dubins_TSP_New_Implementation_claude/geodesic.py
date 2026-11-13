# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a short
# description written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# A robot cannot drive through walls, so the distance between two places is
# not the straight-line ("as the crow flies") distance but the length of the
# shortest way round the obstacles. That is the "geodesic" distance (equation 9
# of the paper). This file works it out for every pair of places in three steps:
#   1. Make every obstacle fatter, so a path that only touches the fat outline
#      still leaves the robot room to turn.
#   2. Join every two important points that can "see" each other with a
#      straight line that does not cut through a fat obstacle. The points and
#      the lines together are called a "visibility graph".
#   3. Find the shortest chain of such lines between every two places, with
#      Dijkstra's algorithm (a famous method for shortest routes, like a sat-nav).
# The places are called "terminals": terminal 0 is the base station, and
# terminal i + 1 is sensor i (sensors are counted from 0).
# ==============================================================================
"""Obstacle-aware (geodesic) distances and paths: eq. (9) and Section 0.9, step 1.

Obstacles are inflated by r_plan = r_safe + R, which leaves the sink room to
turn. The corners of the inflated obstacles, the base station and the sensors
are joined whenever they can see each other (a visibility graph), and Dijkstra
gives the shortest obstacle-free distance and path between every pair of
terminals. Terminal 0 is the base station; terminal i + 1 is sensor i.

Sensors may lie closer to an obstacle than r_plan (for example in the pocket of
a C-shaped obstacle). A line that starts or ends at a terminal therefore only
has to stay outside the obstacles inflated by r_safe + 0.5 m; lines between two
corners keep the full turning room r_plan.
"""
# "import" borrows a toolbox. numpy (nicknamed np) works with long lists of
# numbers ("arrays") very quickly.
import numpy as np
# shapely is a toolbox for flat shapes: points, lines and polygons.
import shapely
# scipy is a toolbox of scientific methods. csr_matrix stores a big table that
# is mostly empty without wasting memory (a "sparse matrix").
from scipy.sparse import csr_matrix
# dijkstra is the shortest-route method, ready-made in scipy.
from scipy.sparse.csgraph import dijkstra
# Polygon: a closed shape with straight sides, like a floor plan.
from shapely.geometry import Polygon


# "def" starts a recipe (a "function"). This one makes obstacles fatter.
# Its inputs: the obstacles (each a list of corners) and r, how much fatter.
def inflate(obstacles, r):
    """Obstacles grown by r (rounded corners approximated by short edges)."""
    # For every obstacle o: make it a Polygon and "buffer" it, which grows it by
    # r metres on every side. At a corner the grown outline would be a round
    # arc; quad_segs=2 draws a quarter circle with 2 straight pieces instead.
    # The square brackets with "for" make a list with one fat polygon per
    # obstacle, and "return" hands that list back.
    return [Polygon(o).buffer(r, quad_segs=2) for o in obstacles]


# The "_" at the start of the name means "only used inside this file".
# This recipe makes one shape that is the inside of all the given polygons.
def _inside(polys):
    """Union of the polygons' insides (boundaries excluded), ready for fast tests."""
    # Shrink every polygon by a tiny 0.000001 m (a negative buffer shrinks),
    # then glue them all into one shape. Shrinking a hair means that a line
    # that only slides along an outline does NOT count as going inside.
    u = shapely.union_all([g.buffer(-1e-6) for g in polys])
    # "prepare" builds a lookup aid inside the shape, so that thousands of
    # "does this line touch it?" questions are answered much faster.
    shapely.prepare(u)
    # Hand the shape back.
    return u


# Step 2 of the method: build the visibility graph.
# Inputs: scen, the map (from scenario.py), and p, the settings (from config.py).
def visibility_graph(scen, p):
    """The graph that build() runs Dijkstra on. Returns (nodes, i, j, length): nodes are the
    terminals (base, then sensors) followed by the corners of the obstacles inflated by r_plan;
    edge k joins nodes i[k] < j[k], which see each other, and has the straight-line length length[k]."""
    # Stack the base on top of the sensors: row 0 is the base, row i + 1 is
    # sensor i. These are the terminals.
    terminals = np.vstack([scen["base"], scen["xy"]])
    # n_t: how many terminals there are (1 base + all sensors).
    n_t = len(terminals)
    # Every obstacle made fatter by r_plan = r_safe + R = 2 + 5 = 7 m.
    grown = inflate(scen["obstacles"], p["r_plan"])
    # The "nodes" (dots) of the graph: first the terminals, then every corner of
    # every fat obstacle. A shortest way round an obstacle only ever bends at
    # such corners, so these points are enough. coords[:-1] skips the last
    # outline point, which repeats the first.
    nodes = np.vstack([terminals] + [np.array(g.exterior.coords[:-1]) for g in grown])

    # Two nodes see each other if the straight line between them does not enter
    # the inside of an inflated obstacle (running along its boundary is allowed).
    # Every pair of nodes once: i and j list the pairs (i smaller than j).
    # np.triu_indices gives the positions above the diagonal of a square table,
    # which is exactly "every pair, once".
    i, j = np.triu_indices(len(nodes), 1)
    # One straight line segment for each pair, from node i to node j.
    lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
    # visible: one yes/no ("boolean", bool) per pair. Start with "yes" for all.
    visible = np.ones(len(i), dtype=bool)
    # Only if there are obstacles (an empty list counts as "no").
    if grown:
        # term: "yes" for pairs where at least one end is a terminal (its
        # number is below n_t). "|" means "or".
        term = (i < n_t) | (j < n_t)
        # A line that starts or ends at a sensor or the base only has to avoid
        # obstacles grown by r_safe + 0.5 = 2.5 m, because sensors may sit
        # closer than 7 m to an obstacle. "~" means "not": a line is visible if
        # it does NOT touch the inside of those 2.5 m shapes.
        visible[term] = ~shapely.intersects(lines[term], _inside(inflate(scen["obstacles"], p["r_safe"] + 0.5)))
        # A line between two corners must avoid the full 7 m shapes.
        visible[~term] = ~shapely.intersects(lines[~term], _inside(grown))
    # Keep only the visible pairs. Each one is an "edge" (a line) of the graph.
    i, j = i[visible], j[visible]
    # Hand back the nodes, the two ends of every edge, and every edge's length:
    # np.hypot gives the straight distance (Pythagoras: square root of
    # x-difference squared plus y-difference squared).
    return nodes, i, j, np.hypot(*(nodes[i] - nodes[j]).T)


# Step 3: shortest routes between every two terminals.
# require_all=True means: stop with an error if some sensor cannot be reached.
def build(scen, p, require_all=True):
    """Return {"nodes", "D", "pred", "n_terminals", "obstacles"}: D[a, b] is the geodesic
    distance between terminals a and b; pred is used by path(); "obstacles" is the union
    of the original (not inflated) obstacles, used for clearance checks."""
    # How many terminals: all the sensors plus 1 for the base.
    n_t = len(scen["xy"]) + 1
    # Build the visibility graph (the recipe above).
    nodes, i, j, length = visibility_graph(scen, p)
    # Store the graph as a big table: the entry in row i, column j is the length
    # of the edge from node i to node j. Pairs with no edge are left empty.
    graph = csr_matrix((length, (i, j)), shape=(len(nodes), len(nodes)))

    # Dijkstra's method, started from every terminal in turn
    # (indices = 0, 1, ..., n_t - 1). directed=False: every edge can be driven
    # both ways. From each start it settles the nodes one at a time, nearest
    # first, and for each settled node tries all its edges, keeping any route
    # to a neighbour that is shorter than the best known so far.
    # dist[a, v]: the shortest distance from terminal a to node v.
    # pred[a, v]: the node just before v on that shortest route ("predecessor").
    dist, pred = dijkstra(graph, directed=False, indices=np.arange(n_t), return_predecessors=True)
    # Keep only the distances between terminals (the first n_t columns); the
    # corners were only stepping stones. D[a, b] is the geodesic distance.
    D = dist[:, :n_t]
    # A distance of "infinity" means no route exists. If that happens and every
    # sensor must be reachable, "raise" stops the program with this message.
    if require_all and not np.all(np.isfinite(D)):
        # Stop the program with this error message.
        raise ValueError("some sensors cannot be reached from the base station")
    # The real (not fattened) obstacles glued into one shape, used later to
    # measure how close a path comes to them. If there are no obstacles, the
    # special value None ("nothing") is used. "x if question else y" picks x
    # when the answer is yes and y when it is no.
    obstacles = shapely.union_all([Polygon(o) for o in scen["obstacles"]]) if scen["obstacles"] else None
    # Speed up later distance questions, if there is a shape.
    if obstacles is not None:
        # Build its lookup aid, so later distance questions are fast.
        shapely.prepare(obstacles)
    # Hand back everything in one dictionary (a table of named values).
    return {"nodes": nodes, "D": D, "pred": pred, "n_terminals": n_t, "obstacles": obstacles}


# Rebuilds the shortest route from terminal a to terminal b as a list of
# points, corner by corner. "geo" is the dictionary that build() handed back.
def path(geo, a, b):
    """Obstacle-free polyline (corner to corner) from terminal a to terminal b."""
    # The "came from" list of Dijkstra's run that started at terminal a.
    pred = geo["pred"][a]
    # Start the route at the far end, b.
    seq = [b]
    # "while" repeats as long as its question is true: until we are back at a.
    # seq[-1] is the last item of the list.
    while seq[-1] != a:
        # Step back to the node we came from, and add it to the list.
        seq.append(pred[seq[-1]])
    # The list runs from b back to a; [::-1] turns it round so it runs from a
    # to b. Then look up each node's (x, y) position and hand the points back.
    return geo["nodes"][seq[::-1]]
