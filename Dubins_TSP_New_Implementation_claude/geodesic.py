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
import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from shapely.geometry import Polygon


def inflate(obstacles, r):
    """Obstacles grown by r (rounded corners approximated by short edges)."""
    return [Polygon(o).buffer(r, quad_segs=2) for o in obstacles]


def _inside(polys):
    """Union of the polygons' insides (boundaries excluded), ready for fast tests."""
    u = shapely.union_all([g.buffer(-1e-6) for g in polys])
    shapely.prepare(u)
    return u


def visibility_graph(scen, p):
    """The graph that build() runs Dijkstra on. Returns (nodes, i, j, length): nodes are the
    terminals (base, then sensors) followed by the corners of the obstacles inflated by r_plan;
    edge k joins nodes i[k] < j[k], which see each other, and has the straight-line length length[k]."""
    terminals = np.vstack([scen["base"], scen["xy"]])
    n_t = len(terminals)
    grown = inflate(scen["obstacles"], p["r_plan"])
    nodes = np.vstack([terminals] + [np.array(g.exterior.coords[:-1]) for g in grown])

    # Two nodes see each other if the straight line between them does not enter
    # the inside of an inflated obstacle (running along its boundary is allowed).
    i, j = np.triu_indices(len(nodes), 1)
    lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
    visible = np.ones(len(i), dtype=bool)
    if grown:
        term = (i < n_t) | (j < n_t)
        visible[term] = ~shapely.intersects(lines[term], _inside(inflate(scen["obstacles"], p["r_safe"] + 0.5)))
        visible[~term] = ~shapely.intersects(lines[~term], _inside(grown))
    i, j = i[visible], j[visible]
    return nodes, i, j, np.hypot(*(nodes[i] - nodes[j]).T)


def build(scen, p, require_all=True):
    """Return {"nodes", "D", "pred", "n_terminals", "obstacles"}: D[a, b] is the geodesic
    distance between terminals a and b; pred is used by path(); "obstacles" is the union
    of the original (not inflated) obstacles, used for clearance checks."""
    n_t = len(scen["xy"]) + 1
    nodes, i, j, length = visibility_graph(scen, p)
    graph = csr_matrix((length, (i, j)), shape=(len(nodes), len(nodes)))

    dist, pred = dijkstra(graph, directed=False, indices=np.arange(n_t), return_predecessors=True)
    D = dist[:, :n_t]
    if require_all and not np.all(np.isfinite(D)):
        raise ValueError("some sensors cannot be reached from the base station")
    obstacles = shapely.union_all([Polygon(o) for o in scen["obstacles"]]) if scen["obstacles"] else None
    if obstacles is not None:
        shapely.prepare(obstacles)
    return {"nodes": nodes, "D": D, "pred": pred, "n_terminals": n_t, "obstacles": obstacles}


def path(geo, a, b):
    """Obstacle-free polyline (corner to corner) from terminal a to terminal b."""
    pred = geo["pred"][a]
    seq = [b]
    while seq[-1] != a:
        seq.append(pred[seq[-1]])
    return geo["nodes"][seq[::-1]]
