"""Curvature-aware geodesic spectral bisection (Section 0.9).

Sensors are indexed 0..n-1; Ds is the sensor-to-sensor geodesic distance matrix
and r the geodesic distance of each sensor from the base.
"""
import numpy as np


def sweep_order(ids, xy, base):
    """The sensors `ids` in polar-angle order around the base, starting after the widest angular gap."""
    ang = np.arctan2(xy[ids, 1] - base[1], xy[ids, 0] - base[0])
    k = np.argsort(ang)
    gaps = np.diff(np.append(ang[k], ang[k[0]] + 2 * np.pi))
    return ids[np.roll(k, -(int(np.argmax(gaps)) + 1))]


def workload(ids, xy, base, Ds, r, tau, v):
    """Estimated time T_hat of a set of sensors: a sweep tour in polar-angle order
    around the base (sweep_order), with geodesic legs, divided by v, plus all dwell times."""
    seq = sweep_order(ids, xy, base)
    length = r[seq[0]] + Ds[seq[:-1], seq[1:]].sum() + r[seq[-1]]
    return length / v + tau[seq].sum()


def bisect(ids, xy, base, Ds, r, tau, p, trace=None):
    """Split `ids` into two groups (steps 1-7 of Section 0.9) and return (A, B).

    If trace is a dict, it is filled for the figures: "sigma", "W" (affinity), "eigenvalues",
    "f" (the Fiedler vector, one value per sensor of ids), "thresholds" and "workloads"
    (T_hat of both sides for every threshold tried) and "best" (index of the chosen threshold).
    It changes nothing else."""
    ids = np.asarray(ids)
    n = len(ids)
    sub = Ds[np.ix_(ids, ids)]
    # Kernel width: the mean nearest-neighbour geodesic spacing, but not below R
    # (gaps smaller than the turning radius do not shorten a Dubins tour).
    nn = np.where(np.eye(n, dtype=bool), np.inf, sub).min(axis=1).mean()
    sigma = max(nn, p["R"])
    W = np.exp(-(sub / sigma) ** 2)                       # step 2: affinity
    np.fill_diagonal(W, 0.0)
    deg = np.maximum(W.sum(axis=1), 1e-300)               # step 3: degrees
    s = 1.0 / np.sqrt(deg)
    L = np.eye(n) - s[:, None] * W * s[None, :]           # step 4: normalised Laplacian
    vals, vecs = np.linalg.eigh(L)                        # step 5: eigenvalues in ascending order
    f = vecs[:, 1]                                        # step 6: Fiedler vector
    best, tried = None, []
    for t in np.unique(f)[:-1]:                           # step 7: threshold with min max workload
        A, B = ids[f <= t], ids[f > t]
        wA, wB = workload(A, xy, base, Ds, r, tau, p["v"]), workload(B, xy, base, Ds, r, tau, p["v"])
        tried.append((t, wA, wB))
        if best is None or max(wA, wB) < best[0]:
            best = (max(wA, wB), A, B, len(tried) - 1)
    if trace is not None:
        trace.update(sigma=sigma, W=W, eigenvalues=vals, f=f, thresholds=np.array([x[0] for x in tried]),
                     workloads=np.array([x[1:] for x in tried]), best=best[3])
    return best[1], best[2]
