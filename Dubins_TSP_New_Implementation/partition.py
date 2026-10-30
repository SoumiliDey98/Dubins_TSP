"""Curvature-aware geodesic spectral bisection (Section 0.9).

Sensors are indexed 0..n-1; Ds is the sensor-to-sensor geodesic distance matrix
and r the geodesic distance of each sensor from the base.
"""
import numpy as np


def workload(ids, xy, base, Ds, r, tau, v):
    """Estimated time T_hat of a set of sensors: a sweep tour in polar-angle order
    around the base (starting after the widest angular gap), with geodesic legs,
    divided by v, plus all dwell times."""
    ang = np.arctan2(xy[ids, 1] - base[1], xy[ids, 0] - base[0])
    k = np.argsort(ang)
    gaps = np.diff(np.append(ang[k], ang[k[0]] + 2 * np.pi))
    seq = ids[np.roll(k, -(int(np.argmax(gaps)) + 1))]
    length = r[seq[0]] + Ds[seq[:-1], seq[1:]].sum() + r[seq[-1]]
    return length / v + tau[seq].sum()


def bisect(ids, xy, base, Ds, r, tau, p):
    """Split `ids` into two groups (steps 1-7 of Section 0.9) and return (A, B)."""
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
    _, vecs = np.linalg.eigh(L)                           # step 5: eigenvalues in ascending order
    f = vecs[:, 1]                                        # step 6: Fiedler vector
    best = None
    for t in np.unique(f)[:-1]:                           # step 7: threshold with min max workload
        A, B = ids[f <= t], ids[f > t]
        w = max(workload(A, xy, base, Ds, r, tau, p["v"]), workload(B, xy, base, Ds, r, tau, p["v"]))
        if best is None or w < best[0]:
            best = (w, A, B)
    return best[1], best[2]
