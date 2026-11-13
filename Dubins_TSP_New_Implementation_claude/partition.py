# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# When one robot has too many sensors to visit within the time budget B, its
# group of sensors is cut into two smaller groups, one for each of two robots.
# Where should the cut go? Sensors that are close together (going round the
# obstacles, not through them) should stay in the same group.
#
# The method is "spectral bisection" (Section 0.9 of the paper). Picture every
# pair of sensors tied together by a rubber band: a strong band if they are
# close, a weak one if they are far apart. Now give every sensor a number, so
# that sensors joined by strong bands get nearly the same number. The smoothest
# way to do that (apart from giving everyone the same number) is a list called
# the "Fiedler vector". Sort the sensors by that number and cut the list in two:
# the cut then falls where the bands are weakest. The exact cut point is chosen
# so that the two groups need about the same time.
# ==============================================================================
"""Curvature-aware geodesic spectral bisection (Section 0.9).

Sensors are indexed 0..n-1; Ds is the sensor-to-sensor geodesic distance matrix
and r the geodesic distance of each sensor from the base.
"""
# "import" borrows a toolbox. numpy (nicknamed np) works with whole lists
# ("arrays") and tables ("matrices") of numbers at once.
import numpy as np


# "def" starts a recipe (a "function"). This one puts a group of sensors in the
# order you would meet them sweeping round the base like the hand of a clock.
# Inputs: ids, the sensors' numbers; xy, all sensor positions; base, the base position.
def sweep_order(ids, xy, base):
    """The sensors `ids` in polar-angle order around the base, starting after the widest angular gap."""
    # The direction of each sensor as seen from the base (an angle between -pi
    # and pi radians). atan2(y difference, x difference) gives that angle.
    ang = np.arctan2(xy[ids, 1] - base[1], xy[ids, 0] - base[0])
    # k: the positions of the sensors sorted by angle (np.argsort gives the
    # order, not the sorted values).
    k = np.argsort(ang)
    # The gap in angle between each sensor and the next one round the circle.
    # The last gap wraps round from the last sensor back to the first (adding
    # 2*pi, one full turn).
    gaps = np.diff(np.append(ang[k], ang[k[0]] + 2 * np.pi))
    # Start the sweep just after the widest empty gap, so the sweep does not
    # jump across it. np.roll rotates the list to start there. Hand back the
    # sensors' numbers in that order.
    return ids[np.roll(k, -(int(np.argmax(gaps)) + 1))]


# A quick estimate, T_hat, of how long one robot would need for a group of
# sensors. It does not plan a real path; it is just a fair guess for comparing
# two groups. Inputs: Ds, the sensor-to-sensor distances round obstacles;
# r, each sensor's distance from the base; tau, the waiting times; v, the speed.
def workload(ids, xy, base, Ds, r, tau, v):
    """Estimated time T_hat of a set of sensors: a sweep tour in polar-angle order
    around the base (sweep_order), with geodesic legs, divided by v, plus all dwell times."""
    # Visit the sensors in clock-hand order.
    seq = sweep_order(ids, xy, base)
    # The length of that trip: from the base to the first sensor, then from each
    # sensor to the next (Ds[seq[:-1], seq[1:]] picks those distances; .sum()
    # adds them), then from the last sensor back to the base.
    length = r[seq[0]] + Ds[seq[:-1], seq[1:]].sum() + r[seq[-1]]
    # Time = length / speed, plus all the waiting times at the sensors.
    return length / v + tau[seq].sum()


# Cuts a group of sensors into two groups, A and B (steps 1-7 of Section 0.9).
# "trace" is an optional empty dictionary; if one is given, the in-between
# results are written into it, so the figures can show them.
def bisect(ids, xy, base, Ds, r, tau, p, trace=None):
    """Split `ids` into two groups (steps 1-7 of Section 0.9) and return (A, B).

    If trace is a dict, it is filled for the figures: "sigma", "W" (affinity), "eigenvalues",
    "f" (the Fiedler vector, one value per sensor of ids), "thresholds" and "workloads"
    (T_hat of both sides for every threshold tried) and "best" (index of the chosen threshold).
    It changes nothing else."""
    # Make sure ids is a numpy array.
    ids = np.asarray(ids)
    # n: how many sensors are in the group.
    n = len(ids)
    # Step 1: the distances between the sensors of this group only.
    # np.ix_(ids, ids) picks the rows AND the columns of those sensors.
    sub = Ds[np.ix_(ids, ids)]
    # Kernel width: the mean nearest-neighbour geodesic spacing, but not below R
    # (gaps smaller than the turning radius do not shorten a Dubins tour).
    # For each sensor, the distance to its nearest neighbour (np.eye marks the
    # diagonal, a sensor's distance to itself, which is replaced by infinity so
    # it is never the nearest), then the average of those.
    nn = np.where(np.eye(n, dtype=bool), np.inf, sub).min(axis=1).mean()
    # sigma, the "width" of the rubber bands: the larger of that average and R (5 m).
    sigma = max(nn, p["R"])
    # Step 2: the strength of the band between every two sensors, the
    # "affinity" W = exp(-(distance / sigma)^2). exp is the exponential
    # function: two sensors in the same place get 1, sensors sigma apart get
    # 0.37, and sensors far apart get almost 0.
    W = np.exp(-(sub / sigma) ** 2)                       # step 2: affinity
    # No sensor is tied to itself: put 0 on the diagonal.
    np.fill_diagonal(W, 0.0)
    # Step 3: each sensor's "degree", the total strength of all its bands.
    # (np.maximum stops it being exactly 0, which would break the next line.)
    deg = np.maximum(W.sum(axis=1), 1e-300)               # step 3: degrees
    # 1 / square root of each degree, used to balance well-tied and loosely
    # tied sensors.
    s = 1.0 / np.sqrt(deg)
    # Step 4: the "normalised Laplacian" L = I - D^(-1/2) W D^(-1/2). I (np.eye)
    # is the identity table: 1 on the diagonal, 0 elsewhere. This table
    # describes the whole web of rubber bands in one go.
    L = np.eye(n) - s[:, None] * W * s[None, :]           # step 4: normalised Laplacian
    # Step 5: its "eigenvalues" and "eigenvectors". Each eigenvector is one
    # natural way the web of bands can wobble, and its eigenvalue says how much
    # the bands stretch in that wobble. np.linalg.eigh (for symmetric tables)
    # lists them from the smallest eigenvalue up.
    vals, vecs = np.linalg.eigh(L)                        # step 5: eigenvalues in ascending order
    # Step 6: the "Fiedler vector", the eigenvector of the SECOND-smallest
    # eigenvalue (column 1; the very first is the boring one where nothing
    # stretches). It gives each sensor a number f_i; tightly tied sensors get
    # similar numbers.
    f = vecs[:, 1]                                        # step 6: Fiedler vector
    # best: the best cut found so far (none yet); tried: a record of every cut.
    best, tried = None, []
    # Step 7: try every possible cut. np.unique(f) lists the different values of
    # f, sorted; [:-1] leaves out the largest, because cutting there would put
    # everyone in A and nobody in B.
    for t in np.unique(f)[:-1]:                           # step 7: threshold with min max workload
        # A: the sensors whose number is at most t; B: the rest.
        A, B = ids[f <= t], ids[f > t]
        # The estimated time of each group.
        wA, wB = workload(A, xy, base, Ds, r, tau, p["v"]), workload(B, xy, base, Ds, r, tau, p["v"])
        # Write this cut down.
        tried.append((t, wA, wB))
        # The aim: the slower of the two groups should be as quick as possible
        # ("min max"). Keep this cut if its slower group beats the best so far.
        if best is None or max(wA, wB) < best[0]:
            # Remember its slower time, the two groups, and the cut's number.
            best = (max(wA, wB), A, B, len(tried) - 1)
    # If a trace dictionary was handed in, write the in-between results into it.
    if trace is not None:
        # Write in sigma, W, the eigenvalues, f, every threshold tried, both groups'
        # times for each, and the number of the best cut.
        trace.update(sigma=sigma, W=W, eigenvalues=vals, f=f, thresholds=np.array([x[0] for x in tried]),
                     workloads=np.array([x[1:] for x in tried]), best=best[3])
    # Hand back the two groups of the best cut.
    return best[1], best[2]
