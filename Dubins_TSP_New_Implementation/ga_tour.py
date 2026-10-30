"""Radially unimodal genetic algorithm (Section 0.8).

A tour visits sensor indices in some order, starting and ending at the base.
r[i] is the geodesic distance of sensor i from the base (eq. 9).
"""
import numpy as np


def peaks(order, r):
    """m(pi): number of radial maxima of the tour, with r = 0 at the base at both ends.
    A flat top (equal neighbours) counts as one peak."""
    d = np.concatenate([[0.0], r[order], [0.0]])
    return max(int(np.sum((d[1:-1] > d[:-2]) & (d[1:-1] > d[2:]))), 1)


def fitness(order, xy, base, r, R):
    """Fitness(pi) = L_euclid(pi) + m(pi) * pi * R   (Section 0.8.1); lower is better."""
    pts = np.vstack([base, xy[order], base])
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T))) + peaks(order, r) * np.pi * R


def initial_population(ids, r, size, rng):
    """Section 0.8.2: split the sensors at random into S_out and S_in, sort S_out by
    increasing r and S_in by decreasing r, and join them: every tour has m(pi) = 1."""
    pop = []
    for _ in range(size):
        out = rng.random(len(ids)) < 0.5
        s_out = sorted(ids[out].tolist(), key=lambda i: r[i])
        s_in = sorted(ids[~out].tolist(), key=lambda i: -r[i])
        pop.append(s_out + s_in)
    return pop


def optimize(ids, xy, base, r, p, rng):
    """Best visiting order of the sensors `ids` found by the GA: keep the better half,
    refill with ordered crossover (a prefix of one parent, the rest in the other
    parent's order) and an occasional swap mutation."""
    ids = np.asarray(ids)
    if len(ids) <= 2:
        return sorted(ids.tolist(), key=lambda i: r[i])
    score = lambda tour: fitness(tour, xy, base, r, p["R"])
    pop = initial_population(ids, r, p["pop_size"], rng)
    half = p["pop_size"] // 2
    for _ in range(p["generations"]):
        pop.sort(key=score)
        parents, children = pop[:half], []
        while half + len(children) < p["pop_size"]:
            i, j = rng.choice(half, 2, replace=False)
            cut = int(rng.integers(1, len(ids)))
            head = parents[i][:cut]
            child = head + [g for g in parents[j] if g not in set(head)]
            if rng.random() < p["mutation"]:
                a, b = rng.choice(len(child), 2, replace=False)
                child[a], child[b] = child[b], child[a]
            children.append(child)
        pop = parents + children
    return min(pop, key=score)
