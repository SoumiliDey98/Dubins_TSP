# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a short
# description written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# A robot must visit a group of sensors and come back to the base. In which
# ORDER should it visit them? This file answers with a "genetic algorithm"
# (GA), which copies how living things improve over many generations:
#   - Keep a "population" of 50 possible orders (each order is called a "tour").
#   - Give each tour a score, its "fitness" (lower is better here).
#   - Keep the better half; make "children" by mixing two good tours
#     ("crossover"), sometimes with a small random change ("mutation").
#   - Repeat 100 times. The best tour at the end is the answer.
# "Radially unimodal" (Section 0.8 of the paper) means: the robot first moves
# further and further away from the base, then closer and closer on the way
# back, so its distance from the base has only ONE peak, like a single hill.
# Tours with more peaks zig-zag, and each extra peak costs one extra U-turn.
# ==============================================================================
"""Radially unimodal genetic algorithm (Section 0.8).

A tour visits sensor indices in some order, starting and ending at the base.
r[i] is the geodesic distance of sensor i from the base (eq. 9).
"""
# "import" borrows a toolbox. numpy (nicknamed np) works with long lists of
# numbers ("arrays") very quickly.
import numpy as np


# "def" starts a recipe (a "function"). This one counts the peaks, m(pi), of a tour.
# Inputs: "order", the sensors' numbers in visiting order; and "r", where r[i]
# is how far sensor i is from the base, going round obstacles (from geodesic.py).
def peaks(order, r):
    """m(pi): number of radial maxima of the tour, with r = 0 at the base at both ends.
    A flat top (equal neighbours) counts as one peak."""
    # d: the distance from the base at every stop of the trip, with 0 added at
    # the start and at the end, because the trip starts and ends AT the base.
    # r[order] picks the distances in visiting order; np.concatenate joins lists.
    d = np.concatenate([[0.0], r[order], [0.0]])
    # A stop is a peak if it is further away than the stop before it AND further
    # away than the stop after it. d[1:-1] is every stop except the two ends;
    # d[:-2] is the stop before each of them, d[2:] the stop after. "&" means
    # "and". np.sum counts how many are True. max(..., 1) makes the answer at
    # least 1, since every trip goes out and comes back at least once.
    return max(int(np.sum((d[1:-1] > d[:-2]) & (d[1:-1] > d[2:]))), 1)


# How long the trip would be in straight lines: base, each sensor in order, base.
def euclid_length(order, xy, base):
    """L_euclid(pi): straight-line length of the tour from the base and back. Obstacles are ignored."""
    # All the points of the trip, one row (x, y) each, in order: the base, the
    # sensors (xy[order]) and the base again. np.vstack stacks them on top of
    # each other.
    pts = np.vstack([base, xy[order], base])
    # np.diff gives the step from each point to the next. np.hypot turns every
    # step into its length (Pythagoras). np.sum adds the lengths up. float(...)
    # makes the answer an ordinary decimal number.
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))


# The score of a tour (Section 0.8.1 of the paper). Lower is better.
# R is the turning radius, 5 m.
def fitness(order, xy, base, r, R):
    """Fitness(pi) = L_euclid(pi) + m(pi) * pi * R   (Section 0.8.1); lower is better."""
    # Straight-line length, plus a penalty of pi * R (about 15.7 m, half a
    # circle of radius R) for every peak, because at every peak the robot has to
    # turn round, and turning round needs about half a circle.
    return euclid_length(order, xy, base) + peaks(order, r) * np.pi * R


# The "_" at the start of the name means "only used inside this file".
# Writes down one generation in the "history" list, so pictures can be drawn
# later. It never touches the random numbers, so it cannot change the result.
def _record(history, pop, xy, base, r, R):
    """Append one generation to history: every tour, best first, with its straight-line length,
    its number of peaks m(pi) and its fitness. Makes no random draws."""
    # A copy of the population sorted by fitness, best first. "lambda" makes a
    # tiny recipe without a name: here, "give a tour, get its fitness".
    pop = sorted(pop, key=lambda tour: fitness(tour, xy, base, r, R))
    # The straight-line length of every tour, as an array.
    length = np.array([euclid_length(tour, xy, base) for tour in pop])
    # The number of peaks of every tour.
    m = np.array([peaks(tour, r) for tour in pop])
    # Add one entry (a dictionary, a table of named values) to the end of the
    # history: the tours, their lengths, their peaks and their fitness values.
    history.append({"orders": [list(tour) for tour in pop], "length": length, "peaks": m,
                    "fitness": length + m * np.pi * R})


# Makes the first population (Section 0.8.2): tours that are all single-peaked.
# Inputs: ids, the sensors of the group; r, their distances from the base;
# size, how many tours to make (50); rng, the random-number maker.
def initial_population(ids, r, size, rng):
    """Section 0.8.2: split the sensors at random into S_out and S_in, sort S_out by
    increasing r and S_in by decreasing r, and join them: every tour has m(pi) = 1."""
    # Start with an empty list of tours.
    pop = []
    # Repeat "size" times, making one tour each time.
    for _ in range(size):
        # Toss a coin for every sensor: rng.random gives a number between 0 and
        # 1, and "< 0.5" is True half the time. True means "visit on the way
        # OUT"; False means "visit on the way back IN".
        out = rng.random(len(ids)) < 0.5
        # The "out" sensors, nearest to the base first (sorted by r).
        s_out = sorted(ids[out].tolist(), key=lambda i: r[i])
        # The "in" sensors (~ means "not"), furthest first (sorted by minus r).
        s_in = sorted(ids[~out].tolist(), key=lambda i: -r[i])
        # Join them: going away, then coming back. The distance rises, then
        # falls, so the tour has exactly one peak.
        pop.append(s_out + s_in)
    # Hand back the 50 tours.
    return pop


# Crossover: makes a child tour from two parent tours.
def ordered_crossover(first, second, cut):
    """Child of two tours: the first `cut` sensors of `first`, then the others in the order of `second`.
    (The paper does not say how the GA crosses tours; this is the code's choice.)"""
    # The first "cut" sensors of the first parent (first[:cut] means "from the
    # start up to position cut").
    head = first[:cut]
    # Then every sensor of the second parent that is not already in the head,
    # in the second parent's order. set(head) makes the look-up "is it already
    # there?" fast. Every sensor appears exactly once in the child.
    return head + [g for g in second if g not in set(head)]


# Mutation: two sensors swap places in the tour.
def swap(tour, a, b):
    """Swap mutation: the sensors at positions a and b change places. Returns a new list."""
    # Make a copy, so the tour that was handed in is not changed.
    tour = list(tour)
    # Swap the sensors at positions a and b (Python can swap two things in one line).
    tour[a], tour[b] = tour[b], tour[a]
    # Hand back the changed copy.
    return tour


# The genetic algorithm itself. Inputs: ids, the sensors of the group; xy, all
# sensor positions; base, the base position; r, distances from the base; p, the
# settings; rng, the random-number maker; history, an optional list to record
# every generation in ("=None" means it may be left out).
def optimize(ids, xy, base, r, p, rng, history=None):
    """Best visiting order of the sensors `ids` found by the GA: keep the better half,
    refill with ordered crossover (a prefix of one parent, the rest in the other
    parent's order) and an occasional swap mutation.

    If history is a list, one entry per generation is appended to it (see _record): entry 0
    is the initial population and the last entry the final one, so there are generations + 1.
    Recording makes no random draws, so the result is the same with or without it. With
    two sensors or fewer there is no GA run and history stays empty."""
    # Make sure ids is a numpy array.
    ids = np.asarray(ids)
    # With 1 or 2 sensors there is nothing to search: visit them nearest first.
    if len(ids) <= 2:
        # Hand back the sensors sorted by distance from the base.
        return sorted(ids.tolist(), key=lambda i: r[i])
    # "score" is a tiny recipe: give it a tour, it gives the tour's fitness.
    score = lambda tour: fitness(tour, xy, base, r, p["R"])
    # The first population: 50 single-peaked tours.
    pop = initial_population(ids, r, p["pop_size"], rng)
    # Half the population size: 50 // 2 = 25 ("//" divides and drops any remainder).
    half = p["pop_size"] // 2
    # Repeat for 100 generations.
    for _ in range(p["generations"]):
        # Sort the tours by score, best (lowest) first.
        pop.sort(key=score)
        # If a history list was handed in, write this generation down.
        if history is not None:
            # Write this generation down.
            _record(history, pop, xy, base, r, p["R"])
        # The better 25 tours become the parents. The children list starts empty.
        parents, children = pop[:half], []
        # Make children until parents + children make 50 tours again.
        while half + len(children) < p["pop_size"]:
            # Pick two different parents at random (replace=False: not the same one twice).
            i, j = rng.choice(half, 2, replace=False)
            # Cross them at a random cut, from 1 up to (but not including) the
            # number of sensors.
            child = ordered_crossover(parents[i], parents[j], int(rng.integers(1, len(ids))))
            # With probability 0.1 (one time in ten), mutate the child: swap two
            # sensors at two different random positions. The "*" hands the two
            # positions to swap as its a and b.
            if rng.random() < p["mutation"]:
                # Swap the sensors at two different random positions (the "*" hands the two
                # positions to swap as its a and b).
                child = swap(child, *rng.choice(len(child), 2, replace=False))
            # Add the child to the list.
            children.append(child)
        # The next generation: the 25 parents and the 25 children. Because the
        # best tours are always kept, the best score can never get worse.
        pop = parents + children
    # Write down the final population too.
    if history is not None:
        # Write the final population down.
        _record(history, pop, xy, base, r, p["R"])
    # Hand back the tour with the lowest score.
    return min(pop, key=score)
