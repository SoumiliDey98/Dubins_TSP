# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
# 
# WHAT THIS FILE IS FOR
# These are the tests: little programs that check the real programs give the
# right answers. Each test is a recipe whose name starts with "test_". It works
# something out and then says "assert" (insist) that a fact is true, for
# example "the path ends at the target". If a fact is false, the test FAILS
# and says where. Running this file runs every test and prints PASS or FAIL
# for each. Small numbers like 1e-9 (0.000000001) allow for tiny rounding
# errors in the computer's arithmetic.
# ==============================================================================
"""Tests for the new implementation. Run:  python tests/run_tests.py  (also works with pytest).

Unit tests check each module against the mathematics of Dubins_TSP.pdf; the
end-to-end tests run the whole algorithm (Steps 1-12) on random maps and check
every constraint of Section 0.7.
"""
# "import" borrows a toolbox. math: maths for single numbers.
import math
# os: folders and file names.
import os
# sys: the Python system itself (where to look for files).
import sys
# time: a stopwatch.
import time
# traceback: prints where an error happened.
import traceback

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np

# The folder this file is in (tests/).
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))                       # the new implementation
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(HERE)), "Dubins_TSP_Antigravity"))   # original dubins.py (read-only reference)

# (The lines above tell Python where to find the programs.) The programs
# being tested: the Dubins comparison ...
import compare_dubins
# ... the conflict game (nicknamed cg) ...
import conflict_game as cg
# ... the Dubins paths ...
import dubins_path
# ... the genetic algorithm ...
import ga_tour
# ... the distances round obstacles ...
import geodesic
# ... the bisection ...
import partition
# ... the whole planner ...
import planner
# ... the map maker ...
import scenario
# ... the simple "wait your turn" rule ...
import simple_resolver
# ... the drivable trajectories ...
import trajectory
# ... and the settings.
from config import default_params

# The normal settings, used by most tests.
P = default_params()
# A memory of maps already made, so each is made only once.
_cache = {}


# "def" starts a recipe (a "function"). This one gives the map for a seed
# and its distances round obstacles, making them only the first time.
def _map(seed=1):
    # Not made yet?
    if seed not in _cache:
        # Make the map ...
        scen = scenario.make(seed, P)
        # ... and its distances, and remember both.
        _cache[seed] = (scen, geodesic.build(scen, P))
    # Hand them back.
    return _cache[seed]


# ---------------------------------------------------------------- Dubins (0.1, 0.13)

# TEST: our Dubins lengths equal the original program's, and every path ends
# exactly at its target, pointing the right way.
def test_dubins_matches_original_and_ends_at_target():
    sys.dont_write_bytecode = True                               # leave the Antigravity folder untouched
    import dubins as original                                    # Dubins_TSP_Antigravity/dubins.py, read-only reference
    # (The two lines above borrow the original program, read-only.) A seeded
    # random-number maker, so the test is the same every time.
    rng = np.random.default_rng(0)
    # 500 random tries:
    for _ in range(500):
        # a random start pose in a 40 m square ...
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # ... and a random end pose;
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # our shortest shape and its three lengths;
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(*q0, *q1, P["R"])
        # its total length;
        length = S1 + Smiddle + S2
        # it must equal the original program's length;
        assert abs(length - original.returnDubinsPath(*q0, *q1, P["R"])[2]) < 1e-9
        # its points, every 0.25 m;
        xt, yt, s, psiT = dubins_path.calculateTrajectory(*q0, P["R"], turnType, S1, Smiddle, S2, 0.25)
        # the last point must be the end point;
        assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9
        # the end heading must be right (comparing angles round the circle);
        assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9
        # the distances must end at the length, and the points be at most 0.25 m apart.
        assert abs(s[-1] - length) < 1e-9 and np.all(np.diff(s) <= 0.25 + 1e-12)


# TEST: all six shapes (not only the shortest) end at the right pose, with
# no jumps and no skipped pieces.
def test_dubins_every_turn_type_reaches_the_target_smoothly():
    """All six turn types, not only the shortest: right end pose, and the points are `step` apart along
    the path, so no part is skipped and no jump is left out of pathDistance (limitations 4, 5 and 7)."""
    # A seeded random-number maker.
    rng = np.random.default_rng(1)
    # For two turning radii, 5 m and 1 m:
    for R in (P["R"], 1.0):
        # 100 random tries:
        for _ in range(100):
            # a random start pose ...
            q0 = (*rng.uniform(0, 8 * R, 2), rng.uniform(-math.pi, math.pi))
            # ... and end pose;
            q1 = (*rng.uniform(0, 8 * R, 2), rng.uniform(-math.pi, math.pi))
            # every one of the six shapes:
            for turnType in dubins_path.TURN_TYPES:
                # its three lengths;
                S1, Smiddle, S2 = (float(S) for S in dubins_path.calculateLengths(*q0, *q1, R, turnType))
                # if it does not exist here ...
                if not math.isfinite(S1):
                    # ... skip it;
                    continue
                # its points, every 0.05 R;
                xt, yt, s, psiT = dubins_path.calculateTrajectory(*q0, R, turnType, S1, Smiddle, S2, 0.05 * R)
                # it must end at the end point (if not, the message names the case);
                assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9, (turnType, q0, q1)
                # with the right heading;
                assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9, (turnType, q0, q1)
                # the straight gaps between neighbouring points;
                chords = np.hypot(np.diff(xt), np.diff(yt))
                # no gap may be longer than the distance along the path (no jumps);
                assert np.all(chords <= np.diff(s) + 1e-9), (turnType, q0, q1)
                # and the gaps must add up to the length, within 0.1 % (nothing skipped).
                assert abs(chords.sum() - s[-1]) <= 1e-3 * s[-1] + 1e-9, (turnType, q0, q1)


# TEST: the cases the old notebook program got wrong now work.
def test_dubins_cases_where_the_notebook_program_failed():
    # The turning radius, 5 m.
    R = P["R"]
    # With both ways of choosing shapes:
    for method in ("pdf", "shortest"):
        xt, yt, length, _ = dubins_path.returnDubinsPath(0.0, 0.0, 0.5, 20.0, 0.0, 0.0, R, method)    # limitation 1: cp2 = 0
        # (the line above is a case where cp2 = 0) it must end at the target, and be
        # longer than the 20 m straight line;
        assert np.hypot(xt[-1] - 20.0, yt[-1]) < 1e-9 and length > 20.0
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(3.0, 4.0, 0.5, 3.0, 4.0, 1.0, R, method)   # limitation 2: same point,
        xt, yt, _, psiT = dubins_path.calculateTrajectory(3.0, 4.0, 0.5, R, turnType, S1, Smiddle, S2, 0.25)  # other heading: a loop
        # (the two lines above: start and end at the same point, but a different
        # heading) it must come back to the point, after a real loop;
        assert np.hypot(xt[-1] - 3.0, yt[-1] - 4.0) < 1e-9 and S1 + Smiddle + S2 > 0
        # with the right heading.
        assert abs((psiT - 1.0 + math.pi) % (2 * math.pi) - math.pi) < 1e-9
    # A seeded random-number maker.
    rng = np.random.default_rng(5)
    for _ in range(500):                                                                               # rounding must not add a full circle
        # (500 tries; rounding must never add a full circle) a random pose;
        x, y, psi = *rng.uniform(0, 100, 2), rng.uniform(-math.pi, math.pi)
        # with both ways:
        for method in ("pdf", "shortest"):
            assert dubins_path.returnDubinsPath(x, y, psi, x, y, psi, R, method)[2] < 1e-5             # same pose: nothing to drive
            # (the line above: the same pose needs nothing) and 30 m straight ahead
            # must be exactly 30 m.
            assert abs(dubins_path.returnDubinsPath(x, y, psi, x + 30 * math.cos(psi), y + 30 * math.sin(psi),
                                                    psi, R, method)[2] - 30.0) < 1e-5                  # straight ahead
    # A seeded random-number maker.
    rng = np.random.default_rng(2)
    for _ in range(300):                                                                               # limitation 3: always a path
        # (300 tries: there must always be a path) a random start pose, close by ...
        q0 = (*rng.uniform(0, 3 * R, 2), rng.uniform(-math.pi, math.pi))
        # ... and end pose;
        q1 = (*rng.uniform(0, 3 * R, 2), rng.uniform(-math.pi, math.pi))
        # with both ways ...
        for method in ("pdf", "shortest"):
            # ... the length must be a real number, not infinity.
            assert math.isfinite(sum(dubins_path.findDubinsPath(*q0, *q1, R, method)[1:]))
    # limitation 4: LSR with Stan = 3R. The straight part is sqrt(9 - 4) R long, leaving at asin(2 / 3) from psiL.
    turnType, S1, dist, S2 = dubins_path.findDubinsPath(0.0, 0.0, 0.0, 15.0, 10.0, 0.0, R, "pdf")
    # (the case is described two lines above) the shape, the straight part and
    # the first arc must be exactly as worked out by hand.
    assert turnType == "LSR" and abs(dist - math.sqrt(5) * R) < 1e-9 and abs(S1 - R * math.asin(2 / 3)) < 1e-9


# TEST: the paper's rule picks the shape Section 0.13 says, in six chosen cases.
def test_pdf_rule_picks_the_word_of_section_0_13():
    R = P["R"]                                                   # = 5 m
    # (R, 5 m, on the line above) A quarter turn.
    h = math.pi / 2
    # Six end poses with the shape the rule must pick for each:
    for q1, word in [((40, 5, h), "LSL"), ((40, -5, -h), "RSR"), ((40, 5, -h), "LSR"), ((40, -5, h), "RSL"),
                     ((17, 5, h), "LRL"), ((17, -5, -h), "RLR")]:     # the last two: 2R <= S_tan = 12 m < 3R
        # starting at (0, 0) heading east, the rule must pick that shape.
        assert dubins_path.findDubinsPath(0, 0, 0, *q1, R, "pdf")[0] == word, (q1, word)


# TEST: when the rule's shape does not exist, the shortest one is used.
def test_pdf_rule_falls_back_when_its_word_does_not_exist():
    q0, q1 = (0, 0, 0), (7.5, 0, math.pi)                        # the rule picks RSL, but S_tan = 1.5 R < 2R
    # (the case is on the line above) The rule picks RSL ...
    assert dubins_path.findTurnType(*q0, *q1, P["R"]) == "RSL"
    # ... which cannot exist here, so "pdf" must give exactly the shortest.
    assert dubins_path.findDubinsPath(*q0, *q1, P["R"], "pdf") == dubins_path.findDubinsPath(*q0, *q1, P["R"], "shortest")


# TEST: the rule's paths end at the target, are never shorter than the
# shortest, and the one-at-a-time and table answers agree.
def test_pdf_rule_paths_reach_the_target_and_tables_agree():
    # A seeded random-number maker.
    rng = np.random.default_rng(0)
    # The turning radius.
    R = P["R"]
    # 500 random tries:
    for _ in range(500):
        # a random start pose ...
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # ... and end pose;
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # the rule's shape and lengths;
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(*q0, *q1, R, "pdf")
        # its total;
        length = S1 + Smiddle + S2
        # it can never beat the shortest;
        assert length >= sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:]) - 1e-9
        # the table recipe must give the same length;
        assert abs(length - dubins_path.returnPathDistances(*q0, *q1, R, "pdf")) < 1e-9
        # its points;
        xt, yt, _, psiT = dubins_path.calculateTrajectory(*q0, R, turnType, S1, Smiddle, S2, 0.25)
        # it must end at the end point ...
        assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9
        # ... with the right heading.
        assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9
    # 5 random start headings and 7 random end headings.
    h0, h1 = rng.uniform(-math.pi, math.pi, 5), rng.uniform(-math.pi, math.pi, 7)
    # The 5 x 7 table of lengths, all at once.
    table = dubins_path.returnPathDistances(0.0, 0.0, h0[:, None], 9.0, 4.0, h1[None, :], R, "pdf")
    # Every cell of the table ...
    for i, j in np.ndindex(table.shape):
        # ... must equal the one-at-a-time answer.
        assert abs(table[i, j] - sum(dubins_path.findDubinsPath(0.0, 0.0, h0[i], 9.0, 4.0, h1[j], R, "pdf")[1:])) < 1e-9


# TEST: compare_dubins.py sorts legs into the right kind.
def test_compare_counts_fallback_legs_as_missing_not_shortest():
    """The kind comes from the rule's own word, checked here against the table of all six words."""
    # The turning radius.
    R = P["R"]
    assert compare_dubins.classify_leg((0, 0, 0), (7.5, 0, math.pi), R) == "missing"     # RSL with S_tan = 1.5 R
    # (A known "missing" case is checked on the line above.) A seeded random-number maker.
    rng = np.random.default_rng(0)
    # The kinds met so far.
    seen = set()
    # 500 random tries:
    for _ in range(500):
        # a random start pose ...
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # ... and end pose;
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        # every shape's total length;
        totals = {t: float(sum(dubins_path.calculateLengths(*q0, *q1, R, t))) for t in dubins_path.TURN_TYPES}
        # the rule's shape's length;
        rule = totals[dubins_path.findTurnType(*q0, *q1, R)]
        # the kind worked out here, independently;
        expected = ("missing" if not math.isfinite(rule) else
                    "shortest" if rule <= min(totals.values()) + 1e-9 else "longer")
        # compare_dubins must agree;
        assert compare_dubins.classify_leg(q0, q1, R) == expected
        # note the kind.
        seen.add(expected)
    # All three kinds must have been met (so the test really tested each).
    assert seen == set(compare_dubins.LEG_KINDS)


# TEST: the four example legs show what their titles promise.
def test_compare_example_legs_show_what_their_titles_say():
    # The kind each must be.
    expected = ("shortest", "longer", "longer", "missing")
    # The four legs.
    legs = compare_dubins.example_legs(P["R"])
    # Their kinds must match.
    assert [leg["kind"] for leg in legs] == list(expected)
    assert legs[1]["rule"][1] == "S" and legs[2]["rule"][1] != "S" and legs[2]["shortest"]["turnType"][1] == "S"   # CSC, then CCC
    # (The shapes' types are checked on the line above.) For every leg:
    for leg in legs:
        # the rule's path is never shorter;
        assert leg["feasible"]["length"] >= leg["shortest"]["length"] - 1e-9
        for name in ("shortest", "feasible"):                    # both paths really join the two poses
            # (for both paths, line above) it ends at the end point.
            assert np.hypot(leg[name]["x"][-1] - leg["q1"][0], leg[name]["y"][-1] - leg["q1"][1]) < 1e-6


# TEST: in every distance band, the three kinds' shares add up to 100 %.
def test_compare_distance_bands_add_up():
    # For every band (40 random legs each, to be quick):
    for band in compare_dubins.distance_bands(P["R"], n=40):
        # the shares add up to 1, and the extra length is never negative.
        assert abs(sum(band[k] for k in compare_dubins.LEG_KINDS) - 1) < 1e-9 and band["extra"] >= 0


# ---------------------------------------------------------------- geodesic distances (eq. 9)

# TEST: the distances round obstacles make sense.
def test_geodesic_distances():
    # Map 1 and its distances.
    scen, geo = _map()
    # The terminals: base, then sensors.
    T = np.vstack([scen["base"], scen["xy"]])
    # The straight-line distance between every two.
    E = np.hypot(*(T[:, None] - T[None]).transpose(2, 0, 1))
    assert np.all(geo["D"] >= E - 1e-9)                          # never shorter than a straight line
    # (Never shorter than straight: line above.) A to B is as far as B to A.
    assert np.allclose(geo["D"], geo["D"].T)
    for b in range(1, len(T)):                                   # paths keep r_safe from the obstacles
        # (For every sensor b, line above:) its path from the base;
        pts = geodesic.path(geo, 0, b)
        # the path's length must equal its distance;
        assert abs(np.sum(np.hypot(*np.diff(pts, axis=0).T)) - geo["D"][0, b]) < 1e-6
        # and it must stay at least r_safe (2 m) from obstacles.
        assert trajectory.clearance(pts, geo) >= P["r_safe"]


# TEST: the visibility graph is the one build() uses, and keeps its margins.
def test_visibility_graph_is_the_graph_build_uses():
    # The shapes toolbox ...
    import shapely
    # ... the sparse table ...
    from scipy.sparse import csr_matrix
    # ... and Dijkstra.
    from scipy.sparse.csgraph import dijkstra
    # Map 1 and its distances.
    scen, geo = _map()
    # The graph.
    nodes, i, j, length = geodesic.visibility_graph(scen, P)
    # How many terminals.
    n_t = geo["n_terminals"]
    # Same points as build(), and every pair listed once.
    assert np.array_equal(nodes, geo["nodes"]) and np.all(i < j)
    # Each edge's length is its straight distance.
    assert np.allclose(length, np.hypot(*(nodes[i] - nodes[j]).T))
    # The graph as a table ...
    graph = csr_matrix((length, (i, j)), shape=(len(nodes), len(nodes)))
    # ... Dijkstra on it must give exactly build()'s distances.
    assert np.array_equal(dijkstra(graph, directed=False, indices=np.arange(n_t))[:, :n_t], geo["D"])
    # Every edge stays r cos(3 pi / 16) from the original obstacles (the chords of the inflated polygons,
    # theory_claude/geodesic_math.pdf), with r = r_safe + 0.5 at a terminal and r_plan between two corners.
    lines = shapely.linestrings(np.stack([nodes[i], nodes[j]], axis=1))
    # (Every edge as a line: line above.) Each edge's distance to the real obstacles.
    gap = shapely.distance(lines, geo["obstacles"])
    # Which edges touch a terminal.
    term = (i < n_t) | (j < n_t)
    # Those must keep 2.5 m x cos(3 pi / 16) = 2.08 m ...
    assert np.all(gap[term] >= (P["r_safe"] + 0.5) * math.cos(3 * math.pi / 16) - 1e-5)
    # ... the others 7 m x cos(3 pi / 16) = 5.82 m.
    assert np.all(gap[~term] >= P["r_plan"] * math.cos(3 * math.pi / 16) - 1e-5)


# TEST: the map maker puts many sensors close to obstacles, some in inner corners.
def test_scenario_puts_sensors_around_obstacles():
    # Points and polygons ...
    from shapely.geometry import Point, Polygon
    # ... and gluing shapes together.
    from shapely.ops import unary_union
    # Map 1.
    scen, _ = _map()
    # The obstacles as shapes.
    polys = [Polygon(o) for o in scen["obstacles"]]
    # Each sensor's distance to the nearest obstacle.
    d = np.array([unary_union(polys).distance(Point(x)) for x in scen["xy"]])
    # Every inner corner of every obstacle.
    inner = np.vstack([c for c in (scenario._inner_corners(q) for q in polys) if len(c)])
    # For each sensor: within 8 m of an inner corner?
    near_corner = [np.min(np.hypot(*(inner - x).T)) <= 8 for x in scen["xy"]]
    assert scen["n_near"] >= 0.4 * len(scen["xy"])               # most of near_share made it
    # (At least 40 % near obstacles: line above.) At least that many within 7 m,
    # and none closer than 3 m.
    assert np.sum(d <= P["near_gap"][1]) >= scen["n_near"] and d.min() >= P["near_gap"][0] - 1e-9
    assert sum(near_corner) >= 5                                   # some around concave corners


# TEST: with no obstacles, distances are straight lines.
def test_geodesic_without_obstacles_is_straight():
    # A tiny map: the base at (0, 0), two sensors, no obstacles.
    scen = {"base": np.array([0.0, 0.0]), "xy": np.array([[3.0, 4.0], [6.0, 8.0]]), "tau": np.ones(2), "obstacles": []}
    # Its distances.
    geo = geodesic.build(scen, P)
    # Base to (3, 4) is 5 m, and (3, 4) to (6, 8) is 5 m (Pythagoras: 3-4-5).
    assert abs(geo["D"][0, 1] - 5) < 1e-12 and abs(geo["D"][1, 2] - 5) < 1e-12


# ---------------------------------------------------------------- GA (0.8)

# TEST: every start tour has exactly one peak (Section 0.8.2).
def test_ga_initial_population_is_radially_unimodal():
    # Map 1 and its distances.
    scen, geo = _map()
    # Each sensor's distance from the base.
    r = geo["D"][0, 1:]
    # All sensors.
    ids = np.arange(len(scen["xy"]))
    # 50 start tours.
    pop = ga_tour.initial_population(ids, r, 50, np.random.default_rng(0))
    assert all(ga_tour.peaks(t, r) == 1 for t in pop)            # m(pi) = 1, Section 0.8.2
    # (One peak each: line above.) Each visits every sensor exactly once.
    assert all(sorted(t) == list(ids) for t in pop)


# TEST: the GA's answer visits every sensor once, and is at least as good as
# any start tour.
def test_ga_returns_a_better_permutation():
    # Map 1 and its distances.
    scen, geo = _map()
    # Distances from the base, and the first 40 sensors.
    r, ids = geo["D"][0, 1:], np.arange(40)
    # A seeded random-number maker.
    rng = np.random.default_rng(0)
    # The GA's best tour.
    best = ga_tour.optimize(ids, scen["xy"], scen["base"], r, P, rng)
    # The same start population.
    start = ga_tour.initial_population(ids, r, 50, np.random.default_rng(0))
    # A tiny recipe: a tour's fitness.
    f = lambda t: ga_tour.fitness(t, scen["xy"], scen["base"], r, P["R"])
    # Every sensor once, and no worse than the best start tour.
    assert sorted(best) == list(ids) and f(best) <= min(f(t) for t in start)


# TEST: counting peaks, on two hand-made examples.
def test_peaks_counts_radial_maxima():
    # Five sensors' distances from the base.
    r = np.array([1.0, 5.0, 3.0, 6.0, 2.0])
    # In order 1, 5, 3, 6, 2: two peaks (5 and 6). In order 1, 2, 3, 5, 6: one.
    assert ga_tour.peaks([0, 1, 2, 3, 4], r) == 2 and ga_tour.peaks([0, 4, 2, 1, 3], r) == 1


# TEST: recording the generations changes nothing, and the record adds up.
def test_ga_history_does_not_change_the_result():
    """Recording the generations (ga_evolution.py) must not change the tour, and what it records must add up."""
    # Map 1 and its distances.
    scen, geo = _map()
    # Positions, base, distances from the base.
    xy, base, r = scen["xy"], scen["base"], geo["D"][0, 1:]
    # The first 30 sensors.
    ids = np.arange(30)
    # Settings with only 40 generations, to be quick.
    p = default_params(generations=40)
    # The GA without recording ...
    plain = ga_tour.optimize(ids, xy, base, r, p, np.random.default_rng(3))
    # ... and with recording, from the same random numbers.
    history = []
    # (the GA with recording)
    logged = ga_tour.optimize(ids, xy, base, r, p, np.random.default_rng(3), history)
    # Both must give the same tour.
    assert logged == plain
    assert len(history) == p["generations"] + 1                   # generation 0 to the final population
    assert all(m == 1 for m in history[0]["peaks"])               # Section 0.8.2: generation 0 is radially unimodal
    # (41 entries, generation 0 one-peaked: lines above.) The best fitness of
    # each generation.
    best = [g["fitness"][0] for g in history]
    assert all(b1 <= b0 + 1e-9 for b0, b1 in zip(best, best[1:]))   # the better half is kept: the best never rises
    assert history[-1]["orders"][0] == plain                      # the last entry's best is the tour returned
    # (The best never gets worse, and the last is the answer: lines above.)
    # For every generation:
    for g in history:
        assert all(np.diff(g["fitness"]) >= -1e-9)                # best first
        # (sorted best first: line above) fitness = length + peaks x pi x R;
        assert np.allclose(g["fitness"], g["length"] + g["peaks"] * np.pi * p["R"])
        # and the best's fitness agrees with the fitness recipe.
        assert math.isclose(g["fitness"][0], ga_tour.fitness(g["orders"][0], xy, base, r, p["R"]))
    # With only two sensors ...
    short = []
    # ... the answer is simply nearest first ...
    assert ga_tour.optimize(np.arange(2), xy, base, r, p, np.random.default_rng(3), short) == sorted([0, 1], key=lambda i: r[i])
    assert short == []                                            # two sensors: no GA run, nothing recorded


# TEST: the GA's two moves, on hand-made tours.
def test_ga_crossover_and_swap():
    """The GA's moves (module_figures.py draws them): ordered crossover keeps the first parent's prefix and
    the second parent's order for the rest; swap exchanges two positions and leaves its input alone."""
    # Two parents.
    p1, p2 = [5, 1, 4, 2, 3, 0], [0, 2, 1, 3, 5, 4]
    # Cross them at 3.
    child = ga_tour.ordered_crossover(p1, p2, 3)
    # 5, 1, 4 from parent 1, then 0, 2, 3 in parent 2's order.
    assert child == [5, 1, 4, 0, 2, 3]
    # Every sensor exactly once.
    assert sorted(child) == sorted(p1)
    # Cutting at the very end copies parent 1.
    assert ga_tour.ordered_crossover(p1, p2, len(p1)) == p1
    # Swapping the first and last; the child handed in is left unchanged.
    assert ga_tour.swap(child, 0, 5) == [3, 1, 4, 0, 2, 5] and child == [5, 1, 4, 0, 2, 3]


# TEST: the planner's GA log changes nothing, and the animations can rebuild
# the queue and the partitions from it.
def test_plan_routes_log_changes_nothing_and_follows_the_queue():
    """The GA log of plan_routes (ga_evolution.py, pipeline_gifs.py) must not change the plan, and the
    splitting animation rebuilds the queue of groups from it."""
    # The animations' helpers.
    import animations
    # Map 1.
    scen, _ = _map()
    # A smaller map: only its first 25 sensors.
    sub = {**scen, "xy": scen["xy"][:25], "tau": scen["tau"][:25]}
    # Its distances.
    geo = geodesic.build(sub, P)
    # Plan the routes without a log ...
    plain, _ = planner.plan_routes(sub, geo, P, np.random.default_rng(4))
    # ... and with one, from the same random numbers.
    log = []
    # (with the log)
    logged, _ = planner.plan_routes(sub, geo, P, np.random.default_rng(4), ga_log=log)
    # Same tours either way.
    assert [t["order"] for t in logged] == [t["order"] for t in plain]
    # The kept runs in the log are exactly the tours.
    assert [r["order"] for r in log if r["kept"]] == [t["order"] for t in plain]
    assert any(not r["kept"] for r in log)                       # this map needs at least one split
    # (At least one split happened: line above.) Every logged run:
    for r in log:
        # its path starts and ends at the base;
        assert np.allclose(r["path"][0], sub["base"]) and np.allclose(r["path"][-1], sub["base"])
        # if it was split ...
        if not r["kept"]:
            # ... its two halves together ...
            halves = np.concatenate(r["halves"]).tolist()
            # ... hold exactly its sensors.
            assert sorted(halves) == sorted(np.asarray(r["ids"]).tolist())
    assert len(animations.split_states(log)) == len(log)         # raises if the log does not follow the queue
    states, splits, slot = animations.partition_states(log)      # 1 sink, then 2, then 3, ...
    # (The queue and partitions are rebuilt: lines above.) 1, 2, 3 ... groups,
    # one split between each.
    assert [len(s) for s in states] == list(range(1, len(plain) + 1)) and len(splits) == len(states) - 1
    for s in states:                                              # every step is a partition of all the sensors
        # (For every step, line above:) its groups hold every sensor exactly once.
        assert sorted(np.concatenate([log[j]["ids"] for j in s]).tolist()) == list(range(len(sub["xy"])))
    # The last step's groups are the planner's tours.
    assert sorted(log[j]["order"] for j in states[-1]) == sorted(t["order"] for t in plain)
    assert len({slot[j] for j in states[-1]}) == len(states[-1])  # every final group has its own colour number


# TEST: recording the game for the animations changes nothing.
def test_game_trace_changes_nothing():
    """The trace that conflict_game.resolve records for the game animation must not change its result."""
    # Settings with the base far away (so no docking zone) and a huge budget.
    p = default_params(base=(50.0, -100.0), B=1000.0)
    # Two robots crossing at right angles (a sure conflict).
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    # Their times and lengths.
    T, L = {k: 20.0 for k in trajs}, {k: 100.0 for k in trajs}
    # No obstacles.
    geo = {"obstacles": None}
    # Settle without recording ...
    plain = cg.resolve(trajs, T, L, geo, p)
    # ... and with recording.
    trace = []
    # (with recording)
    logged = cg.resolve(trajs, T, L, geo, p, trace)
    # Same trajectories and times either way.
    assert all(np.array_equal(plain[0][k], logged[0][k]) for k in trajs) and plain[1] == logged[1]
    # One recording per game, and at least one game.
    assert len(trace) == len(logged[3]) >= 1
    # Each recording ...
    for tr, g in zip(trace, logged[3]):
        # ... has the game's players and every robot's trajectory.
        assert sorted(tr["acts"]) == g["players"] and set(tr["trajs"]) == set(trajs)


# TEST: the animations count conflict time exactly as conflict_game does.
def test_animation_conflict_count_matches_summary():
    """The animations count the time in conflict as conflict_game.summary does."""
    # The animations' helpers.
    import animations
    # Frames 0 to 20, as a column.
    t = np.arange(0, 21)[:, None]
    # Three robots on straight lines; two pass head-on 1 m apart.
    trajs = {0: np.hstack([t * 2.0 + 60, 0 * t + 50]), 1: np.hstack([100 - t * 2.0, 0 * t + 51]),
             2: np.hstack([0 * t + 80, t * 2.0])}
    # The animations' count ...
    tl = animations.conflict_timeline(trajs, P)
    # ... and the game's.
    s = cg.summary(trajs, P)
    # There is a conflict, and both count the same seconds ...
    assert s["pairs"] and math.isclose(tl["cum"][-1], s["exposure"])
    # ... and the same closest distance.
    assert math.isclose(float(np.min(tl["closest"])), s["closest"])


# ---------------------------------------------------------------- partitioning (0.9)

# TEST: a split gives two non-empty halves that share no sensor and hold all.
def test_bisection_is_disjoint_and_complete():
    # Map 1 and its distances.
    scen, geo = _map()
    # All sensors.
    ids = np.arange(len(scen["xy"]))
    # Split them.
    A, B = partition.bisect(ids, scen["xy"], scen["base"], geo["D"][1:, 1:], geo["D"][0, 1:], scen["tau"], P)
    # Both non-empty, nothing shared, everything covered.
    assert len(A) and len(B) and not set(A) & set(B) and sorted(set(A) | set(B)) == list(ids)


# TEST: recording the split changes nothing, and the record adds up.
def test_bisection_trace_changes_nothing_and_adds_up():
    """The trace of partition.bisect (module_figures.py) must not change the split, and must record it:
    the chosen threshold is the min of the larger workload, and its workloads are those of the halves."""
    # Map 1 and its distances.
    scen, geo = _map()
    # Short names.
    xy, base, tau, Ds, r = scen["xy"], scen["base"], scen["tau"], geo["D"][1:, 1:], geo["D"][0, 1:]
    # All sensors.
    ids = np.arange(len(xy))
    # Split without recording ...
    A, B = partition.bisect(ids, xy, base, Ds, r, tau, P)
    # ... and with.
    tr = {}
    # (with recording)
    A2, B2 = partition.bisect(ids, xy, base, Ds, r, tau, P, trace=tr)
    # Same halves.
    assert np.array_equal(A, A2) and np.array_equal(B, B2)
    # One f per sensor, a full W table, and sigma at least R.
    assert len(tr["f"]) == len(ids) and tr["W"].shape == (len(ids), len(ids)) and tr["sigma"] >= P["R"]
    # The chosen cut is the one whose slower half is quickest.
    assert tr["best"] == int(np.argmin(tr["workloads"].max(1)))
    # A is exactly the sensors with f at most the chosen threshold.
    assert np.array_equal(A, ids[tr["f"] <= tr["thresholds"][tr["best"]]])
    # The recorded times are the halves' real estimates.
    assert np.allclose(tr["workloads"][tr["best"]], [partition.workload(A, xy, base, Ds, r, tau, P["v"]),
                                                     partition.workload(B, xy, base, Ds, r, tau, P["v"])])
    seq = partition.sweep_order(A, xy, base)                     # the tour that workload measures
    # (A's sweep order: line above.) It holds exactly A's sensors ...
    assert sorted(seq.tolist()) == sorted(A.tolist())
    # ... and their angles round the base ...
    ang = np.arctan2(*(xy[seq] - base).T[::-1])
    assert np.sum(np.diff(ang) < 0) <= 1                          # by angle, wrapping round once at most


# TEST: the two pictures that need no plan can be drawn.
def test_module_figures_draw():
    """The two figures of module_figures.py that need no plan are drawn without error."""
    # tempfile: a folder that is deleted afterwards.
    import tempfile
    # The pictures.
    import module_figures
    # In a temporary folder d:
    with tempfile.TemporaryDirectory() as d:
        # draw the flow diagram;
        module_figures.flow_figure(P, os.path.join(d, "flow.png"))
        # draw the geodesic picture;
        s = module_figures.geodesic_figure(1, P, os.path.join(d, "geodesic.png"))
        # both files must be there and not empty.
        assert os.path.getsize(os.path.join(d, "flow.png")) > 0 and os.path.getsize(os.path.join(d, "geodesic.png")) > 0
    # The picture counted the base and every sensor.
    assert s["terminals"] == len(_map()[0]["xy"]) + 1


# ---------------------------------------------------------------- trajectories (eq. 17, 19)

# TEST: drivable trajectories obey the time formula, start and end at the
# base, never jump, keep r_safe from obstacles, and never bend too tightly.
def test_trajectory_time_clearance_and_curvature():
    # Map 1 and its distances.
    scen, geo = _map()
    # A seeded random-number maker.
    rng = np.random.default_rng(0)
    # 15 random tries:
    for _ in range(15):
        # 2 to 9 random sensors in a random order;
        order = list(rng.choice(len(scen["xy"]), size=rng.integers(2, 10), replace=False))
        # the drivable trajectory;
        tr = trajectory.build(order, geo, scen, P)
        assert abs(tr["T"] - (tr["L"] / P["v"] + scen["tau"][order].sum())) < 1e-9     # eq. (17)
        # (T = L / v + waiting: line above) it starts and ends at the base;
        assert np.allclose(tr["frames"][0], scen["base"]) and np.allclose(tr["frames"][-1], scen["base"])
        # no frame-to-frame step is longer than 2.5 m (5 m/s x 0.5 s);
        assert np.max(np.hypot(*np.diff(tr["frames"], axis=0).T)) <= P["v"] * P["dt"] + 1e-9
        assert trajectory.clearance(tr["path"], geo) >= P["r_safe"]                     # eq. (19)
        assert trajectory.max_curvature(tr["path"]) <= 1.02 / P["R"]                     # Dubins: radius >= R


# ---------------------------------------------------------------- conflict game (0.10)

# A helper: a straight trajectory from point a to point b in n steps.
def _line(a, b, n=40):
    # The two ends as numbers.
    a, b = np.array(a, float), np.array(b, float)
    # n + 1 evenly spaced points from a to b.
    return np.array([a + (b - a) * i / n for i in range(n + 1)])


# TEST: the game settles two simple conflicts as expected.
def test_game_head_on_and_crossing():
    # The base far away (no docking zone) and a huge budget.
    p = default_params(base=(50.0, -100.0), B=1000.0)
    # No obstacles.
    geo = {"obstacles": None}
    # Two cases: head-on on one line (robot 0 should DETOUR), and crossing at
    # right angles (robot 0 should WAIT):
    for trajs, expected in [({0: _line((0, 50), (100, 50)), 1: _line((100, 50), (0, 50))}, "DETOUR"),
                            ({0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}, "WAIT")]:
        # both take 20 s;
        T = {k: 20.0 for k in trajs}
        # settle the conflict;
        new, T2, _, games = cg.resolve(trajs, T, {k: 100.0 for k in trajs}, geo, p)
        # robot 0 does what was expected, robot 1 GOes;
        assert games[0]["equilibrium"] == {0: expected, 1: "GO"}
        # no conflict is left, and they never come within 3 m.
        assert cg.summary(new, p)["exposure"] == 0.0 and cg.summary(new, p)["closest"] >= p["d_safe"]


# TEST: on random games, a player's cost change always equals the potential's
# change (the property that makes the game always stop).
def test_game_is_an_exact_potential_game():
    """Delta J_k = Delta Phi for every single-player change, on random 3-player games."""
    # The base far away and a huge budget.
    p = default_params(base=(500.0, 500.0), B=1000.0)
    # A seeded random-number maker.
    rng = np.random.default_rng(1)
    # 20 random games:
    for _ in range(20):
        # 4 robots on random wandering paths (each step a small random move);
        trajs = {k: np.cumsum(rng.normal(0, 1.5, (60, 2)), axis=0) + rng.uniform(0, 6, 2) for k in range(4)}
        # 3 of them play; robot 3 just drives;
        players = [0, 1, 2]
        # each player may GO, WAIT 3 frames or WAIT 7 frames;
        acts = {k: [{"name": n, "traj": cg.wait_traj(trajs[k], 5, w) if w else trajs[k], "dT": w * 0.5, "dD": 0.0,
                     "W": w * 0.5} for n, w in (("GO", 0), ("WAIT", 3), ("WAIT", 7))] for k in players}
        # E: the danger between two trajectories, from frame 5;
        E = lambda x, y: cg.exposure(x, y, 5, p)
        # each strategy's own cost;
        c = {k: [a["dT"] + a["W"] for a in acts[k]] for k in players}
        # the price of danger, worked out as the game does;
        alpha = (1 + sum(max(c[k]) - min(c[k]) for k in players)) / p["dt"]
        # J: player k's cost under the choices a, written out independently;
        J = lambda k, a: alpha * (sum(E(acts[k][a[k]]["traj"], acts[j][a[j]]["traj"]) for j in players if j != k)
                                  + E(acts[k][a[k]]["traj"], trajs[3])) + c[k][a[k]]
        # Phi: the potential, written out independently (the "\" continues the line);
        Phi = lambda a: alpha * (sum(E(acts[k][a[k]]["traj"], acts[j][a[j]]["traj"]) for i, k in enumerate(players)
                                     for j in players[i + 1:]) + sum(E(acts[k][a[k]]["traj"], trajs[3]) for k in players)) \
            + sum(c[k][a[k]] for k in players)
        # random choices for all players;
        a = {k: int(rng.integers(3)) for k in players}
        # for every player k ...
        for k in players:
            # ... and every strategy x it could switch to:
            for x in range(3):
                # the choices with k switched to x;
                b = {**a, k: x}
                # k's cost change must equal Phi's change, exactly.
                assert abs((J(k, b) - J(k, a)) - (Phi(b) - Phi(a))) < 1e-9
        # Then play the real game ...
        prof, rec = cg.play(players, acts, {**{k: trajs[k] for k in players}, 3: trajs[3]}, 5, p)
        # ... whose potential must fall at every switch.
        assert all(rec["potential"][i + 1] < rec["potential"][i] for i in range(len(rec["potential"]) - 1))


# TEST: the game never makes a robot exceed the budget.
def test_game_respects_the_budget():
    p = default_params(base=(50.0, -100.0), B=20.5)              # 0.5 s of slack only
    # (Only 0.5 s to spare: line above.) Two robots crossing.
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    # Settle the conflict.
    _, T2, _, _ = cg.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    # Every time must still be within B.
    assert all(t <= p["B"] + 1e-9 for t in T2.values())


# ---------------------------------------------------------------- simple resolver ("wait your turn")

# TEST: with the simple rule, the shorter tour waits and the conflict is gone.
def test_simple_resolver_crossing_one_sink_waits():
    # The base far away and a huge budget.
    p = default_params(base=(50.0, -100.0), B=1000.0)
    # Two robots crossing.
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    # Robot 1's tour is longer (21 s).
    T, L = {0: 20.0, 1: 21.0}, {0: 100.0, 1: 100.0}
    # Settle with the simple rule.
    new, T2, L2, stops = simple_resolver.resolve(trajs, T, L, {"obstacles": None}, p)
    assert cg.summary(new, p)["exposure"] == 0.0                 # the independent check agrees
    assert [s["sink"] for s in stops] == [0]                     # the longer tour (sink 1) never waits
    # (No conflict left, only robot 0 stopped: lines above.) Robot 1's time is
    # unchanged, robot 0's grew by its wait, and no distance was added.
    assert T2[1] == 21.0 and T2[0] == 20.0 + stops[0]["wait"] and L2 == L
    # Robot 1's trajectory is untouched.
    assert np.array_equal(new[1], trajs[1])


# TEST: the simple rule never changes a path, only when the robot is where.
def test_simple_resolver_only_changes_timing():
    # The base far away and a huge budget.
    p = default_params(base=(50.0, -100.0), B=1000.0)
    # Two robots crossing.
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    # Settle with the simple rule.
    new, _, _, _ = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    for k in trajs:                                              # same path, repeated positions only
        # (For each robot, line above:) drop the repeated positions (the waits) ...
        q = new[k][np.concatenate([[True], np.hypot(*np.diff(new[k], axis=0).T) > 1e-9])]
        # ... and what is left must be exactly the original path.
        assert np.array_equal(q, trajs[k])


# TEST: the simple rule never makes a robot exceed the budget.
def test_simple_resolver_respects_the_budget():
    p = default_params(base=(50.0, -100.0), B=20.5)              # 0.5 s of slack only
    # (Only 0.5 s to spare: line above.) Two robots crossing.
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    # Settle with the simple rule.
    _, T2, _, _ = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    # Every time must still be within B.
    assert all(t <= p["B"] + 1e-9 for t in T2.values())


# TEST: when waiting cannot help, the simple rule leaves the conflict visible.
def test_simple_resolver_reports_what_it_cannot_solve():
    """Head-on on one line: waiting cannot help, the other sink drives through. No stops; the conflict stays visible."""
    # The base far away and a huge budget.
    p = default_params(base=(50.0, -100.0), B=1000.0)
    # Two robots head-on on the same line.
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((100, 50), (0, 50))}
    # Settle with the simple rule.
    new, _, _, stops = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    # No stop was made, and the conflict is still there to be reported.
    assert stops == [] and cg.summary(new, p)["exposure"] > 0


# TEST: the whole planner on map 1 with the simple rule passes every check.
def test_end_to_end_with_simple_resolver():
    # Settings with the simple rule.
    p = default_params(resolver="simple")
    # Run the whole planner.
    out = planner.run(1, p)
    # Its checks.
    c = out["checks"]
    # Every sensor visited, every budget kept, not too many robots, no problems.
    assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], c
    # Far enough from obstacles, and no bend too tight.
    assert c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"], c
    # Print a summary line.
    print(f"      seed 1: K={c['K']} T_max={c['T_max']:.1f}s conflict pairs "
          f"{len(out['result']['before']['pairs'])}->{c['conflict_pairs']} closest {c['closest']:.2f}m")


# ---------------------------------------------------------------- whole algorithm (0.11)

# TEST: the whole planner on maps 1, 2 and 3 passes every check.
def test_end_to_end(seeds=(1, 2, 3)):
    # For each map:
    for seed in seeds:
        # run the whole planner;
        out = planner.run(seed)
        # its checks;
        c = out["checks"]
        # every sensor visited, every budget kept, not too many robots, no problems;
        assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], (seed, c)
        # far enough from obstacles;
        assert c["clearance"] >= P["r_safe"], (seed, c["clearance"])
        # no bend too tight;
        assert c["curvature_x_R"] <= 1 + P["curvature_tol"], (seed, c["curvature_x_R"])
        # print a summary line.
        print(f"      seed {seed}: K={c['K']} T_max={c['T_max']:.1f}s L_total={c['L_total']:.0f}m "
              f"conflict pairs {len(out['result']['before']['pairs'])}->{c['conflict_pairs']} "
              f"closest {c['closest']:.2f}m clearance {c['clearance']:.2f}m")


# TEST: the whole planner on map 1 with the paper's shape rule passes every check.
def test_end_to_end_with_pdf_rule():
    # Settings with the paper's rule.
    p = default_params(dubins_method="pdf")
    # Run the whole planner.
    out = planner.run(1, p)
    # Its checks.
    c = out["checks"]
    # Every sensor visited, every budget kept, not too many robots, no problems.
    assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], c
    # Far enough from obstacles, and no bend too tight.
    assert c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"], c
    # Print a summary line.
    print(f"      seed 1: K={c['K']} T_max={c['T_max']:.1f}s L_total={c['L_total']:.0f}m")


# TEST: running the same map twice gives exactly the same result.
def test_same_seed_same_result():
    # Map 2, twice.
    a, b = planner.run(2), planner.run(2)
    # The same visiting orders ...
    assert [t["order"] for t in a["result"]["tours"]] == [t["order"] for t in b["result"]["tours"]]
    # ... and the same longest time.
    assert a["checks"]["T_max"] == b["checks"]["T_max"]


# True only when this file is started directly: then run every test.
if __name__ == "__main__":
    # Every recipe in this file whose name starts with "test_", in name order.
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    # No failures yet.
    failed = 0
    # For each test:
    for name, f in tests:
        # start the stopwatch;
        t0 = time.time()
        # "try" runs the test; if anything goes wrong ...
        try:
            # (run it)
            f()
            # (if it finished: PASS, and how long it took)
            print(f"PASS  {name}  ({time.time() - t0:.1f} s)")
        # ... we land here:
        except Exception:
            # count the failure;
            failed += 1
            # say FAIL;
            print(f"FAIL  {name}")
            # and print where it went wrong.
            traceback.print_exc()
    # The totals.
    print(f"\n{len(tests) - failed} passed, {failed} failed")
    # Finish: exit code 0 means all passed, 1 means something failed.
    sys.exit(1 if failed else 0)
