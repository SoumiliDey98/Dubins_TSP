"""Tests for the new implementation. Run:  python tests/run_tests.py  (also works with pytest).

Unit tests check each module against the mathematics of Dubins_TSP.pdf; the
end-to-end tests run the whole algorithm (Steps 1-12) on random maps and check
every constraint of Section 0.7.
"""
import math
import os
import sys
import time
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))                       # the new implementation
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(HERE)), "Dubins_TSP_Antigravity"))   # original dubins.py (read-only reference)

import compare_dubins
import conflict_game as cg
import dubins_path
import ga_tour
import geodesic
import partition
import planner
import scenario
import simple_resolver
import trajectory
from config import default_params

P = default_params()
_cache = {}


def _map(seed=1):
    if seed not in _cache:
        scen = scenario.make(seed, P)
        _cache[seed] = (scen, geodesic.build(scen, P))
    return _cache[seed]


# ---------------------------------------------------------------- Dubins (0.1, 0.13)

def test_dubins_matches_original_and_ends_at_target():
    sys.dont_write_bytecode = True                               # leave the Antigravity folder untouched
    import dubins as original                                    # Dubins_TSP_Antigravity/dubins.py, read-only reference
    rng = np.random.default_rng(0)
    for _ in range(500):
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(*q0, *q1, P["R"])
        length = S1 + Smiddle + S2
        assert abs(length - original.returnDubinsPath(*q0, *q1, P["R"])[2]) < 1e-9
        xt, yt, s, psiT = dubins_path.calculateTrajectory(*q0, P["R"], turnType, S1, Smiddle, S2, 0.25)
        assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9
        assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9
        assert abs(s[-1] - length) < 1e-9 and np.all(np.diff(s) <= 0.25 + 1e-12)


def test_dubins_every_turn_type_reaches_the_target_smoothly():
    """All six turn types, not only the shortest: right end pose, and the points are `step` apart along
    the path, so no part is skipped and no jump is left out of pathDistance (limitations 4, 5 and 7)."""
    rng = np.random.default_rng(1)
    for R in (P["R"], 1.0):
        for _ in range(100):
            q0 = (*rng.uniform(0, 8 * R, 2), rng.uniform(-math.pi, math.pi))
            q1 = (*rng.uniform(0, 8 * R, 2), rng.uniform(-math.pi, math.pi))
            for turnType in dubins_path.TURN_TYPES:
                S1, Smiddle, S2 = (float(S) for S in dubins_path.calculateLengths(*q0, *q1, R, turnType))
                if not math.isfinite(S1):
                    continue
                xt, yt, s, psiT = dubins_path.calculateTrajectory(*q0, R, turnType, S1, Smiddle, S2, 0.05 * R)
                assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9, (turnType, q0, q1)
                assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9, (turnType, q0, q1)
                chords = np.hypot(np.diff(xt), np.diff(yt))
                assert np.all(chords <= np.diff(s) + 1e-9), (turnType, q0, q1)
                assert abs(chords.sum() - s[-1]) <= 1e-3 * s[-1] + 1e-9, (turnType, q0, q1)


def test_dubins_cases_where_the_notebook_program_failed():
    R = P["R"]
    for method in ("pdf", "shortest"):
        xt, yt, length, _ = dubins_path.returnDubinsPath(0.0, 0.0, 0.5, 20.0, 0.0, 0.0, R, method)    # limitation 1: cp2 = 0
        assert np.hypot(xt[-1] - 20.0, yt[-1]) < 1e-9 and length > 20.0
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(3.0, 4.0, 0.5, 3.0, 4.0, 1.0, R, method)   # limitation 2: same point,
        xt, yt, _, psiT = dubins_path.calculateTrajectory(3.0, 4.0, 0.5, R, turnType, S1, Smiddle, S2, 0.25)  # other heading: a loop
        assert np.hypot(xt[-1] - 3.0, yt[-1] - 4.0) < 1e-9 and S1 + Smiddle + S2 > 0
        assert abs((psiT - 1.0 + math.pi) % (2 * math.pi) - math.pi) < 1e-9
    rng = np.random.default_rng(5)
    for _ in range(500):                                                                               # rounding must not add a full circle
        x, y, psi = *rng.uniform(0, 100, 2), rng.uniform(-math.pi, math.pi)
        for method in ("pdf", "shortest"):
            assert dubins_path.returnDubinsPath(x, y, psi, x, y, psi, R, method)[2] < 1e-5             # same pose: nothing to drive
            assert abs(dubins_path.returnDubinsPath(x, y, psi, x + 30 * math.cos(psi), y + 30 * math.sin(psi),
                                                    psi, R, method)[2] - 30.0) < 1e-5                  # straight ahead
    rng = np.random.default_rng(2)
    for _ in range(300):                                                                               # limitation 3: always a path
        q0 = (*rng.uniform(0, 3 * R, 2), rng.uniform(-math.pi, math.pi))
        q1 = (*rng.uniform(0, 3 * R, 2), rng.uniform(-math.pi, math.pi))
        for method in ("pdf", "shortest"):
            assert math.isfinite(sum(dubins_path.findDubinsPath(*q0, *q1, R, method)[1:]))
    # limitation 4: LSR with Stan = 3R. The straight part is sqrt(9 - 4) R long, leaving at asin(2 / 3) from psiL.
    turnType, S1, dist, S2 = dubins_path.findDubinsPath(0.0, 0.0, 0.0, 15.0, 10.0, 0.0, R, "pdf")
    assert turnType == "LSR" and abs(dist - math.sqrt(5) * R) < 1e-9 and abs(S1 - R * math.asin(2 / 3)) < 1e-9


def test_pdf_rule_picks_the_word_of_section_0_13():
    R = P["R"]                                                   # = 5 m
    h = math.pi / 2
    for q1, word in [((40, 5, h), "LSL"), ((40, -5, -h), "RSR"), ((40, 5, -h), "LSR"), ((40, -5, h), "RSL"),
                     ((17, 5, h), "LRL"), ((17, -5, -h), "RLR")]:     # the last two: 2R <= S_tan = 12 m < 3R
        assert dubins_path.findDubinsPath(0, 0, 0, *q1, R, "pdf")[0] == word, (q1, word)


def test_pdf_rule_falls_back_when_its_word_does_not_exist():
    q0, q1 = (0, 0, 0), (7.5, 0, math.pi)                        # the rule picks RSL, but S_tan = 1.5 R < 2R
    assert dubins_path.findTurnType(*q0, *q1, P["R"]) == "RSL"
    assert dubins_path.findDubinsPath(*q0, *q1, P["R"], "pdf") == dubins_path.findDubinsPath(*q0, *q1, P["R"], "shortest")


def test_pdf_rule_paths_reach_the_target_and_tables_agree():
    rng = np.random.default_rng(0)
    R = P["R"]
    for _ in range(500):
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        turnType, S1, Smiddle, S2 = dubins_path.findDubinsPath(*q0, *q1, R, "pdf")
        length = S1 + Smiddle + S2
        assert length >= sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:]) - 1e-9
        assert abs(length - dubins_path.returnPathDistances(*q0, *q1, R, "pdf")) < 1e-9
        xt, yt, _, psiT = dubins_path.calculateTrajectory(*q0, R, turnType, S1, Smiddle, S2, 0.25)
        assert np.hypot(xt[-1] - q1[0], yt[-1] - q1[1]) < 1e-9
        assert abs((psiT - q1[2] + math.pi) % (2 * math.pi) - math.pi) < 1e-9
    h0, h1 = rng.uniform(-math.pi, math.pi, 5), rng.uniform(-math.pi, math.pi, 7)
    table = dubins_path.returnPathDistances(0.0, 0.0, h0[:, None], 9.0, 4.0, h1[None, :], R, "pdf")
    for i, j in np.ndindex(table.shape):
        assert abs(table[i, j] - sum(dubins_path.findDubinsPath(0.0, 0.0, h0[i], 9.0, 4.0, h1[j], R, "pdf")[1:])) < 1e-9


def test_compare_counts_fallback_legs_as_missing_not_shortest():
    """The kind comes from the rule's own word, checked here against the table of all six words."""
    R = P["R"]
    assert compare_dubins.classify_leg((0, 0, 0), (7.5, 0, math.pi), R) == "missing"     # RSL with S_tan = 1.5 R
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(500):
        q0 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        q1 = (*rng.uniform(0, 40, 2), rng.uniform(-math.pi, math.pi))
        totals = {t: float(sum(dubins_path.calculateLengths(*q0, *q1, R, t))) for t in dubins_path.TURN_TYPES}
        rule = totals[dubins_path.findTurnType(*q0, *q1, R)]
        expected = ("missing" if not math.isfinite(rule) else
                    "shortest" if rule <= min(totals.values()) + 1e-9 else "longer")
        assert compare_dubins.classify_leg(q0, q1, R) == expected
        seen.add(expected)
    assert seen == set(compare_dubins.LEG_KINDS)


def test_compare_example_legs_show_what_their_titles_say():
    expected = ("shortest", "longer", "longer", "missing")
    legs = compare_dubins.example_legs(P["R"])
    assert [leg["kind"] for leg in legs] == list(expected)
    assert legs[1]["rule"][1] == "S" and legs[2]["rule"][1] != "S" and legs[2]["shortest"]["turnType"][1] == "S"   # CSC, then CCC
    for leg in legs:
        assert leg["feasible"]["length"] >= leg["shortest"]["length"] - 1e-9
        for name in ("shortest", "feasible"):                    # both paths really join the two poses
            assert np.hypot(leg[name]["x"][-1] - leg["q1"][0], leg[name]["y"][-1] - leg["q1"][1]) < 1e-6


def test_compare_distance_bands_add_up():
    for band in compare_dubins.distance_bands(P["R"], n=40):
        assert abs(sum(band[k] for k in compare_dubins.LEG_KINDS) - 1) < 1e-9 and band["extra"] >= 0


# ---------------------------------------------------------------- geodesic distances (eq. 9)

def test_geodesic_distances():
    scen, geo = _map()
    T = np.vstack([scen["base"], scen["xy"]])
    E = np.hypot(*(T[:, None] - T[None]).transpose(2, 0, 1))
    assert np.all(geo["D"] >= E - 1e-9)                          # never shorter than a straight line
    assert np.allclose(geo["D"], geo["D"].T)
    for b in range(1, len(T)):                                   # paths keep r_safe from the obstacles
        pts = geodesic.path(geo, 0, b)
        assert abs(np.sum(np.hypot(*np.diff(pts, axis=0).T)) - geo["D"][0, b]) < 1e-6
        assert trajectory.clearance(pts, geo) >= P["r_safe"]


def test_scenario_puts_sensors_around_obstacles():
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    scen, _ = _map()
    polys = [Polygon(o) for o in scen["obstacles"]]
    d = np.array([unary_union(polys).distance(Point(x)) for x in scen["xy"]])
    inner = np.vstack([c for c in (scenario._inner_corners(q) for q in polys) if len(c)])
    near_corner = [np.min(np.hypot(*(inner - x).T)) <= 8 for x in scen["xy"]]
    assert scen["n_near"] >= 0.4 * len(scen["xy"])               # most of near_share made it
    assert np.sum(d <= P["near_gap"][1]) >= scen["n_near"] and d.min() >= P["near_gap"][0] - 1e-9
    assert sum(near_corner) >= 5                                   # some around concave corners


def test_geodesic_without_obstacles_is_straight():
    scen = {"base": np.array([0.0, 0.0]), "xy": np.array([[3.0, 4.0], [6.0, 8.0]]), "tau": np.ones(2), "obstacles": []}
    geo = geodesic.build(scen, P)
    assert abs(geo["D"][0, 1] - 5) < 1e-12 and abs(geo["D"][1, 2] - 5) < 1e-12


# ---------------------------------------------------------------- GA (0.8)

def test_ga_initial_population_is_radially_unimodal():
    scen, geo = _map()
    r = geo["D"][0, 1:]
    ids = np.arange(len(scen["xy"]))
    pop = ga_tour.initial_population(ids, r, 50, np.random.default_rng(0))
    assert all(ga_tour.peaks(t, r) == 1 for t in pop)            # m(pi) = 1, Section 0.8.2
    assert all(sorted(t) == list(ids) for t in pop)


def test_ga_returns_a_better_permutation():
    scen, geo = _map()
    r, ids = geo["D"][0, 1:], np.arange(40)
    rng = np.random.default_rng(0)
    best = ga_tour.optimize(ids, scen["xy"], scen["base"], r, P, rng)
    start = ga_tour.initial_population(ids, r, 50, np.random.default_rng(0))
    f = lambda t: ga_tour.fitness(t, scen["xy"], scen["base"], r, P["R"])
    assert sorted(best) == list(ids) and f(best) <= min(f(t) for t in start)


def test_peaks_counts_radial_maxima():
    r = np.array([1.0, 5.0, 3.0, 6.0, 2.0])
    assert ga_tour.peaks([0, 1, 2, 3, 4], r) == 2 and ga_tour.peaks([0, 4, 2, 1, 3], r) == 1


def test_ga_history_does_not_change_the_result():
    """Recording the generations (ga_evolution.py) must not change the tour, and what it records must add up."""
    scen, geo = _map()
    xy, base, r = scen["xy"], scen["base"], geo["D"][0, 1:]
    ids = np.arange(30)
    p = default_params(generations=40)
    plain = ga_tour.optimize(ids, xy, base, r, p, np.random.default_rng(3))
    history = []
    logged = ga_tour.optimize(ids, xy, base, r, p, np.random.default_rng(3), history)
    assert logged == plain
    assert len(history) == p["generations"] + 1                   # generation 0 to the final population
    assert all(m == 1 for m in history[0]["peaks"])               # Section 0.8.2: generation 0 is radially unimodal
    best = [g["fitness"][0] for g in history]
    assert all(b1 <= b0 + 1e-9 for b0, b1 in zip(best, best[1:]))   # the better half is kept: the best never rises
    assert history[-1]["orders"][0] == plain                      # the last entry's best is the tour returned
    for g in history:
        assert all(np.diff(g["fitness"]) >= -1e-9)                # best first
        assert np.allclose(g["fitness"], g["length"] + g["peaks"] * np.pi * p["R"])
        assert math.isclose(g["fitness"][0], ga_tour.fitness(g["orders"][0], xy, base, r, p["R"]))
    short = []
    assert ga_tour.optimize(np.arange(2), xy, base, r, p, np.random.default_rng(3), short) == sorted([0, 1], key=lambda i: r[i])
    assert short == []                                            # two sensors: no GA run, nothing recorded


def test_plan_routes_log_changes_nothing_and_follows_the_queue():
    """The GA log of plan_routes (ga_evolution.py, pipeline_gifs.py) must not change the plan, and the
    splitting animation rebuilds the queue of groups from it."""
    import animations
    scen, _ = _map()
    sub = {**scen, "xy": scen["xy"][:25], "tau": scen["tau"][:25]}
    geo = geodesic.build(sub, P)
    plain, _ = planner.plan_routes(sub, geo, P, np.random.default_rng(4))
    log = []
    logged, _ = planner.plan_routes(sub, geo, P, np.random.default_rng(4), ga_log=log)
    assert [t["order"] for t in logged] == [t["order"] for t in plain]
    assert [r["order"] for r in log if r["kept"]] == [t["order"] for t in plain]
    assert any(not r["kept"] for r in log)                       # this map needs at least one split
    for r in log:
        assert np.allclose(r["path"][0], sub["base"]) and np.allclose(r["path"][-1], sub["base"])
        if not r["kept"]:
            halves = np.concatenate(r["halves"]).tolist()
            assert sorted(halves) == sorted(np.asarray(r["ids"]).tolist())
    assert len(animations.split_states(log)) == len(log)         # raises if the log does not follow the queue


def test_animation_conflict_count_matches_summary():
    """The animations count the time in conflict as conflict_game.summary does."""
    import animations
    t = np.arange(0, 21)[:, None]
    trajs = {0: np.hstack([t * 2.0 + 60, 0 * t + 50]), 1: np.hstack([100 - t * 2.0, 0 * t + 51]),
             2: np.hstack([0 * t + 80, t * 2.0])}
    tl = animations.conflict_timeline(trajs, P)
    s = cg.summary(trajs, P)
    assert s["pairs"] and math.isclose(tl["cum"][-1], s["exposure"])
    assert math.isclose(float(np.min(tl["closest"])), s["closest"])


# ---------------------------------------------------------------- partitioning (0.9)

def test_bisection_is_disjoint_and_complete():
    scen, geo = _map()
    ids = np.arange(len(scen["xy"]))
    A, B = partition.bisect(ids, scen["xy"], scen["base"], geo["D"][1:, 1:], geo["D"][0, 1:], scen["tau"], P)
    assert len(A) and len(B) and not set(A) & set(B) and sorted(set(A) | set(B)) == list(ids)


# ---------------------------------------------------------------- trajectories (eq. 17, 19)

def test_trajectory_time_clearance_and_curvature():
    scen, geo = _map()
    rng = np.random.default_rng(0)
    for _ in range(15):
        order = list(rng.choice(len(scen["xy"]), size=rng.integers(2, 10), replace=False))
        tr = trajectory.build(order, geo, scen, P)
        assert abs(tr["T"] - (tr["L"] / P["v"] + scen["tau"][order].sum())) < 1e-9     # eq. (17)
        assert np.allclose(tr["frames"][0], scen["base"]) and np.allclose(tr["frames"][-1], scen["base"])
        assert np.max(np.hypot(*np.diff(tr["frames"], axis=0).T)) <= P["v"] * P["dt"] + 1e-9
        assert trajectory.clearance(tr["path"], geo) >= P["r_safe"]                     # eq. (19)
        assert trajectory.max_curvature(tr["path"]) <= 1.02 / P["R"]                     # Dubins: radius >= R


# ---------------------------------------------------------------- conflict game (0.10)

def _line(a, b, n=40):
    a, b = np.array(a, float), np.array(b, float)
    return np.array([a + (b - a) * i / n for i in range(n + 1)])


def test_game_head_on_and_crossing():
    p = default_params(base=(50.0, -100.0), B=1000.0)
    geo = {"obstacles": None}
    for trajs, expected in [({0: _line((0, 50), (100, 50)), 1: _line((100, 50), (0, 50))}, "DETOUR"),
                            ({0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}, "WAIT")]:
        T = {k: 20.0 for k in trajs}
        new, T2, _, games = cg.resolve(trajs, T, {k: 100.0 for k in trajs}, geo, p)
        assert games[0]["equilibrium"] == {0: expected, 1: "GO"}
        assert cg.summary(new, p)["exposure"] == 0.0 and cg.summary(new, p)["closest"] >= p["d_safe"]


def test_game_is_an_exact_potential_game():
    """Delta J_k = Delta Phi for every single-player change, on random 3-player games."""
    p = default_params(base=(500.0, 500.0), B=1000.0)
    rng = np.random.default_rng(1)
    for _ in range(20):
        trajs = {k: np.cumsum(rng.normal(0, 1.5, (60, 2)), axis=0) + rng.uniform(0, 6, 2) for k in range(4)}
        players = [0, 1, 2]
        acts = {k: [{"name": n, "traj": cg.wait_traj(trajs[k], 5, w) if w else trajs[k], "dT": w * 0.5, "dD": 0.0,
                     "W": w * 0.5} for n, w in (("GO", 0), ("WAIT", 3), ("WAIT", 7))] for k in players}
        E = lambda x, y: cg.exposure(x, y, 5, p)
        c = {k: [a["dT"] + a["W"] for a in acts[k]] for k in players}
        alpha = (1 + sum(max(c[k]) - min(c[k]) for k in players)) / p["dt"]
        J = lambda k, a: alpha * (sum(E(acts[k][a[k]]["traj"], acts[j][a[j]]["traj"]) for j in players if j != k)
                                  + E(acts[k][a[k]]["traj"], trajs[3])) + c[k][a[k]]
        Phi = lambda a: alpha * (sum(E(acts[k][a[k]]["traj"], acts[j][a[j]]["traj"]) for i, k in enumerate(players)
                                     for j in players[i + 1:]) + sum(E(acts[k][a[k]]["traj"], trajs[3]) for k in players)) \
            + sum(c[k][a[k]] for k in players)
        a = {k: int(rng.integers(3)) for k in players}
        for k in players:
            for x in range(3):
                b = {**a, k: x}
                assert abs((J(k, b) - J(k, a)) - (Phi(b) - Phi(a))) < 1e-9
        prof, rec = cg.play(players, acts, {**{k: trajs[k] for k in players}, 3: trajs[3]}, 5, p)
        assert all(rec["potential"][i + 1] < rec["potential"][i] for i in range(len(rec["potential"]) - 1))


def test_game_respects_the_budget():
    p = default_params(base=(50.0, -100.0), B=20.5)              # 0.5 s of slack only
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    _, T2, _, _ = cg.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    assert all(t <= p["B"] + 1e-9 for t in T2.values())


# ---------------------------------------------------------------- simple resolver ("wait your turn")

def test_simple_resolver_crossing_one_sink_waits():
    p = default_params(base=(50.0, -100.0), B=1000.0)
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    T, L = {0: 20.0, 1: 21.0}, {0: 100.0, 1: 100.0}
    new, T2, L2, stops = simple_resolver.resolve(trajs, T, L, {"obstacles": None}, p)
    assert cg.summary(new, p)["exposure"] == 0.0                 # the independent check agrees
    assert [s["sink"] for s in stops] == [0]                     # the longer tour (sink 1) never waits
    assert T2[1] == 21.0 and T2[0] == 20.0 + stops[0]["wait"] and L2 == L
    assert np.array_equal(new[1], trajs[1])


def test_simple_resolver_only_changes_timing():
    p = default_params(base=(50.0, -100.0), B=1000.0)
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    new, _, _, _ = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    for k in trajs:                                              # same path, repeated positions only
        q = new[k][np.concatenate([[True], np.hypot(*np.diff(new[k], axis=0).T) > 1e-9])]
        assert np.array_equal(q, trajs[k])


def test_simple_resolver_respects_the_budget():
    p = default_params(base=(50.0, -100.0), B=20.5)              # 0.5 s of slack only
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((50, 0), (50, 100))}
    _, T2, _, _ = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    assert all(t <= p["B"] + 1e-9 for t in T2.values())


def test_simple_resolver_reports_what_it_cannot_solve():
    """Head-on on one line: waiting cannot help, the other sink drives through. No stops; the conflict stays visible."""
    p = default_params(base=(50.0, -100.0), B=1000.0)
    trajs = {0: _line((0, 50), (100, 50)), 1: _line((100, 50), (0, 50))}
    new, _, _, stops = simple_resolver.resolve(trajs, {0: 20.0, 1: 20.0}, {0: 100.0, 1: 100.0}, {"obstacles": None}, p)
    assert stops == [] and cg.summary(new, p)["exposure"] > 0


def test_end_to_end_with_simple_resolver():
    p = default_params(resolver="simple")
    out = planner.run(1, p)
    c = out["checks"]
    assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], c
    assert c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"], c
    print(f"      seed 1: K={c['K']} T_max={c['T_max']:.1f}s conflict pairs "
          f"{len(out['result']['before']['pairs'])}->{c['conflict_pairs']} closest {c['closest']:.2f}m")


# ---------------------------------------------------------------- whole algorithm (0.11)

def test_end_to_end(seeds=(1, 2, 3)):
    for seed in seeds:
        out = planner.run(seed)
        c = out["checks"]
        assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], (seed, c)
        assert c["clearance"] >= P["r_safe"], (seed, c["clearance"])
        assert c["curvature_x_R"] <= 1 + P["curvature_tol"], (seed, c["curvature_x_R"])
        print(f"      seed {seed}: K={c['K']} T_max={c['T_max']:.1f}s L_total={c['L_total']:.0f}m "
              f"conflict pairs {len(out['result']['before']['pairs'])}->{c['conflict_pairs']} "
              f"closest {c['closest']:.2f}m clearance {c['clearance']:.2f}m")


def test_end_to_end_with_pdf_rule():
    p = default_params(dubins_method="pdf")
    out = planner.run(1, p)
    c = out["checks"]
    assert c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"], c
    assert c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"], c
    print(f"      seed 1: K={c['K']} T_max={c['T_max']:.1f}s L_total={c['L_total']:.0f}m")


def test_same_seed_same_result():
    a, b = planner.run(2), planner.run(2)
    assert [t["order"] for t in a["result"]["tours"]] == [t["order"] for t in b["result"]["tours"]]
    assert a["checks"]["T_max"] == b["checks"]["T_max"]


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for name, f in tests:
        t0 = time.time()
        try:
            f()
            print(f"PASS  {name}  ({time.time() - t0:.1f} s)")
        except Exception:
            failed += 1
            print(f"FAIL  {name}")
            traceback.print_exc()
    print(f"\n{len(tests) - failed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
