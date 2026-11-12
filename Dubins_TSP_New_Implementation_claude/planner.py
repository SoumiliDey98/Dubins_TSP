"""Overall algorithm of Dubins_TSP.pdf, Section 0.11 (Steps 1-12), and the command line.

    python planner.py --seed 1                 # plan, check, print a summary, save output/seed1.png
    python planner.py --seed 1 --budget 100    # another mission budget B
    python planner.py --seed 1 --gif           # also save an animation
    python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the conflict game
    python planner.py --seed 1 --dubins pdf        # turn-type rule of Section 0.13 instead of the shortest of six
"""
import argparse
import os
import time

import numpy as np

import conflict_game
import ga_tour
import geodesic
import partition
import scenario
import simple_resolver
import trajectory
from config import default_params


def plan_routes(scen, geo, p, rng, ga_log=None):
    """Steps 1-7: one sink with all sensors; every group whose tour exceeds B is split
    by the geodesic spectral bisection and both halves are planned again.

    If ga_log is a list, one entry per GA run is appended to it, in the order of the runs:
    {"ids", "generations" (see ga_tour.optimize), "order", "path" (the tour's dense polyline),
    "L", "T", "kept"}, where kept is False for a group that was split; a split group also gets
    "halves", the two groups it was split into. Recording makes no random draws, so the plan
    is the same."""
    xy, base, tau = scen["xy"], scen["base"], scen["tau"]
    Ds, r = geo["D"][1:, 1:], geo["D"][0, 1:]
    todo, tours, problems = [np.arange(len(xy))], [], []          # Step 1
    while todo:
        ids = todo.pop(0)
        history = [] if ga_log is not None else None
        order = ga_tour.optimize(ids, xy, base, r, p, rng, history)   # Step 3: radially unimodal GA
        tour = trajectory.build(order, geo, scen, p)              # Steps 2, 4: Dubins / geodesic T = T(pi*)
        if ga_log is not None:
            ga_log.append({"ids": ids, "generations": history, "order": order, "path": tour["path"],
                           "L": tour["L"], "T": tour["T"], "kept": tour["T"] <= p["B"] or len(ids) == 1})
        if tour["T"] <= p["B"]:
            tours.append(tour)                                    # Steps 5, 7: accept
        elif len(ids) == 1:
            tours.append(tour)
            problems.append(f"sensor {ids[0]} alone needs T = {tour['T']:.1f} s > B")
        else:
            halves = list(partition.bisect(ids, xy, base, Ds, r, tau, p))   # Steps 5-7: split, repeat Step 6
            todo += halves
            if ga_log is not None:
                ga_log[-1]["halves"] = halves
        if len(tours) + len(todo) > p["K_max"]:
            problems.append(f"more than K_max = {p['K_max']} sinks would be needed")
            break
    return tours, problems


def _exposure_with_others(frames, k, tours, p):
    return sum(conflict_game.exposure(frames, t["frames"], 0, p) for j, t in enumerate(tours) if j != k)


def reoptimize(k, tours, scen, geo, p, rng):
    """Step 12 for one sink: try the reversed visiting order and a fresh GA run, and
    keep whichever has the least risk against the other sinks (within the budget)."""
    xy, base, r = scen["xy"], scen["base"], geo["D"][0, 1:]
    best, best_risk = tours[k], _exposure_with_others(tours[k]["frames"], k, tours, p)
    for order in (tours[k]["order"][::-1], ga_tour.optimize(np.array(tours[k]["order"]), xy, base, r, p, rng)):
        cand = trajectory.build(order, geo, scen, p)
        risk = _exposure_with_others(cand["frames"], k, tours, p)
        if cand["T"] <= p["B"] and risk < best_risk:
            best, best_risk = cand, risk
    return best


def resolve_and_check(tours, scen, geo, p, rng):
    """Steps 8-12: detect conflicts, resolve them (conflict game or simple rule, see p["resolver"]),
    verify; if conflicts remain, re-optimise the routes of the sinks involved and repeat (at most
    repair_rounds times). The round with the least remaining risk is kept."""
    resolver = simple_resolver.resolve if p["resolver"] == "simple" else conflict_game.resolve
    best = None
    for rnd in range(p["repair_rounds"] + 1):
        trajs = {k: t["frames"] for k, t in enumerate(tours)}
        before = conflict_game.summary(trajs, p)                                   # Step 8
        new, T, L, games = resolver(trajs, {k: t["T"] for k, t in enumerate(tours)},
                                    {k: t["L"] for k, t in enumerate(tours)}, geo, p)   # Steps 9-10
        after = conflict_game.summary(new, p)                                      # Step 11 (separation)
        result = {"round": rnd, "tours": list(tours), "frames": new, "T": T, "L": L, "games": games,
                  "before": before, "after": after}
        if best is None or after["exposure"] < best["after"]["exposure"]:
            best = result
        if not after["pairs"]:                                                     # Step 12: all clear
            break
        tours = list(tours)
        for k in sorted({k for pair in after["pairs"] for k in pair}):            # Step 12: re-optimise
            tours[k] = reoptimize(k, tours, scen, geo, p, rng)
    return best


def check(result, scen, geo, p, problems):
    """Step 11 and the objectives: every constraint of Section 0.7, measured independently."""
    orders = [t["order"] for t in result["tours"]]
    frames, T, L = result["frames"], result["T"], result["L"]
    K = len(orders)
    return {
        "K": K, "T_max": max(T.values()), "L_total": sum(L.values()),                        # eq. (11)-(14)
        "coverage_ok": sorted(i for o in orders for i in o) == list(range(len(scen["xy"]))),  # eq. (15)-(16)
        "budget_ok": all(t <= p["B"] + 1e-9 for t in T.values()),                            # eq. (18)
        "clearance": min(trajectory.clearance(f, geo) for f in frames.values()),             # eq. (19)
        "curvature_x_R": max(trajectory.max_curvature(f) for f in frames.values()) * p["R"], # Dubins feasibility
        "conflict_pairs": len(result["after"]["pairs"]),                                      # eq. (20)
        "closest": result["after"]["closest"],
        "K_ok": K <= p["K_max"], "problems": problems,
    }


def run(seed, p=None, ga_log=None):
    """Plan one random scenario. Returns {"scenario", "geo", "result", "checks", "times"}.
    ga_log: see plan_routes (used by ga_evolution.py and pipeline_gifs.py; it changes nothing)."""
    p = p or default_params()
    rng = np.random.default_rng(seed)
    times = {}
    t0 = time.time()
    scen = scenario.make(seed, p)
    geo = geodesic.build(scen, p)                                  # Step 2: feasible graph, geodesic costs
    times["graph"] = time.time() - t0
    t0 = time.time()
    tours, problems = plan_routes(scen, geo, p, rng, ga_log)       # Steps 1-7
    times["routes"] = time.time() - t0
    t0 = time.time()
    # Steps 8-12, with their own random stream so that both resolvers start them alike
    result = resolve_and_check(tours, scen, geo, p, np.random.default_rng(seed + 1000))
    times["conflicts"] = time.time() - t0
    checks = check(result, scen, geo, p, problems)
    return {"scenario": scen, "geo": geo, "result": result, "checks": checks, "times": times, "params": p}


def report(out):
    c, res, p = out["checks"], out["result"], out["params"]
    if p["resolver"] == "simple":
        how = f"simple resolver: {len(res['games'])} stops, {sum(s['wait'] for s in res['games']):.1f} s of waiting in total"
    else:
        eq = {}
        for g in res["games"]:
            for name in g["equilibrium"].values():
                eq[name] = eq.get(name, 0) + 1
        how = f"conflict games: {len(res['games'])}, choices {eq}"
    lines = [
        f"sensors {len(out['scenario']['xy'])}, obstacles {len(out['scenario']['obstacles'])}, budget B = {p['B']} s,"
        f" Dubins method: {p['dubins_method']}",
        f"objectives:  K = {c['K']} sinks,  T_max = {c['T_max']:.1f} s,  L_total = {c['L_total']:.0f} m",
        f"coverage (eq. 15-16): {'ok' if c['coverage_ok'] else 'FAILED'}   budget (eq. 18): {'ok' if c['budget_ok'] else 'FAILED'}"
        f"   K <= K_max: {'ok' if c['K_ok'] else 'FAILED'}",
        f"obstacle clearance (eq. 19): {c['clearance']:.2f} m (needs >= r_safe = {p['r_safe']})"
        f"   max curvature x R: {c['curvature_x_R']:.3f} (needs <= {1 + p['curvature_tol']:.2f})",
        f"inter-sink conflicts (eq. 20): {len(res['before']['pairs'])} pairs / {res['before']['exposure']:.1f} s before resolving"
        f" -> {c['conflict_pairs']} pairs / {res['after']['exposure']:.1f} s after (closest {c['closest']:.2f} m,"
        f" d_safe = {p['d_safe']})",
        f"{how}, kept repair round {res['round']}",
        "time: " + ", ".join(f"{k} {v:.1f} s" for k, v in out["times"].items()),
    ]
    lines += [f"problem: {x}" for x in c["problems"]]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]")
    ap.add_argument("--gif", action="store_true", help="also save an animation (slow)")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--resolver", choices=["game", "simple"], default="game", help="how conflicts between sinks are resolved")
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    args = ap.parse_args()
    changes = {} if args.budget is None else {"B": args.budget}
    p = default_params(resolver=args.resolver, dubins_method=args.dubins, **changes)
    out = run(args.seed, p)
    print(report(out))
    if not args.no_plot:
        import plot
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
        os.makedirs(folder, exist_ok=True)
        name = f"seed{args.seed}" + ("_simple" if args.resolver == "simple" else "") + ("_pdf" if args.dubins == "pdf" else "")
        path = os.path.join(folder, f"{name}.png")
        plot.routes(out, path)
        print(f"figure: {path}")
        if args.gif:
            path = os.path.join(folder, f"{name}.gif")
            plot.animation(out, path)
            print(f"animation: {path}")


if __name__ == "__main__":
    main()
