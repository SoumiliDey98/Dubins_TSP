# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# This is the conductor of the orchestra: it calls all the other files in the
# right order (Steps 1-12 of Section 0.11 of the paper):
#   - make a map (scenario.py) and the distances round obstacles (geodesic.py);
#   - give all sensors to one robot; find a visiting order (ga_tour.py) and the
#     drivable path and time (trajectory.py); if it takes longer than the budget
#     B, cut the group in two (partition.py) and try again with each half;
#   - when every robot fits the budget, settle the conflicts between robots
#     (conflict_game.py, or simple_resolver.py), repairing routes if needed;
#   - check every rule of the paper and print a report.
# It can also be started from the command line, e.g. "python planner.py --seed 1".
# ==============================================================================
"""Overall algorithm of Dubins_TSP.pdf, Section 0.11 (Steps 1-12), and the command line.

    python planner.py --seed 1                 # plan, check, print a summary, save output/seed1.png
    python planner.py --seed 1 --budget 100    # another mission budget B
    python planner.py --seed 1 --gif           # also save an animation
    python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the conflict game
    python planner.py --seed 1 --dubins pdf        # turn-type rule of Section 0.13 instead of the shortest of six
"""
# "import" borrows a toolbox. argparse reads the options typed on the command
# line (like --seed 1).
import argparse
# os: for folders and file names.
import os
# time: a stopwatch, to measure how long each part takes.
import time

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np

# The other files of this project, each doing one job.
# conflict_game.py: finds conflicts between robots and settles them with a game.
import conflict_game
# ga_tour.py: the genetic algorithm that finds visiting orders.
import ga_tour
# geodesic.py: the shortest distances and ways round obstacles.
import geodesic
# partition.py: cuts a group of sensors in two.
import partition
# scenario.py: makes the map.
import scenario
# simple_resolver.py: the "wait your turn" rule.
import simple_resolver
# trajectory.py: the drivable paths and their times.
import trajectory
# From config.py, only the recipe that makes the settings.
from config import default_params


# "def" starts a recipe (a "function"). Steps 1-7: decide how many robots are
# needed and which sensors each one visits. Inputs: scen, the map; geo, the
# distances round obstacles; p, the settings; rng, the random-number maker;
# ga_log, an optional list in which every GA run is written down.
def plan_routes(scen, geo, p, rng, ga_log=None):
    """Steps 1-7: one sink with all sensors; every group whose tour exceeds B is split
    by the geodesic spectral bisection and both halves are planned again.

    If ga_log is a list, one entry per GA run is appended to it, in the order of the runs:
    {"ids", "generations" (see ga_tour.optimize), "order", "path" (the tour's dense polyline),
    "L", "T", "kept"}, where kept is False for a group that was split; a split group also gets
    "halves", the two groups it was split into. Recording makes no random draws, so the plan
    is the same."""
    # Short names for the sensor positions, the base, and the waiting times.
    xy, base, tau = scen["xy"], scen["base"], scen["tau"]
    # Ds: sensor-to-sensor distances round obstacles; r: each sensor's distance
    # from the base. (Row and column 0 of D belong to the base.)
    Ds, r = geo["D"][1:, 1:], geo["D"][0, 1:]
    # Step 1: "todo" is a queue (a waiting line) of sensor groups still to plan.
    # It starts with ONE group holding every sensor (np.arange(n) is 0, 1, ..., n-1).
    # tours: the finished robots' plans; problems: anything that went wrong.
    todo, tours, problems = [np.arange(len(xy))], [], []          # Step 1
    # Keep going while the queue is not empty.
    while todo:
        # Take the first group out of the queue (first in, first out).
        ids = todo.pop(0)
        # If we are writing a log, prepare a list for this GA run's generations;
        # otherwise None ("nothing").
        history = [] if ga_log is not None else None
        # Step 3: the genetic algorithm finds a good visiting order.
        order = ga_tour.optimize(ids, xy, base, r, p, rng, history)   # Step 3: radially unimodal GA
        # Steps 2 and 4: turn the order into a drivable path with its time T.
        tour = trajectory.build(order, geo, scen, p)              # Steps 2, 4: Dubins / geodesic T = T(pi*)
        # Write this run into the log, if there is one. "kept" says whether the
        # group stays as one robot (T fits the budget, or it is a single sensor).
        if ga_log is not None:
            # Add one entry to the log.
            ga_log.append({"ids": ids, "generations": history, "order": order, "path": tour["path"],
                           "L": tour["L"], "T": tour["T"], "kept": tour["T"] <= p["B"] or len(ids) == 1})
        # Steps 5 and 7: within the budget B? Then this group is one robot. Done.
        if tour["T"] <= p["B"]:
            tours.append(tour)                                    # Steps 5, 7: accept
        # A single sensor that still takes too long cannot be split: keep it,
        # but note the problem. (The f"..." text fills in values inside { }.)
        elif len(ids) == 1:
            # Keep it as a robot anyway ...
            tours.append(tour)
            # ... and note the problem.
            problems.append(f"sensor {ids[0]} alone needs T = {tour['T']:.1f} s > B")
        # Too long: cut the group in two (partition.py) and put both halves at
        # the back of the queue, to be planned again (Step 6).
        else:
            halves = list(partition.bisect(ids, xy, base, Ds, r, tau, p))   # Steps 5-7: split, repeat Step 6
            # Add both halves to the back of the queue.
            todo += halves
            # Note in the log which two groups this one was split into.
            if ga_log is not None:
                # ga_log[-1] is the last entry, this run.
                ga_log[-1]["halves"] = halves
        # If finished robots plus waiting groups would be more than K_max (30)
        # robots, give up and note the problem. "break" leaves the loop.
        if len(tours) + len(todo) > p["K_max"]:
            # Note the problem ...
            problems.append(f"more than K_max = {p['K_max']} sinks would be needed")
            # ... and stop planning.
            break
    # Hand back the robots' plans and the problems.
    return tours, problems


# The "_" at the start of the name means "only used inside this file".
# How many seconds robot k (with these frames) would spend too close to all
# the OTHER robots.
def _exposure_with_others(frames, k, tours, p):
    # Add up the danger against every other robot j, from frame 0 on.
    return sum(conflict_game.exposure(frames, t["frames"], 0, p) for j, t in enumerate(tours) if j != k)


# Step 12 for one robot k: try two other routes and keep the safest.
def reoptimize(k, tours, scen, geo, p, rng):
    """Step 12 for one sink: try the reversed visiting order and a fresh GA run, and
    keep whichever has the least risk against the other sinks (within the budget)."""
    # Short names for the positions, the base, and the distances from the base.
    xy, base, r = scen["xy"], scen["base"], geo["D"][0, 1:]
    # The best so far: the current route, with its danger.
    best, best_risk = tours[k], _exposure_with_others(tours[k]["frames"], k, tours, p)
    # Two other orders to try: the same order backwards ([::-1]), and a fresh
    # run of the genetic algorithm on the same sensors.
    for order in (tours[k]["order"][::-1], ga_tour.optimize(np.array(tours[k]["order"]), xy, base, r, p, rng)):
        # Its drivable path and time.
        cand = trajectory.build(order, geo, scen, p)
        # Its danger against the other robots.
        risk = _exposure_with_others(cand["frames"], k, tours, p)
        # Keep it if it fits the budget and is less dangerous.
        if cand["T"] <= p["B"] and risk < best_risk:
            # This is the new best.
            best, best_risk = cand, risk
    # Hand back the safest route.
    return best


# Steps 8-12: settle the conflicts between robots, and repair routes if some remain.
def resolve_and_check(tours, scen, geo, p, rng):
    """Steps 8-12: detect conflicts, resolve them (conflict game or simple rule, see p["resolver"]),
    verify; if conflicts remain, re-optimise the routes of the sinks involved and repeat (at most
    repair_rounds times). The round with the least remaining risk is kept."""
    # Which settler to use: the simple "wait your turn" rule, or the game.
    resolver = simple_resolver.resolve if p["resolver"] == "simple" else conflict_game.resolve
    # The best round so far (none yet).
    best = None
    # Rounds 0, 1, 2, 3: the first try and up to 3 repairs.
    for rnd in range(p["repair_rounds"] + 1):
        # Every robot's frames, numbered 0, 1, 2 ... ("enumerate" counts them).
        trajs = {k: t["frames"] for k, t in enumerate(tours)}
        # Step 8: the conflicts before settling.
        before = conflict_game.summary(trajs, p)                                   # Step 8
        # Steps 9-10: settle them. Hand in frames, times T and lengths L; get
        # back the new ones and a record of every game (or stop).
        new, T, L, games = resolver(trajs, {k: t["T"] for k, t in enumerate(tours)},
                                    {k: t["L"] for k, t in enumerate(tours)}, geo, p)   # Steps 9-10
        # Step 11: the conflicts left after settling.
        after = conflict_game.summary(new, p)                                      # Step 11 (separation)
        # Everything about this round, in one dictionary.
        result = {"round": rnd, "tours": list(tours), "frames": new, "T": T, "L": L, "games": games,
                  "before": before, "after": after}
        # Keep the round with the fewest seconds of conflict left.
        if best is None or after["exposure"] < best["after"]["exposure"]:
            # This round is the best so far.
            best = result
        # Step 12: no conflicts left? Then stop.
        if not after["pairs"]:                                                     # Step 12: all clear
            # Leave the rounds loop.
            break
        # Otherwise make a copy of the route list ...
        tours = list(tours)
        # ... and re-plan every robot that is still in a conflict (the set
        # { } lists each robot once).
        for k in sorted({k for pair in after["pairs"] for k in pair}):            # Step 12: re-optimise
            # Try other routes for robot k and keep the safest.
            tours[k] = reoptimize(k, tours, scen, geo, p, rng)
    # Hand back the best round.
    return best


# Step 11 and the scores: measures every rule of Section 0.7 on the final result.
def check(result, scen, geo, p, problems):
    """Step 11 and the objectives: every constraint of Section 0.7, measured independently."""
    # Every robot's visiting order.
    orders = [t["order"] for t in result["tours"]]
    # The final frames, times and lengths (after settling the conflicts).
    frames, T, L = result["frames"], result["T"], result["L"]
    # K: the number of robots.
    K = len(orders)
    # Hand back a dictionary of measurements:
    return {
        # The three scores (equations 11-14): number of robots K, the longest
        # mission time T_max, and the total distance driven L_total.
        "K": K, "T_max": max(T.values()), "L_total": sum(L.values()),                        # eq. (11)-(14)
        # Every sensor visited exactly once (equations 15-16): all the orders
        # together, sorted, must be exactly 0, 1, 2, ..., n-1.
        "coverage_ok": sorted(i for o in orders for i in o) == list(range(len(scen["xy"]))),  # eq. (15)-(16)
        # Every robot back within the budget B (equation 18).
        "budget_ok": all(t <= p["B"] + 1e-9 for t in T.values()),                            # eq. (18)
        # The smallest distance of any robot to any obstacle (equation 19 needs >= r_safe).
        "clearance": min(trajectory.clearance(f, geo) for f in frames.values()),             # eq. (19)
        # The sharpest bend of any robot, times R (must be about 1 or less).
        "curvature_x_R": max(trajectory.max_curvature(f) for f in frames.values()) * p["R"], # Dubins feasibility
        # How many pairs of robots are still in conflict (equation 20).
        "conflict_pairs": len(result["after"]["pairs"]),                                      # eq. (20)
        # The closest any two robots come.
        "closest": result["after"]["closest"],
        # No more robots than allowed, and the list of problems.
        "K_ok": K <= p["K_max"], "problems": problems,
    }


# The whole planner for one map. seed: which map; p: the settings (the normal
# ones if none are given); ga_log: as in plan_routes.
def run(seed, p=None, ga_log=None):
    """Plan one random scenario. Returns {"scenario", "geo", "result", "checks", "times"}.
    ga_log: see plan_routes (used by ga_evolution.py and pipeline_gifs.py; it changes nothing)."""
    # Use the settings handed in, or else ("or") the normal ones.
    p = p or default_params()
    # The random-number maker for Steps 1-7, started from the seed.
    rng = np.random.default_rng(seed)
    # times: how long each part took; t0: start the stopwatch.
    times = {}
    # Start the stopwatch (time.time() is the clock, in seconds).
    t0 = time.time()
    # Make the map.
    scen = scenario.make(seed, p)
    # Step 2: the distances round obstacles.
    geo = geodesic.build(scen, p)                                  # Step 2: feasible graph, geodesic costs
    # Note the time, and restart the stopwatch.
    times["graph"] = time.time() - t0
    # Restart the stopwatch.
    t0 = time.time()
    # Steps 1-7: the robots and their routes.
    tours, problems = plan_routes(scen, geo, p, rng, ga_log)       # Steps 1-7
    # Note how long the routes took ...
    times["routes"] = time.time() - t0
    # ... and restart the stopwatch.
    t0 = time.time()
    # Steps 8-12, with their own random stream so that both resolvers start them alike
    # (a second random-number maker, started from seed + 1000).
    result = resolve_and_check(tours, scen, geo, p, np.random.default_rng(seed + 1000))
    # Note how long settling the conflicts took.
    times["conflicts"] = time.time() - t0
    # Measure every rule.
    checks = check(result, scen, geo, p, problems)
    # Hand back everything in one dictionary.
    return {"scenario": scen, "geo": geo, "result": result, "checks": checks, "times": times, "params": p}


# Writes the report that is printed on the screen.
def report(out):
    # Short names for the checks, the result and the settings.
    c, res, p = out["checks"], out["result"], out["params"]
    # With the simple rule: how many stops, and how many seconds of waiting.
    if p["resolver"] == "simple":
        # The text for the report.
        how = f"simple resolver: {len(res['games'])} stops, {sum(s['wait'] for s in res['games']):.1f} s of waiting in total"
    # With the game: how many games, and how often each strategy was chosen.
    else:
        # eq counts how often each strategy was chosen; empty at first.
        eq = {}
        # For every game ...
        for g in res["games"]:
            # ... and every player's final choice in it:
            for name in g["equilibrium"].values():
                # Count one more for this strategy (.get gives 0 if not counted yet).
                eq[name] = eq.get(name, 0) + 1
        # The text for the report.
        how = f"conflict games: {len(res['games'])}, choices {eq}"
    # The lines of the report. Inside f"...", {x:.1f} writes x with 1 digit
    # after the point, {x:.0f} with none. "a if q else b" picks a or b.
    lines = [
        # The map and the main settings.
        f"sensors {len(out['scenario']['xy'])}, obstacles {len(out['scenario']['obstacles'])}, budget B = {p['B']} s,"
        f" Dubins method: {p['dubins_method']}",
        # The three scores.
        f"objectives:  K = {c['K']} sinks,  T_max = {c['T_max']:.1f} s,  L_total = {c['L_total']:.0f} m",
        # Were all sensors visited, the budget kept, and not too many robots used?
        f"coverage (eq. 15-16): {'ok' if c['coverage_ok'] else 'FAILED'}   budget (eq. 18): {'ok' if c['budget_ok'] else 'FAILED'}"
        f"   K <= K_max: {'ok' if c['K_ok'] else 'FAILED'}",
        # Distance to obstacles, and the sharpest bend.
        f"obstacle clearance (eq. 19): {c['clearance']:.2f} m (needs >= r_safe = {p['r_safe']})"
        f"   max curvature x R: {c['curvature_x_R']:.3f} (needs <= {1 + p['curvature_tol']:.2f})",
        # Conflicts between robots, before and after settling.
        f"inter-sink conflicts (eq. 20): {len(res['before']['pairs'])} pairs / {res['before']['exposure']:.1f} s before resolving"
        f" -> {c['conflict_pairs']} pairs / {res['after']['exposure']:.1f} s after (closest {c['closest']:.2f} m,"
        f" d_safe = {p['d_safe']})",
        # How they were settled, and which repair round was kept.
        f"{how}, kept repair round {res['round']}",
        # How long each part took.
        "time: " + ", ".join(f"{k} {v:.1f} s" for k, v in out["times"].items()),
    ]
    # One more line for every problem.
    lines += [f"problem: {x}" for x in c["problems"]]
    # Join the lines with line breaks ("\n") and hand back the text.
    return "\n".join(lines)


# What happens when the file is started from the command line.
def main():
    # Describe the options the user may type.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --seed: which map (a whole number, normally 1).
    ap.add_argument("--seed", type=int, default=1)
    # --budget: another time budget B in seconds.
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]")
    # --gif: also make an animation. "store_true" means the option needs no value.
    ap.add_argument("--gif", action="store_true", help="also save an animation (slow)")
    # --no-plot: do not draw the picture.
    ap.add_argument("--no-plot", action="store_true")
    # --resolver: "game" or "simple".
    ap.add_argument("--resolver", choices=["game", "simple"], default="game", help="how conflicts between sinks are resolved")
    # --dubins: "shortest" or "pdf".
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    # Read what the user typed.
    args = ap.parse_args()
    # Settings to change: the budget, if one was given.
    changes = {} if args.budget is None else {"B": args.budget}
    # Make the settings with those changes. (**changes hands the dictionary in
    # as name=value pairs.)
    p = default_params(resolver=args.resolver, dubins_method=args.dubins, **changes)
    # Run the planner and print the report.
    out = run(args.seed, p)
    # Print the report.
    print(report(out))
    # Unless --no-plot was given, draw the picture.
    if not args.no_plot:
        # plot.py draws pictures; it is borrowed only when needed.
        import plot
        # The "output" folder next to this file; make it if it is missing.
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
        # Make the folder if it is missing (exist_ok=True: no error if it is there).
        os.makedirs(folder, exist_ok=True)
        # The file name, e.g. "seed1", with "_simple" or "_pdf" added for those options.
        name = f"seed{args.seed}" + ("_simple" if args.resolver == "simple" else "") + ("_pdf" if args.dubins == "pdf" else "")
        # Save the picture of the routes.
        path = os.path.join(folder, f"{name}.png")
        # Draw the routes and save the picture.
        plot.routes(out, path)
        # Say where it was saved.
        print(f"figure: {path}")
        # With --gif, also save an animation.
        if args.gif:
            # The animation's file name.
            path = os.path.join(folder, f"{name}.gif")
            # Make and save the animation.
            plot.animation(out, path)
            # Say where it was saved.
            print(f"animation: {path}")


# This is true only when the file is started directly ("python planner.py"),
# not when another file borrows it with "import planner". Then run main().
if __name__ == "__main__":
    # Run the main recipe.
    main()
