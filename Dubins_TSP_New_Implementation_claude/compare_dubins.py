# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# There are two ways to choose the shape (the "word", like LSL or RSR) of each
# curved Dubins path: "shortest" tries all six shapes and keeps the shortest;
# "pdf" uses the rule written in Section 0.13 of the paper. This file plans
# the same maps both ways and prints tables comparing them: how many robots,
# how far they drive, how often the paper's rule picks the shortest shape.
# It can also save five comparison pictures.
# ==============================================================================
"""Compare the two ways of choosing a Dubins word (p["dubins_method"]) on the same maps:
"shortest" (the true shortest path) and "pdf" (the rule of Section 0.13, which gives a feasible path).

Prints the numbers of the README section "Shortest of six or the rule of Section 0.13?":
  1. Each method plans its own routes on every seed; the checks of planner.py score them.
  2. Same visiting orders: the tours of the default plan are rebuilt with the rule. This
     separates the rule from differences in GA runs and splits.
  3. Legs: how the rule does on the legs of each final plan (see classify_leg).
With --figures it also saves five figures to output/ and prints the table behind one of them.

    python compare_dubins.py                       # seeds 1-10, one job per core
    python compare_dubins.py --seeds 1 2 --jobs 1
    python compare_dubins.py --figures             # also output/compare_*.png
"""
# "import" borrows a toolbox. argparse reads options typed on the command line.
import argparse
# math: maths for single numbers (pi, cos, radians ...).
import math
# multiprocessing: runs several jobs at the same time on the computer's cores.
import multiprocessing
# os: folders and file names.
import os
# sys: talks to the Python system itself; used here to print progress messages.
import sys

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np

# Files of this project: the Dubins paths, the whole planner, the trajectories.
# dubins_path.py: the six Dubins shapes.
import dubins_path
# planner.py: the whole planner.
import planner
# trajectory.py: drivable paths and their times.
import trajectory
# The recipe that makes the settings.
from config import default_params

# The two ways of choosing a shape.
METHODS = ("shortest", "pdf")
# The three kinds of leg, as seen by the paper's rule: its shape is the
# shortest, or it exists but is longer, or it does not exist ("missing").
LEG_KINDS = ("shortest", "longer", "missing")
# Example legs for the first figure: (title, start, end), with (x [m], y [m], heading [degrees]) for R = 5 m.
# Four hand-picked legs, each showing one thing that can happen.
EXAMPLE_LEGS = (("The rule picks the shortest path", (0, 0, 90), (30, 5, -90)),
                ("The rule picks a longer CSC path", (0, 0, 0), (15, -5, 180)),
                ("The rule picks CCC, but CSC is shorter", (0, 0, 90), (5, -10, -90)),
                ("The rule's {rule} does not exist: fallback used", (0, 0, 0), (5, -5, 0)))
# Groups of distances between the two points of a leg, measured in turning
# radii R: 0 to 2R, 2R to 4R, 4R to 8R, and 8R to 20R.
DISTANCE_BANDS = ((0, 2), (2, 4), (4, 8), (8, 20))      # distance between the two points, in units of R


# "def" starts a recipe (a "function"). This one sorts one leg (from pose q0 to
# pose q1) into one of the three kinds.
def classify_leg(q0, q1, R):
    """How the rule of Section 0.13 does on one leg: its word is the "shortest" one,
    exists but is "longer", or is "missing" (it does not exist, so the fallback is used)."""
    # The shape the "pdf" method ends up using, and its lengths. The "*" in
    # front of lengths collects all the remaining answers into one list.
    turnType, *lengths = dubins_path.findDubinsPath(*q0, *q1, R, "pdf")
    # If that is not the shape the rule asked for, the rule's shape did not
    # exist and the fallback was used: "missing".
    if turnType != dubins_path.findTurnType(*q0, *q1, R):
        # Hand back "missing".
        return "missing"
    # The length of the truly shortest shape ([1:] skips the name, keeping the
    # three lengths, which sum adds up).
    shortest = sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:])
    # "shortest" if the rule's shape is no longer (allowing a tiny rounding
    # error), otherwise "longer".
    return "shortest" if sum(lengths) <= shortest + 1e-9 else "longer"


# Counts the legs of each kind in a set of robot tours.
def leg_counts(tours, geo, p):
    """Number of legs of each kind in these tours. Waypoints and headings are recomputed
    the way trajectory.build does; both are deterministic, so these are the legs it drove."""
    # A dictionary with every kind counted as 0 to start with.
    counts = dict.fromkeys(LEG_KINDS, 0)
    # For every robot's tour:
    for tour in tours:
        # Its waypoints and headings, worked out exactly as trajectory.build does.
        pts, _ = trajectory.waypoints(tour["order"], geo, p)
        # The heading chosen at each waypoint.
        h = trajectory.choose_headings(pts, geo, p)
        # Sort every leg and count one more of its kind ("+= 1" adds one).
        for k in range(len(pts) - 1):
            # Sort this leg and count one more of its kind ("+= 1" adds one).
            counts[classify_leg((*pts[k], h[k]), (*pts[k + 1], h[k + 1]), p["R"])] += 1
    # Hand back the counts.
    return counts


# Plans one map with one method and collects its numbers into a "row".
def run_one(seed, method):
    """Plan one map with one method and return its numbers."""
    # The settings, with this method of choosing shapes.
    p = default_params(dubins_method=method)
    # Run the whole planner.
    out = planner.run(seed, p)
    # Short names for the checks and the result.
    c, res = out["checks"], out["result"]
    # The row: the map, the method, the scores and the checks.
    row = {"seed": seed, "method": method, "K": c["K"], "T_max": c["T_max"], "L": c["L_total"],
           "clearance": c["clearance"], "curvature": c["curvature_x_R"], "pairs": c["conflict_pairs"],
           "risk": res["after"]["exposure"], "closest": c["closest"],
           # "ok": did every check pass, except the one between robots (eq. 20)?
           "ok": (c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"]    # all but eq. (20)
                  and c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"]),
           # And how many legs of each kind.
           "legs": leg_counts(res["tours"], out["geo"], p)}
    # For the "shortest" plan only: rebuild the very same visiting orders with
    # the paper's rule, to see what the rule alone changes.
    if method == "shortest":
        # The settings with the paper's rule.
        rule = default_params(dubins_method="pdf")
        # Every robot's tour rebuilt with the rule.
        rebuilt = [trajectory.build(t["order"], out["geo"], out["scenario"], rule) for t in res["tours"]]
        # Total length both ways, and how many rebuilt tours would break the budget B.
        row["same_orders"] = {"L_shortest": sum(t["L"] for t in res["tours"]), "L_rule": sum(t["L"] for t in rebuilt),
                              "over_B": sum(t["T"] > p["B"] + 1e-9 for t in rebuilt)}
    # Print a progress message to the "error" channel (so it does not mix with
    # the tables). flush=True prints it at once.
    print(f"seed {seed}, {method}: done", file=sys.stderr, flush=True)
    # Hand back the row.
    return row


# Turns all the rows into tables written in "Markdown" (a simple text format
# where | separates columns), as in the README.
def report(rows):
    """The comparison as Markdown tables. rows must hold both methods for every seed, in the same seed order."""
    # The rows of each method, kept apart.
    plans = {m: [r for r in rows if r["method"] == m] for m in METHODS}

    # A recipe inside a recipe: one table line comparing the two methods.
    # "value" is itself a recipe that turns a method's rows into one number or text.
    def compare(label, value):
        # The label, then the value for each method, between | signs.
        return f"| {label} | {value(plans['shortest'])} | {value(plans['pdf'])} |"

    # Makes a "value" recipe for one kind of leg: "n of total (percent)".
    def legs(kind):
        # The recipe it makes: give it a method's rows, get back the text.
        def value(rs):
            # n: legs of this kind; total: all legs.
            n, total = sum(r["legs"][kind] for r in rs), sum(sum(r["legs"].values()) for r in rs)
            # For example "120 of 300 (40.0 %)".
            return f"{n} of {total} ({100 * n / total:.1f} %)"
        # Hand back that recipe.
        return value

    # The first table: one line per map and method. Inside f"...", {x:.1f}
    # writes x with one digit after the point.
    lines = ["Per map", "",
             "| Seed | Method | K | T_max (s) | L_total (m) | Clearance (m) | Curvature x R "
             "| Pairs left | Risk left (s) | Closest (m) | Checks except eq. 20 |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    # One table line per row (per map and method).
    lines += [f"| {r['seed']} | {r['method']} | {r['K']} | {r['T_max']:.1f} | {r['L']:.0f} | {r['clearance']:.2f} "
              f"| {r['curvature']:.3f} | {r['pairs']} | {r['risk']:.1f} | {r['closest']:.2f} "
              f"| {'ok' if r['ok'] else 'FAILED'} |" for r in rows]

    # The second table: totals over all maps. Each "lambda rs: ..." is a tiny
    # nameless recipe that adds up (sum), or finds the smallest (min) or largest
    # (max), of one number over a method's rows.
    lines += ["", "Totals", "", "| | Shortest of six | Rule of Section 0.13 (feasible) |", "|---|---|---|",
              compare("Sinks K", lambda rs: sum(r["K"] for r in rs)),
              compare("L_total", lambda rs: f"{sum(r['L'] for r in rs):.0f} m"),
              compare("Conflicting pairs left", lambda rs: sum(r["pairs"] for r in rs)),
              compare("Risk left", lambda rs: f"{sum(r['risk'] for r in rs):.1f} s"),
              compare("Maps with no conflict", lambda rs: f"{sum(r['pairs'] == 0 for r in rs)} of {len(rs)}"),
              compare("Worst closest approach", lambda rs: f"{min(r['closest'] for r in rs):.2f} m"),
              compare("Lowest obstacle clearance", lambda rs: f"{min(r['clearance'] for r in rs):.2f} m"),
              compare("Highest curvature x R", lambda rs: f"{max(r['curvature'] for r in rs):.3f}"),
              compare("Maps passing every check except eq. 20", lambda rs: f"{sum(r['ok'] for r in rs)} of {len(rs)}")]
    # The total length with each method ...
    a, b = sum(r["L"] for r in plans["shortest"]), sum(r["L"] for r in plans["pdf"])
    # ... and on how many maps the rule's plan is longer ("zip" pairs the maps up).
    longer = sum(y["L"] > x["L"] for x, y in zip(plans["shortest"], plans["pdf"]))
    # How much longer in percent: (b / a - 1) * 100. "+.1f" always shows the sign.
    lines += ["", f"L_total with the rule: {100 * (b / a - 1):+.1f} %, longer on {longer} of {len(plans['pdf'])} maps."]

    # The "same visiting orders" comparison.
    same = [r["same_orders"] for r in plans["shortest"]]
    # The total length before and after rebuilding with the rule.
    a, b = sum(s["L_shortest"] for s in same), sum(s["L_rule"] for s in same)
    # One sentence comparing them.
    lines += ["", f"Same visiting orders: the {sum(r['K'] for r in plans['shortest'])} tours of the default plan rebuilt "
                  f"with the rule: {a:.0f} m -> {b:.0f} m ({100 * (b / a - 1):+.1f} %), "
                  f"tours over B: {sum(s['over_B'] for s in same)}."]

    # The third table: the kinds of leg in each plan.
    lines += ["", "How the rule of Section 0.13 does on the legs of each final plan", "",
              "| | Default plan | Plan made with the rule |", "|---|---|---|",
              compare("Rule's word is the shortest", legs("shortest")),
              compare("Rule's word exists but is longer", legs("longer")),
              compare("Rule's word does not exist (fallback)", legs("missing"))]
    # Join all lines with line breaks and hand back the text.
    return "\n".join(lines)


# The four example legs, each with both paths, ready to draw.
def example_legs(R):
    """The legs of EXAMPLE_LEGS, each with its kind and both paths: {"title", "q0", "q1", "kind", "shortest", "feasible"}."""
    # The list of legs, empty at first.
    legs = []
    # For each example: its title, its start and its end.
    for title, a, b in EXAMPLE_LEGS:
        # The two poses, with the headings turned from degrees into radians.
        q0, q1 = (a[0], a[1], math.radians(a[2])), (b[0], b[1], math.radians(b[2]))
        # The shape the paper's rule picks.
        rule = dubins_path.findTurnType(*q0, *q1, R)
        # The leg's facts. title.format(rule=rule) fills the {rule} gap in the title.
        leg = dict(title=title.format(rule=rule), q0=q0, q1=q1, kind=classify_leg(q0, q1, R), rule=rule)
        # Both paths: the shortest, and the "feasible" one from the rule, with
        # points 0.05 m apart for smooth drawing.
        for name, method in (("shortest", "shortest"), ("feasible", "pdf")):
            # The path's points and length ("_" drops the distances along it).
            x, y, length, _ = dubins_path.returnDubinsPath(*q0, *q1, R, method, 0.05)
            # Keep the points, the length and the shape's name.
            leg[name] = dict(x=x, y=y, length=length, turnType=dubins_path.findDubinsPath(*q0, *q1, R, method)[0])
        # Add the leg to the list.
        legs.append(leg)
    # Hand back the four legs.
    return legs


# Tests the rule on n = 5000 random legs in each distance band.
def distance_bands(R, n=5000, seed=0):
    """How the rule does on n random poses in each band of DISTANCE_BANDS: the share of each kind of
    leg, and the mean extra length of the feasible path. These poses are random, so this is not the
    planner's cost: the heading search avoids many of the poses where the rule does badly."""
    # A seeded random-number maker, and an empty list of results.
    rng, bands = np.random.default_rng(seed), []
    # For each band, from lo to hi turning radii:
    for lo, hi in DISTANCE_BANDS:
        # Counts start at 0, and so does the total extra length.
        counts, extra = dict.fromkeys(LEG_KINDS, 0), 0.0
        # n random legs:
        for _ in range(n):
            # A random distance in the band, and a random direction.
            d, angle = rng.uniform(lo, hi) * R, rng.uniform(0, 2 * math.pi)
            # Start at (0, 0), end d metres away in that direction; both headings random.
            q0, q1 = (0.0, 0.0, rng.uniform(0, 2 * math.pi)), (d * math.cos(angle), d * math.sin(angle), rng.uniform(0, 2 * math.pi))
            # Count its kind.
            counts[classify_leg(q0, q1, R)] += 1
            # How much longer the rule's path is, as a fraction of the shortest.
            shortest = sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:])
            # Add how much longer the rule's path is, as a fraction (0.1 is 10 % longer).
            extra += sum(dubins_path.findDubinsPath(*q0, *q1, R, "pdf")[1:]) / shortest - 1
        # The band's label in metres, the share of each kind, and the mean extra
        # length. (**{...} pours one dictionary into another.)
        bands.append(dict(label=f"{lo * R:.0f}-{hi * R:.0f} m", **{k: counts[k] / n for k in LEG_KINDS}, extra=extra / n))
    # Hand back one result per band.
    return bands


# The bands as a Markdown table.
def bands_table(bands):
    # The title and the table's header lines.
    lines = ["Random poses, by distance between the two points", "",
             "| Distance | Rule's word is the shortest | Exists but is longer | Does not exist (fallback) | Mean extra length |",
             "|---|---|---|---|---|"]
    # One line per band, as percentages.
    lines += [f"| {b['label']} | {100 * b['shortest']:.1f} % | {100 * b['longer']:.1f} % | {100 * b['missing']:.1f} % | "
              f"+{100 * b['extra']:.1f} % |" for b in bands]
    # Join the lines and hand back the text.
    return "\n".join(lines)


# Finds, on one map, the robot whose tour grows the most when rebuilt with the rule.
def most_changed_tour(seed):
    """The tour of the default plan on this map that gets longest when rebuilt with the rule."""
    # The normal settings, and the settings with the rule.
    p, rule = default_params(), default_params(dubins_method="pdf")
    # Plan the map the normal way.
    out = planner.run(seed, p)
    # The most changed tour so far: none yet.
    best = None
    # For every robot k and its tour:
    for k, tour in enumerate(out["result"]["tours"]):
        # Rebuild it with the rule.
        rebuilt = trajectory.build(tour["order"], out["geo"], out["scenario"], rule)
        # Keep it if it grew more than the best found so far.
        if best is None or rebuilt["L"] - tour["L"] > best["feasible"]["L"] - best["shortest"]["L"]:
            # Remember the map, the robot, and both versions of its tour.
            best = dict(seed=seed, sink=k, shortest=tour, feasible=rebuilt)
    # Hand back the planner's output and the most changed tour.
    return out, best


# Saves the five comparison pictures (drawn by plot.py) into the output folder.
def figures(rows, seed):
    """Save the five comparison figures to output/. Returns (the bands table, the paths saved)."""
    # plot.py draws pictures; it is borrowed only when needed.
    import plot
    # The "output" folder next to this file; make it if it is missing.
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    # Make the folder if it is missing (exist_ok=True: no error if it is there).
    os.makedirs(folder, exist_ok=True)
    # A tiny recipe that turns a short name into a full file path.
    path = lambda name: os.path.join(folder, f"compare_{name}.png")
    # The turning radius R from the normal settings.
    R = default_params()["R"]
    # The random-leg results.
    bands = distance_bands(R)
    # Draw and save each of the five pictures.
    # The four example legs.
    plot.compare_example_legs(example_legs(R), R, path("example_legs"))
    # How much longer the rule's plan is, map by map.
    plot.compare_extra_length(rows, path("extra_length"))
    # One robot's tour, planned both ways.
    plot.compare_same_tour(*most_changed_tour(seed), path("same_tour"))
    # The kinds of leg in each plan.
    plot.compare_leg_types(rows, path("leg_types"))
    # The random-leg results, by distance.
    plot.compare_by_distance(bands, R, path("by_distance"))
    # Hand back the bands table and the list of saved files.
    return bands_table(bands), [path(n) for n in ("example_legs", "extra_length", "same_tour", "leg_types", "by_distance")]


# What happens when the file is started from the command line.
def main():
    # Describe the options the user may type.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --seeds: which maps (one or more whole numbers; normally 1 to 10).
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    # --jobs: how many maps to plan at the same time (normally one per core).
    ap.add_argument("--jobs", type=int, default=os.cpu_count(), help="maps planned at the same time")
    # --figures: also save the pictures.
    ap.add_argument("--figures", action="store_true", help="also save comparison figures to output/")
    # Read what the user typed.
    args = ap.parse_args()
    # A "pool" of helper processes that work at the same time. "with" closes
    # the pool when the block ends. starmap calls run_one(seed, method) for
    # every seed and both methods, and collects the rows in the same order.
    with multiprocessing.Pool(args.jobs) as pool:
        # Plan every map with both methods and collect the rows.
        rows = pool.starmap(run_one, [(s, m) for s in args.seeds for m in METHODS], chunksize=1)
    # Print the tables.
    print(report(rows))
    # With --figures, save the pictures and print the bands table and file names.
    if args.figures:
        # Save the pictures (the first seed gives the one-tour picture).
        table, saved = figures(rows, args.seeds[0])
        # Print the bands table and the saved files.
        print("\n" + table + "\n\nFigures:\n" + "\n".join(saved))


# True only when this file is started directly, not when it is imported.
if __name__ == "__main__":
    # Run the main recipe.
    main()
