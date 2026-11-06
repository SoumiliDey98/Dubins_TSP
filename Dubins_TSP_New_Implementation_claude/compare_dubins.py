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
import argparse
import math
import multiprocessing
import os
import sys

import numpy as np

import dubins_path
import planner
import trajectory
from config import default_params

METHODS = ("shortest", "pdf")
LEG_KINDS = ("shortest", "longer", "missing")
# Example legs for the first figure: (title, start, end), with (x [m], y [m], heading [degrees]) for R = 5 m.
EXAMPLE_LEGS = (("The rule picks the shortest path", (0, 0, 90), (30, 5, -90)),
                ("The rule picks a longer CSC path", (0, 0, 0), (15, -5, 180)),
                ("The rule picks CCC, but CSC is shorter", (0, 0, 90), (5, -10, -90)),
                ("The rule's {rule} does not exist: fallback used", (0, 0, 0), (5, -5, 0)))
DISTANCE_BANDS = ((0, 2), (2, 4), (4, 8), (8, 20))      # distance between the two points, in units of R


def classify_leg(q0, q1, R):
    """How the rule of Section 0.13 does on one leg: its word is the "shortest" one,
    exists but is "longer", or is "missing" (it does not exist, so the fallback is used)."""
    turnType, *lengths = dubins_path.findDubinsPath(*q0, *q1, R, "pdf")
    if turnType != dubins_path.findTurnType(*q0, *q1, R):
        return "missing"
    shortest = sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:])
    return "shortest" if sum(lengths) <= shortest + 1e-9 else "longer"


def leg_counts(tours, geo, p):
    """Number of legs of each kind in these tours. Waypoints and headings are recomputed
    the way trajectory.build does; both are deterministic, so these are the legs it drove."""
    counts = dict.fromkeys(LEG_KINDS, 0)
    for tour in tours:
        pts, _ = trajectory.waypoints(tour["order"], geo, p)
        h = trajectory.choose_headings(pts, geo, p)
        for k in range(len(pts) - 1):
            counts[classify_leg((*pts[k], h[k]), (*pts[k + 1], h[k + 1]), p["R"])] += 1
    return counts


def run_one(seed, method):
    """Plan one map with one method and return its numbers."""
    p = default_params(dubins_method=method)
    out = planner.run(seed, p)
    c, res = out["checks"], out["result"]
    row = {"seed": seed, "method": method, "K": c["K"], "T_max": c["T_max"], "L": c["L_total"],
           "clearance": c["clearance"], "curvature": c["curvature_x_R"], "pairs": c["conflict_pairs"],
           "risk": res["after"]["exposure"], "closest": c["closest"],
           "ok": (c["coverage_ok"] and c["budget_ok"] and c["K_ok"] and not c["problems"]    # all but eq. (20)
                  and c["clearance"] >= p["r_safe"] and c["curvature_x_R"] <= 1 + p["curvature_tol"]),
           "legs": leg_counts(res["tours"], out["geo"], p)}
    if method == "shortest":
        rule = default_params(dubins_method="pdf")
        rebuilt = [trajectory.build(t["order"], out["geo"], out["scenario"], rule) for t in res["tours"]]
        row["same_orders"] = {"L_shortest": sum(t["L"] for t in res["tours"]), "L_rule": sum(t["L"] for t in rebuilt),
                              "over_B": sum(t["T"] > p["B"] + 1e-9 for t in rebuilt)}
    print(f"seed {seed}, {method}: done", file=sys.stderr, flush=True)
    return row


def report(rows):
    """The comparison as Markdown tables. rows must hold both methods for every seed, in the same seed order."""
    plans = {m: [r for r in rows if r["method"] == m] for m in METHODS}

    def compare(label, value):
        return f"| {label} | {value(plans['shortest'])} | {value(plans['pdf'])} |"

    def legs(kind):
        def value(rs):
            n, total = sum(r["legs"][kind] for r in rs), sum(sum(r["legs"].values()) for r in rs)
            return f"{n} of {total} ({100 * n / total:.1f} %)"
        return value

    lines = ["Per map", "",
             "| Seed | Method | K | T_max (s) | L_total (m) | Clearance (m) | Curvature x R "
             "| Pairs left | Risk left (s) | Closest (m) | Checks except eq. 20 |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    lines += [f"| {r['seed']} | {r['method']} | {r['K']} | {r['T_max']:.1f} | {r['L']:.0f} | {r['clearance']:.2f} "
              f"| {r['curvature']:.3f} | {r['pairs']} | {r['risk']:.1f} | {r['closest']:.2f} "
              f"| {'ok' if r['ok'] else 'FAILED'} |" for r in rows]

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
    a, b = sum(r["L"] for r in plans["shortest"]), sum(r["L"] for r in plans["pdf"])
    longer = sum(y["L"] > x["L"] for x, y in zip(plans["shortest"], plans["pdf"]))
    lines += ["", f"L_total with the rule: {100 * (b / a - 1):+.1f} %, longer on {longer} of {len(plans['pdf'])} maps."]

    same = [r["same_orders"] for r in plans["shortest"]]
    a, b = sum(s["L_shortest"] for s in same), sum(s["L_rule"] for s in same)
    lines += ["", f"Same visiting orders: the {sum(r['K'] for r in plans['shortest'])} tours of the default plan rebuilt "
                  f"with the rule: {a:.0f} m -> {b:.0f} m ({100 * (b / a - 1):+.1f} %), "
                  f"tours over B: {sum(s['over_B'] for s in same)}."]

    lines += ["", "How the rule of Section 0.13 does on the legs of each final plan", "",
              "| | Default plan | Plan made with the rule |", "|---|---|---|",
              compare("Rule's word is the shortest", legs("shortest")),
              compare("Rule's word exists but is longer", legs("longer")),
              compare("Rule's word does not exist (fallback)", legs("missing"))]
    return "\n".join(lines)


def example_legs(R):
    """The legs of EXAMPLE_LEGS, each with its kind and both paths: {"title", "q0", "q1", "kind", "shortest", "feasible"}."""
    legs = []
    for title, a, b in EXAMPLE_LEGS:
        q0, q1 = (a[0], a[1], math.radians(a[2])), (b[0], b[1], math.radians(b[2]))
        rule = dubins_path.findTurnType(*q0, *q1, R)
        leg = dict(title=title.format(rule=rule), q0=q0, q1=q1, kind=classify_leg(q0, q1, R), rule=rule)
        for name, method in (("shortest", "shortest"), ("feasible", "pdf")):
            x, y, length, _ = dubins_path.returnDubinsPath(*q0, *q1, R, method, 0.05)
            leg[name] = dict(x=x, y=y, length=length, turnType=dubins_path.findDubinsPath(*q0, *q1, R, method)[0])
        legs.append(leg)
    return legs


def distance_bands(R, n=5000, seed=0):
    """How the rule does on n random poses in each band of DISTANCE_BANDS: the share of each kind of
    leg, and the mean extra length of the feasible path. These poses are random, so this is not the
    planner's cost: the heading search avoids many of the poses where the rule does badly."""
    rng, bands = np.random.default_rng(seed), []
    for lo, hi in DISTANCE_BANDS:
        counts, extra = dict.fromkeys(LEG_KINDS, 0), 0.0
        for _ in range(n):
            d, angle = rng.uniform(lo, hi) * R, rng.uniform(0, 2 * math.pi)
            q0, q1 = (0.0, 0.0, rng.uniform(0, 2 * math.pi)), (d * math.cos(angle), d * math.sin(angle), rng.uniform(0, 2 * math.pi))
            counts[classify_leg(q0, q1, R)] += 1
            shortest = sum(dubins_path.findDubinsPath(*q0, *q1, R, "shortest")[1:])
            extra += sum(dubins_path.findDubinsPath(*q0, *q1, R, "pdf")[1:]) / shortest - 1
        bands.append(dict(label=f"{lo * R:.0f}-{hi * R:.0f} m", **{k: counts[k] / n for k in LEG_KINDS}, extra=extra / n))
    return bands


def bands_table(bands):
    lines = ["Random poses, by distance between the two points", "",
             "| Distance | Rule's word is the shortest | Exists but is longer | Does not exist (fallback) | Mean extra length |",
             "|---|---|---|---|---|"]
    lines += [f"| {b['label']} | {100 * b['shortest']:.1f} % | {100 * b['longer']:.1f} % | {100 * b['missing']:.1f} % | "
              f"+{100 * b['extra']:.1f} % |" for b in bands]
    return "\n".join(lines)


def most_changed_tour(seed):
    """The tour of the default plan on this map that gets longest when rebuilt with the rule."""
    p, rule = default_params(), default_params(dubins_method="pdf")
    out = planner.run(seed, p)
    best = None
    for k, tour in enumerate(out["result"]["tours"]):
        rebuilt = trajectory.build(tour["order"], out["geo"], out["scenario"], rule)
        if best is None or rebuilt["L"] - tour["L"] > best["feasible"]["L"] - best["shortest"]["L"]:
            best = dict(seed=seed, sink=k, shortest=tour, feasible=rebuilt)
    return out, best


def figures(rows, seed):
    """Save the five comparison figures to output/. Returns (the bands table, the paths saved)."""
    import plot
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(folder, exist_ok=True)
    path = lambda name: os.path.join(folder, f"compare_{name}.png")
    R = default_params()["R"]
    bands = distance_bands(R)
    plot.compare_example_legs(example_legs(R), R, path("example_legs"))
    plot.compare_extra_length(rows, path("extra_length"))
    plot.compare_same_tour(*most_changed_tour(seed), path("same_tour"))
    plot.compare_leg_types(rows, path("leg_types"))
    plot.compare_by_distance(bands, R, path("by_distance"))
    return bands_table(bands), [path(n) for n in ("example_legs", "extra_length", "same_tour", "leg_types", "by_distance")]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    ap.add_argument("--jobs", type=int, default=os.cpu_count(), help="maps planned at the same time")
    ap.add_argument("--figures", action="store_true", help="also save comparison figures to output/")
    args = ap.parse_args()
    with multiprocessing.Pool(args.jobs) as pool:
        rows = pool.starmap(run_one, [(s, m) for s in args.seeds for m in METHODS], chunksize=1)
    print(report(rows))
    if args.figures:
        table, saved = figures(rows, args.seeds[0])
        print("\n" + table + "\n\nFigures:\n" + "\n".join(saved))


if __name__ == "__main__":
    main()
