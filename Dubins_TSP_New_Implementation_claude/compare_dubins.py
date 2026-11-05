"""Compare the two ways of choosing a Dubins word (p["dubins_method"]) on the same maps.

Prints the numbers of the README section "Shortest of six or the rule of Section 0.13?":
  1. Each method plans its own routes on every seed; the checks of planner.py score them.
  2. Same visiting orders: the tours of the default plan are rebuilt with the rule. This
     separates the rule from differences in GA runs and splits.
  3. Legs: how the rule does on the legs of each final plan (see classify_leg).

    python compare_dubins.py                       # seeds 1-10, one job per core
    python compare_dubins.py --seeds 1 2 --jobs 1
"""
import argparse
import multiprocessing
import os
import sys

import dubins_path
import planner
import trajectory
from config import default_params

METHODS = ("shortest", "pdf")
LEG_KINDS = ("shortest", "longer", "missing")


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

    lines += ["", "Totals", "", "| | Shortest of six | Rule of Section 0.13 |", "|---|---|---|",
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(1, 11)))
    ap.add_argument("--jobs", type=int, default=os.cpu_count(), help="maps planned at the same time")
    args = ap.parse_args()
    with multiprocessing.Pool(args.jobs) as pool:
        rows = pool.starmap(run_one, [(s, m) for s in args.seeds for m in METHODS], chunksize=1)
    print(report(rows))


if __name__ == "__main__":
    main()
