"""Animations that show the whole algorithm improving the plan (Steps 1-12 of Section 0.11).

    python pipeline_gifs.py --seed 1                       # all four GIFs (about 4 minutes)
    python pipeline_gifs.py --seed 1 --only conflicts      # one or more of: conflicts splitting refine pipeline
    python pipeline_gifs.py --seed 1 --only refine --sink 3

The map is planned once, exactly as planner.py does (same random numbers), with every GA run recorded.
Then, in output/:

    seed1_conflicts.gif   the sinks moving as planned and after the conflict game, side by side; red links
                          join two sinks closer than d_safe (eq. 20); the time in conflict so far is counted
    seed1_splitting.gif   one step per GA run: a group's tour is kept (T <= B) or split in two (Section 0.9),
                          until every tour fits the budget; beside it, the tour time of every run against B
    seed1_sink8_refine.gif  one sink's tour: the GA's straight lines, the waypoints around the obstacles and
                          the Dubins path it drives, with their lengths (by default the sink whose Dubins path
                          is longest compared with its straight lines)
    seed1_pipeline.gif    all of it in one GIF: GA, splitting, routes, conflicts, conflict game, with the
                          objectives K, T_max, L_total and the time in conflict updating

--budget, --resolver and --dubins work as in planner.py and add the same suffixes to the file names.
"""
import argparse
import os

import animations
from config import default_params
from planner import run

KINDS = ("conflicts", "splitting", "refine", "pipeline")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--only", nargs="+", choices=KINDS, default=list(KINDS), help="which animations to make")
    ap.add_argument("--sink", type=int, default=None, help="sink for the refine animation (default: see above)")
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]")
    ap.add_argument("--resolver", choices=["game", "simple"], default="game", help="how conflicts between sinks are resolved")
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    args = ap.parse_args()
    changes = {} if args.budget is None else {"B": args.budget}
    p = default_params(resolver=args.resolver, dubins_method=args.dubins, **changes)

    log = []
    out = run(args.seed, p, ga_log=log)
    c = out["checks"]
    print(f"seed {args.seed}: K = {c['K']}, T_max = {c['T_max']:.1f} s, L_total = {c['L_total']:.0f} m, "
          f"{len(log)} GA runs, conflicts {out['result']['before']['exposure']:.1f} s before the game -> "
          f"{out['result']['after']['exposure']:.1f} s after")

    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(folder, exist_ok=True)
    name = f"seed{args.seed}" + ("_simple" if args.resolver == "simple" else "") + \
        ("_pdf" if args.dubins == "pdf" else "") + ("" if args.budget is None else f"_B{args.budget:g}")
    for kind in KINDS:
        if kind not in args.only:
            continue
        if kind == "conflicts":
            path = os.path.join(folder, f"{name}_conflicts.gif")
            animations.conflict_animation(out, path)
        elif kind == "splitting":
            path = os.path.join(folder, f"{name}_splitting.gif")
            animations.splitting_animation(out, log, path)
        elif kind == "refine":
            k = animations.pick_refine_sink(out) if args.sink is None else args.sink
            if not 0 <= k < len(out["result"]["tours"]):
                raise SystemExit(f"--sink must be between 0 and {len(out['result']['tours']) - 1}")
            path = os.path.join(folder, f"{name}_sink{k}_refine.gif")
            animations.refine_animation(out, k, path)
        else:
            path = os.path.join(folder, f"{name}_pipeline.gif")
            animations.pipeline_animation(out, log, path)
        print(f"animation: {path}")


if __name__ == "__main__":
    main()
