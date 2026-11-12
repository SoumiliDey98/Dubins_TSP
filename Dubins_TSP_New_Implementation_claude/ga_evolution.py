"""How the genetic algorithm of Section 0.8 improves its tours, generation by generation.

    python ga_evolution.py --seed 1              # table of every GA run; figures for the run that improved most
    python ga_evolution.py --seed 1 --run 0      # figures for another run in the table (run 0: all sensors)
    python ga_evolution.py --seed 1 --no-gif     # only the two PNG figures (the five GIFs take about 4 minutes)

In Steps 1-7 the planner runs the GA once for each group of sensors: first for all sensors, then for
each half whenever a group's tour takes longer than the budget B. This script plans the routes exactly
as planner.py does (same random numbers, so the same groups and tours), records every generation of
every GA run, prints a table of the runs, and saves seven figures for the chosen run (here run 5):

    output/ga_seed1_run5_convergence.png    fitness of the best tour, population mean and range, per generation
    output/ga_seed1_run5_fitness_split.png  the best tour's fitness split into straight-line length and
                                            peak penalty m(pi)*pi*R, and its number of peaks m(pi)
    output/ga_seed1_run5_evolution.gif      the population and its best tour on the map, generation by generation
    output/ga_seed1_run5_population.gif     every tour as a dot: straight-line length against m(pi), with lines
                                            of equal fitness, generation by generation
    output/ga_seed1_run5_radial.gif         distance from the base along the best tour: one hill at generation 0
                                            (radially unimodal), more hills later
    output/ga_seed1_run5_unimodal.gif       the same on the map: legs coloured by going out (blue) or coming back
                                            (orange), peaks ringed; generation 0 and the last traced leg by leg
    output/ga_seed1_run5_unimodal_dubins.gif  the same, with every tour drawn as the Dubins path the sink would
                                            drive (trajectory.build), and its length and mission time

The GA minimises Fitness(pi) = straight-line length + m(pi)*pi*R (Section 0.8.1). It keeps the better half
of each generation, so the best fitness never rises. Groups that are split later are usually large, and
their runs improve the most; the small groups that become the final sinks often gain nothing, because the
radially unimodal initial population (Section 0.8.2) already holds the best tour the run finds.
"""
import argparse
import os

import numpy as np

import geodesic
import plot
import scenario
import trajectory
from config import default_params
from planner import plan_routes


def run_table(log, R):
    """One line per GA run."""
    lines = [" run  sensors   T [s]  outcome   fitness gen 0 -> last     gain   improved in   peaks   different tours",
             "                                                                  generations  m(pi)   at the end"]
    for k, run in enumerate(log):
        n = len(run["ids"])
        outcome = "kept" if run["kept"] else "split"
        if not run["generations"]:
            lines.append(f"{k:4d}  {n:7d}  {run['T']:6.1f}  {outcome:7s}   (two sensors or fewer: no GA run)")
            continue
        s = plot.ga_summary(run["generations"], R)
        lines.append(f"{k:4d}  {n:7d}  {run['T']:6.1f}  {outcome:7s}   {s['best'][0]:7.1f} -> {s['best'][-1]:7.1f}"
                     f"  {s['gain']:5.1f} %  {len(s['improved']):6d}       {int(s['peaks'][0])} -> {int(s['peaks'][-1])}"
                     f"   {s['distinct'][-1]:6d}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--run", type=int, default=None,
                    help="which GA run to draw (see the table); default: the run whose best tour improved most")
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]; it decides which groups are split")
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    ap.add_argument("--no-gif", action="store_true", help="skip the animation")
    args = ap.parse_args()
    changes = {} if args.budget is None else {"B": args.budget}
    p = default_params(dubins_method=args.dubins, **changes)

    # Steps 1-7 exactly as planner.run does them, with every GA run recorded
    scen = scenario.make(args.seed, p)
    geo = geodesic.build(scen, p)
    log = []
    tours, _ = plan_routes(scen, geo, p, np.random.default_rng(args.seed), ga_log=log)
    print(f"seed {args.seed}: {len(scen['xy'])} sensors, B = {p['B']} s, {len(log)} GA runs, {len(tours)} sinks kept\n")
    print(run_table(log, p["R"]))

    k = args.run
    if k is None:
        k = max((i for i, r in enumerate(log) if r["generations"]),
                key=lambda i: plot.ga_summary(log[i]["generations"], p["R"])["gain"])
        print(f"\ndrawing run {k}, the run whose best tour improved most (choose another with --run)")
    if not 0 <= k < len(log):
        raise SystemExit(f"\n--run must be between 0 and {len(log) - 1}")
    run = log[k]
    if not run["generations"]:
        raise SystemExit(f"\nrun {k} has two sensors or fewer, so there was no GA run to draw")
    n = len(run["ids"])
    what = "all sensors" if n == len(scen["xy"]) else f"a group of {n} sensors"
    then = "kept as one sink's tour" if run["kept"] else f"later split, because T = {run['T']:.1f} s > B = {p['B']:g} s"
    label = f"Seed {args.seed}, GA run {k}: {what}, {then}"

    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    os.makedirs(folder, exist_ok=True)
    name = f"ga_seed{args.seed}_run{k}" + ("_pdf" if args.dubins == "pdf" else "") + \
        ("" if args.budget is None else f"_B{args.budget:g}")
    paths = {kind: os.path.join(folder, f"{name}_{kind}") for kind in
             ("convergence.png", "fitness_split.png", "evolution.gif", "population.gif", "radial.gif", "unimodal.gif",
              "unimodal_dubins.gif")}
    plot.ga_convergence(run["generations"], p["R"], label, paths["convergence.png"])
    plot.ga_fitness_split(run["generations"], p["R"], label, paths["fitness_split.png"])
    print(f"\nfigures: {paths['convergence.png']}\n         {paths['fitness_split.png']}")
    if not args.no_gif:
        plot.ga_animation({"scenario": scen, "params": p}, run, label, paths["evolution.gif"])
        plot.ga_population_animation(run["generations"], p["R"], label, paths["population.gif"])
        plot.ga_radial_animation(run["generations"], geo["D"][0, 1:], p["R"], label, paths["radial.gif"])
        plot.ga_unimodal_animation({"scenario": scen, "params": p}, run, geo["D"][0, 1:], label, paths["unimodal.gif"])
        # the Dubins tour the sink would drive for every best tour shown: generation 0, the improving ones, the last
        gens = run["generations"]
        tours, built = {}, {}
        for g in sorted({0, len(gens) - 1, *plot.ga_summary(gens, p["R"])["improved"]}):
            order = tuple(gens[g]["orders"][0])
            if order not in built:
                built[order] = trajectory.build(list(order), geo, scen, p)
            tours[g] = built[order]
        plot.ga_unimodal_animation({"scenario": scen, "params": p}, run, geo["D"][0, 1:], label,
                                   paths["unimodal_dubins.gif"], tours=tours)
        print("animations: " + "\n            ".join(paths[k] for k in ("evolution.gif", "population.gif", "radial.gif",
                                                                     "unimodal.gif", "unimodal_dubins.gif")))


if __name__ == "__main__":
    main()
