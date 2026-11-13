# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# It shows HOW the genetic algorithm (ga_tour.py) gets better, generation by
# generation. It plans the robots' routes exactly as planner.py does (same
# random numbers, so the same results), but writes down every generation of
# every GA run. Then it prints a table of all the runs, and draws two pictures
# and five animations of one run.
# ==============================================================================
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
# "import" borrows a toolbox. argparse reads options typed on the command line.
import argparse
# os: folders and file names.
import os

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np

# Files of this project: distances round obstacles, drawing, the map, the
# drivable paths, the settings, and the planner's route-planning recipe.
# geodesic.py: distances round obstacles.
import geodesic
# plot.py: draws the pictures and animations.
import plot
# scenario.py: makes the map.
import scenario
# trajectory.py: drivable paths.
import trajectory
# config.py: the settings.
from config import default_params
# planner.py's recipe for Steps 1-7.
from planner import plan_routes


# "def" starts a recipe (a "function"). This one writes the table with one line
# per GA run. log: the planner's record of every run; R: the turning radius.
def run_table(log, R):
    """One line per GA run."""
    # The two heading lines of the table.
    lines = [" run  sensors   T [s]  outcome   fitness gen 0 -> last     gain   improved in   peaks   different tours",
             "                                                                  generations  m(pi)   at the end"]
    # For every run k ("enumerate" counts 0, 1, 2 ... alongside each run):
    for k, run in enumerate(log):
        # How many sensors the group had.
        n = len(run["ids"])
        # Was the group kept as one robot, or split?
        outcome = "kept" if run["kept"] else "split"
        # Groups of 1 or 2 sensors have no GA run, so no generations to show.
        # "continue" jumps straight to the next run.
        if not run["generations"]:
            # A line saying there was no GA run ...
            lines.append(f"{k:4d}  {n:7d}  {run['T']:6.1f}  {outcome:7s}   (two sensors or fewer: no GA run)")
            # ... and on to the next run.
            continue
        # A summary of the run (from plot.py): best fitness per generation, how
        # much it improved, in which generations, peaks, and how many different tours.
        s = plot.ga_summary(run["generations"], R)
        # One table line. Inside f"...", {k:4d} writes a whole number 4 wide,
        # {x:7.1f} a decimal 7 wide with 1 digit after the point, {t:7s} text 7 wide.
        lines.append(f"{k:4d}  {n:7d}  {run['T']:6.1f}  {outcome:7s}   {s['best'][0]:7.1f} -> {s['best'][-1]:7.1f}"
                     f"  {s['gain']:5.1f} %  {len(s['improved']):6d}       {int(s['peaks'][0])} -> {int(s['peaks'][-1])}"
                     f"   {s['distinct'][-1]:6d}")
    # Join the lines with line breaks and hand back the table.
    return "\n".join(lines)


# What happens when the file is started from the command line.
def main():
    # Describe the options the user may type.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --seed: which map.
    ap.add_argument("--seed", type=int, default=1)
    # --run: which GA run to draw.
    ap.add_argument("--run", type=int, default=None,
                    help="which GA run to draw (see the table); default: the run whose best tour improved most")
    # --budget: another time budget B, which changes which groups get split.
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]; it decides which groups are split")
    # --dubins: how the curved paths' shapes are chosen.
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    # --no-gif: skip the slow animations.
    ap.add_argument("--no-gif", action="store_true", help="skip the animation")
    # Read what the user typed.
    args = ap.parse_args()
    # Settings to change: the budget, if one was given.
    changes = {} if args.budget is None else {"B": args.budget}
    # Make the settings with those changes.
    p = default_params(dubins_method=args.dubins, **changes)

    # Steps 1-7 exactly as planner.run does them, with every GA run recorded
    # The map, and the distances round obstacles.
    scen = scenario.make(args.seed, p)
    # The distances round obstacles.
    geo = geodesic.build(scen, p)
    # An empty log, filled by plan_routes with one entry per GA run.
    log = []
    # Plan the routes with the same random numbers as planner.run (seeded with the seed).
    tours, _ = plan_routes(scen, geo, p, np.random.default_rng(args.seed), ga_log=log)
    # Print a summary line and the table.
    print(f"seed {args.seed}: {len(scen['xy'])} sensors, B = {p['B']} s, {len(log)} GA runs, {len(tours)} sinks kept\n")
    # Print the table of GA runs.
    print(run_table(log, p["R"]))

    # k: the run to draw.
    k = args.run
    # If none was chosen, pick the run whose best tour improved the most
    # (among runs that had a GA; "max" with "key" picks the biggest gain).
    if k is None:
        # The run with the biggest gain.
        k = max((i for i, r in enumerate(log) if r["generations"]),
                key=lambda i: plot.ga_summary(log[i]["generations"], p["R"])["gain"])
        # Say which run is drawn ("\n" starts a new line).
        print(f"\ndrawing run {k}, the run whose best tour improved most (choose another with --run)")
    # A run number that does not exist: stop with a message.
    if not 0 <= k < len(log):
        # Stop with this message.
        raise SystemExit(f"\n--run must be between 0 and {len(log) - 1}")
    # The chosen run.
    run = log[k]
    # A run without a GA (1 or 2 sensors) cannot be drawn.
    if not run["generations"]:
        # Stop with this message.
        raise SystemExit(f"\nrun {k} has two sensors or fewer, so there was no GA run to draw")
    # Words for the pictures' titles: which sensors, and what happened to the group.
    n = len(run["ids"])
    # "all sensors", or "a group of n sensors".
    what = "all sensors" if n == len(scen["xy"]) else f"a group of {n} sensors"
    # Kept as one robot, or split because it took too long.
    then = "kept as one sink's tour" if run["kept"] else f"later split, because T = {run['T']:.1f} s > B = {p['B']:g} s"
    # The title used on every picture.
    label = f"Seed {args.seed}, GA run {k}: {what}, {then}"

    # The "output" folder next to this file; make it if it is missing.
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    # Make the folder if it is missing (exist_ok=True: no error if it is there).
    os.makedirs(folder, exist_ok=True)
    # The start of every file name, e.g. "ga_seed1_run5", plus "_pdf" or "_B100"
    # for those options. (The "\" at the end of the next line continues the
    # statement on the line after it.)
    name = f"ga_seed{args.seed}_run{k}" + ("_pdf" if args.dubins == "pdf" else "") + \
        ("" if args.budget is None else f"_B{args.budget:g}")
    # A dictionary from each kind of picture to its full file path.
    paths = {kind: os.path.join(folder, f"{name}_{kind}") for kind in
             ("convergence.png", "fitness_split.png", "evolution.gif", "population.gif", "radial.gif", "unimodal.gif",
              "unimodal_dubins.gif")}
    # Draw and save the two still pictures (plot.py does the drawing).
    # How the fitness improved, generation by generation.
    plot.ga_convergence(run["generations"], p["R"], label, paths["convergence.png"])
    # The best tour's fitness split into its two parts.
    plot.ga_fitness_split(run["generations"], p["R"], label, paths["fitness_split.png"])
    # Say where they were saved.
    print(f"\nfigures: {paths['convergence.png']}\n         {paths['fitness_split.png']}")
    # Unless --no-gif was given, make the five animations.
    if not args.no_gif:
        # The population on the map, generation by generation.
        plot.ga_animation({"scenario": scen, "params": p}, run, label, paths["evolution.gif"])
        # Every tour as a dot: length against number of peaks.
        plot.ga_population_animation(run["generations"], p["R"], label, paths["population.gif"])
        # Distance from the base along the best tour.
        plot.ga_radial_animation(run["generations"], geo["D"][0, 1:], p["R"], label, paths["radial.gif"])
        # The best tour on the map, legs coloured going out or coming back.
        plot.ga_unimodal_animation({"scenario": scen, "params": p}, run, geo["D"][0, 1:], label, paths["unimodal.gif"])
        # the Dubins tour the sink would drive for every best tour shown: generation 0, the improving ones, the last
        gens = run["generations"]
        # tours: generation -> drivable tour; built: a memory, so the same order
        # is never built twice.
        tours, built = {}, {}
        # The generations to show: the first, the last, and every one that
        # improved (a set { } holds each only once; sorted puts them in order).
        for g in sorted({0, len(gens) - 1, *plot.ga_summary(gens, p["R"])["improved"]}):
            # The best tour of that generation (tuple: a fixed list, usable as a key).
            order = tuple(gens[g]["orders"][0])
            # Build its drivable path, unless it was built already.
            if order not in built:
                # Build it and remember it.
                built[order] = trajectory.build(list(order), geo, scen, p)
            # The drivable tour for generation g.
            tours[g] = built[order]
        # The same animation, with the drivable (Dubins) paths.
        plot.ga_unimodal_animation({"scenario": scen, "params": p}, run, geo["D"][0, 1:], label,
                                   paths["unimodal_dubins.gif"], tours=tours)
        # Print where the animations were saved.
        print("animations: " + "\n            ".join(paths[k] for k in ("evolution.gif", "population.gif", "radial.gif",
                                                                     "unimodal.gif", "unimodal_dubins.gif")))


# True only when this file is started directly, not when it is imported.
if __name__ == "__main__":
    # Run the main recipe.
    main()
