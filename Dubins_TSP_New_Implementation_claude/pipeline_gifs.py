# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# It makes eight animations (GIF files: short silent films) of the whole
# algorithm at work. It plans one map exactly as planner.py does, then asks
# animations.py to draw each film. The docstring below lists every film.
# ==============================================================================
"""Animations that show the whole algorithm improving the plan (Steps 1-12 of Section 0.11).

    python pipeline_gifs.py --seed 1                       # all eight GIFs (about 9 minutes)
    python pipeline_gifs.py --seed 1 --only partition      # one or more of: conflicts splitting partition refine
                                                           #   pipeline game dubins geodesic
    python pipeline_gifs.py --only dubins                  # needs no map, so it does not plan one
    python pipeline_gifs.py --seed 1 --only refine --sink 3

The map is planned once, exactly as planner.py does (same random numbers), with every GA run recorded.
Then, in output/:

    seed1_conflicts.gif   the sinks moving as planned and after the conflict game, side by side; red links
                          join two sinks closer than d_safe (eq. 20); the time in conflict so far is counted
    seed1_splitting.gif   one step per GA run: a group's tour is kept (T <= B) or split in two (Section 0.9),
                          until every tour fits the budget; beside it, the tour time of every run against B
    seed1_partition.gif   the same splitting, one step per split: 1 sink, then 2, then 3, ... Every group is
                          drawn with its Dubins tour, beside a bar of its tour time against B; the group that
                          is split next is outlined in red
    seed1_sink8_refine.gif  one sink's tour: the GA's straight lines, the waypoints around the obstacles and
                          the Dubins path it drives, with their lengths (by default the sink whose Dubins path
                          is longest compared with its straight lines)
    seed1_pipeline.gif    all of it in one GIF: GA, splitting, routes, conflicts, conflict game, with the
                          objectives K, T_max, L_total and the time in conflict updating
    seed1_game.gif        one conflict game replayed switch by switch (by default the one with the most
                          switches; --game N picks another): the sinks near the conflict, a timeline per
                          player of WAITs, stops and conflicts, and the potential Phi falling at every switch
    dubins_six_types.gif  the six Dubins turn types of one leg as its end heading turns through 360 degrees,
                          the shortest against the type the rule of Section 0.13 picks (no map needed)
    seed1_geodesic.gif    the geodesic distance r of eq. 9 spreading from the base along the shortest paths
                          of the visibility graph, against the straight-line distance

--budget, --resolver and --dubins work as in planner.py and add the same suffixes to the file names.
"""
# "import" borrows a toolbox. argparse reads options typed on the command line.
import argparse
# os: folders and file names.
import os

# Files of this project: the drawing of the animations, the settings, and the
# whole planner.
# animations.py: draws the films.
import animations
# config.py: the settings.
from config import default_params
# planner.py: the whole planner.
from planner import run

# The names of the eight animations, in the order they are made.
KINDS = ("conflicts", "splitting", "partition", "refine", "pipeline", "game", "dubins", "geodesic")


# "def" starts a recipe (a "function"). main is what happens when the file is
# started from the command line.
def main():
    # Describe the options the user may type.
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --seed: which map.
    ap.add_argument("--seed", type=int, default=1)
    # --only: make only some of the animations (one or more names from KINDS).
    ap.add_argument("--only", nargs="+", choices=KINDS, default=list(KINDS), help="which animations to make")
    # --sink: which robot the "refine" animation follows.
    ap.add_argument("--sink", type=int, default=None, help="sink for the refine animation (default: see above)")
    # --game: which conflict game to replay.
    ap.add_argument("--game", type=int, default=None, help="game to replay, counted from 0 (default: the most switches)")
    # --budget, --resolver, --dubins: the same options as planner.py.
    ap.add_argument("--budget", type=float, default=None, help="mission budget B [s]")
    # --resolver: "game" or "simple".
    ap.add_argument("--resolver", choices=["game", "simple"], default="game", help="how conflicts between sinks are resolved")
    # --dubins: "shortest" or "pdf".
    ap.add_argument("--dubins", choices=["shortest", "pdf"], default="shortest", help="how the Dubins word of a leg is chosen")
    # Read what the user typed.
    args = ap.parse_args()
    # Settings to change: the budget, if one was given.
    changes = {} if args.budget is None else {"B": args.budget}
    # Make the settings with those changes.
    p = default_params(resolver=args.resolver, dubins_method=args.dubins, **changes)

    # The game animation needs the game; with the simple rule there is no game,
    # so stop with a message.
    if "game" in args.only and args.resolver == "simple":
        # Stop with this message.
        raise SystemExit("the game animation needs the conflict game, not --resolver simple")
    # log: will hold every GA run; out: the planner's result (None, "nothing", for now).
    log, out = [], None
    # Every animation except "dubins" needs a planned map. If any of those was
    # asked for, plan the map once, recording every GA run.
    if any(kind != "dubins" for kind in args.only):
        # Plan the map, writing every GA run into log.
        out = run(args.seed, p, ga_log=log)
        # Print a summary: robots, longest time, total length, GA runs, and
        # seconds of conflict before and after the game.
        c = out["checks"]
        # Print the summary.
        print(f"seed {args.seed}: K = {c['K']}, T_max = {c['T_max']:.1f} s, L_total = {c['L_total']:.0f} m, "
              f"{len(log)} GA runs, conflicts {out['result']['before']['exposure']:.1f} s before the game -> "
              f"{out['result']['after']['exposure']:.1f} s after")

    # The "output" folder next to this file; make it if it is missing.
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    # Make the folder if it is missing (exist_ok=True: no error if it is there).
    os.makedirs(folder, exist_ok=True)
    # The start of every file name, e.g. "seed1", plus "_simple", "_pdf" or
    # "_B100" for those options. (The "\" continues the statement on the next line.)
    name = f"seed{args.seed}" + ("_simple" if args.resolver == "simple" else "") + \
        ("_pdf" if args.dubins == "pdf" else "") + ("" if args.budget is None else f"_B{args.budget:g}")
    # Go through the eight kinds in order:
    for kind in KINDS:
        # Skip any that were not asked for ("continue" jumps to the next one).
        if kind not in args.only:
            # Skip it.
            continue
        # Robots moving as planned and after the game, side by side.
        if kind == "conflicts":
            # Its file name.
            path = os.path.join(folder, f"{name}_conflicts.gif")
            # Make the film and save it.
            animations.conflict_animation(out, path)
        # One step per GA run: each group kept or split.
        elif kind == "splitting":
            # Its file name.
            path = os.path.join(folder, f"{name}_splitting.gif")
            # Make the film and save it.
            animations.splitting_animation(out, log, path)
        # One step per split: 1 robot, 2 robots, 3 robots ...
        elif kind == "partition":
            # Its file name.
            path = os.path.join(folder, f"{name}_partition.gif")
            # Make the film and save it.
            animations.partition_animation(out, log, path)
        # One robot's tour: straight lines, then waypoints, then the Dubins path.
        elif kind == "refine":
            # The robot to follow: the one chosen with --sink, or else a good default.
            k = animations.pick_refine_sink(out) if args.sink is None else args.sink
            # A robot number that does not exist: stop with a message.
            if not 0 <= k < len(out["result"]["tours"]):
                # Stop with this message.
                raise SystemExit(f"--sink must be between 0 and {len(out['result']['tours']) - 1}")
            # Its file name, with the robot's number.
            path = os.path.join(folder, f"{name}_sink{k}_refine.gif")
            # Make the film and save it.
            animations.refine_animation(out, k, path)
        # Everything in one film.
        elif kind == "pipeline":
            # Its file name.
            path = os.path.join(folder, f"{name}_pipeline.gif")
            # Make the film and save it.
            animations.pipeline_animation(out, log, path)
        # One conflict game replayed switch by switch; print which game it was.
        elif kind == "game":
            # Its file name.
            path = os.path.join(folder, f"{name}_game.gif")
            # Make the film; get back the game's number and its record.
            gi, g = animations.game_animation(out, path, args.game)
            # Print how many players and switches the game had.
            print(f"game {gi}: {len(g['players'])} players, {len(g['moves'])} switches")
        # The six Dubins shapes of one leg; needs no map. Print how often the
        # paper's rule picked a longer or a missing shape.
        elif kind == "dubins":
            # Its file name (the same for every map).
            path = os.path.join(folder, "dubins_six_types.gif")
            # Make the film; get back how the rule did.
            r = animations.dubins_rotation_animation(path, R=p["R"])
            # Print that.
            print(f"rule of Section 0.13 longer at {r['longer']} of {r['headings']} headings, its type missing at {r['missing']}")
        # The last kind: distances round obstacles spreading from the base.
        else:
            # Its file name.
            path = os.path.join(folder, f"{name}_geodesic.gif")
            # Make the film and save it.
            animations.geodesic_wave_animation(out, path)
        # Print where the animation was saved.
        print(f"animation: {path}")


# True only when this file is started directly, not when it is imported.
if __name__ == "__main__":
    # Run the main recipe.
    main()
