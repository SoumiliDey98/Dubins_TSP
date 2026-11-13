# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a short
# description written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# This file is the program's "settings page". Every number the planner needs
# (how fast the robots drive, how big the map is, how long they may take ...)
# is written down here, in one place, so it is easy to find and change.
# ==============================================================================
"""Parameters of the model (Dubins_TSP.pdf, Sections 0.2-0.7) and of the algorithms.

All functions receive the parameters as one plain dict. Change any value by
passing it to default_params, e.g. default_params(B=100.0, wheelbase=3.0).
Derived values (R, r_safe, r_plan) are recomputed from the values you pass.
"""
# "import" borrows a toolbox that comes with Python. The "math" toolbox knows
# maths words such as pi (3.14159...), tan (tangent) and atan (arc-tangent).
import math


# "def" starts a recipe (programmers say "function"). This recipe is called
# default_params. Whoever uses it gets back the full settings page.
# "**changes" means: the user may hand in any number of "name=value" pairs,
# for example default_params(B=100.0). They arrive here as a small table
# called "changes", and they replace the normal values further down.
def default_params(**changes):
    # p is a "dictionary" (dict): a table in which every entry has a name in
    # quotes (the "key") and a value after the colon. The curly bracket {
    # starts the table and the matching } further down ends it.
    p = {
        # Vehicle (Section 0.3)
        # Each mobile sink is a small robot car. It always drives at the same
        # speed v: 5 metres every second.
        "v": 5.0,                          # constant speed v [m/s]
        # The wheelbase L is the distance between the front and back wheels:
        # 2.5 metres. A longer car cannot turn as tightly.
        "wheelbase": 2.5,                  # L [m]
        # The largest angle the front wheels can be turned. math.atan(0.5) is
        # the angle whose tangent is 0.5, about 26.6 degrees (0.4636 radians).
        "delta_max": math.atan(0.5),       # maximum steering angle [rad]
        # How wide the car is: 2 metres.
        "width": 2.0,                      # w [m]
        # An extra safety gap kept from obstacles, on top of half the width: 1 metre.
        "delta_margin": 1.0,               # extra obstacle margin [m]
        # Two cars must never come closer to each other than 3 metres
        # (equation 20 of the paper). Closer than that is called a "conflict".
        "d_safe": 3.0,                     # minimum distance between two sinks, eq. (20) [m]
        # Mission (Section 0.7)
        # The time budget B: every car must be back at the base within 80 seconds.
        "B": 80.0,                         # time budget of every sink, eq. (18) [s]
        # We are allowed to use at most 30 cars.
        "K_max": 30,                       # maximum number of sinks
        # All cars start together at the base, so near the base they are
        # allowed to be close. Inside this 5 metre circle (the "docking zone")
        # the 3 metre rule is not checked when both cars are inside it.
        "base_radius": 5.0,                # docking zone: eq. (20) is not applied when both sinks are inside it
        # Map used by scenario.py (same recipe as main.py of the original code)
        # The base station stands at x = 50 m, y = 50 m: the middle of the map.
        # Round brackets ( , ) make a "tuple": a fixed little list of values.
        "base": (50.0, 50.0),
        # The map is a square, 100 metres on each side.
        "map_size": 100.0,
        # Each map gets between 10 and 15 obstacles (walls, buildings), chosen at random.
        "n_obstacles": (10, 15),
        # Each map gets between 90 and 110 sensors, chosen at random.
        "n_sensors": (90, 110),
        # Each sensor needs a car to wait next to it while its data is copied.
        # That waiting time, tau, is a random number between 1 and 5 seconds.
        "tau": (1.0, 5.0),                 # dwell time tau_i ~ uniform [s]
        # "obstacles" means: put many sensors close to the obstacles (the hard
        # case). "uniform" would spread them evenly over the map.
        "sensor_layout": "obstacles",      # "obstacles": many sensors close to obstacles; "uniform": spread out
        # 0.6 means 60 % of the sensors are placed close to an obstacle.
        "near_share": 0.6,                 # share of sensors placed close to obstacles
        # "Close" means between 3 and 7 metres from the obstacle's edge.
        "near_gap": (3.0, 7.0),            # their distance from the obstacle boundary [m] (lower bound > r_safe)
        # Half of those (0.5) are put in the inner corners of obstacles, like
        # the inside of an L shape, which are the hardest places to reach.
        "concave_share": 0.5,              # share of those placed around the concave (inner) corners of obstacles
        # Trajectories
        # dt: we write down where every car is once every half second (0.5 s).
        # Each such snapshot is called a "frame", like a frame of a film.
        "dt": 0.5,                         # time between frames [s]
        # ds: when a curved path is drawn as dots, the dots are 0.25 m apart.
        "ds": 0.25,                        # sampling step along a Dubins path [m]
        # Which way to choose the shape of each curved path: "shortest" tries
        # all six shapes and keeps the shortest; "pdf" uses the rule written in
        # Section 0.13 of the paper.
        "dubins_method": "shortest",       # "shortest": best of the six words; "pdf": turn-type rule of Section 0.13
        # Genetic algorithm (Section 0.8)
        # The genetic algorithm (GA) keeps 50 possible visiting orders at once,
        # like a population of 50 animals.
        "pop_size": 50,
        # It improves that population 100 times (100 "generations").
        "generations": 100,
        # Each new visiting order has a 0.1 chance (1 in 10) of a small random
        # change, called a "mutation".
        "mutation": 0.1,
        # Conflict game (Section 0.10)
        # Three weights (beta, gamma, delta), all 1.0, saying how much a car
        # dislikes extra time, extra distance and waiting. Equal weights mean it
        # dislikes them all the same.
        "weights": (1.0, 1.0, 1.0),        # (beta, gamma, delta) for extra time, extra distance, waiting
        # A car may stop and WAIT for at most 20 seconds.
        "max_wait": 20.0,                  # longest WAIT [s]
        # A car that steps aside (DETOUR) moves sideways d_safe + 0.5 m = 3.5 m.
        "detour_margin": 0.5,              # DETOUR side-step = d_safe + detour_margin [m]
        # When we measure how sharply a path bends, from dots, the answer can be
        # a little too big. We allow 15 % (0.15) extra before calling it a problem.
        "curvature_tol": 0.15,             # allowance when checking curvature <= 1/R on sampled points
        # Stop after 500 games, so the program can never run forever.
        "max_games": 500,
        # If conflicts remain, try to repair the routes at most 3 times.
        "repair_rounds": 3,                # Step 12: how often affected routes are re-optimised
        # "game" uses conflict_game.py to settle conflicts; "simple" uses the
        # simpler "wait your turn" rule of simple_resolver.py.
        "resolver": "game",                # "game": conflict_game.py; "simple": simple_resolver.py
    }
    # The closing } above ended the table. Now copy in any values the user
    # handed in (the "changes" table), replacing the normal ones.
    p.update(changes)
    # R, the turning radius: the smallest circle the car can drive round.
    # Equation 7 of the paper: R = wheelbase / tan(steering angle)
    # = 2.5 / tan(atan(0.5)) = 2.5 / 0.5 = 5 metres.
    p["R"] = p["wheelbase"] / math.tan(p["delta_max"])     # minimum turning radius, eq. (7) -> 5 m
    # r_safe, how far the middle of the car must stay from any obstacle
    # (equation 5): half the width (2 / 2 = 1 m) plus the safety gap (1 m) = 2 m.
    p["r_safe"] = p["width"] / 2 + p["delta_margin"]       # obstacle clearance, eq. (5) -> 2 m
    # r_plan: when we plan routes we pretend obstacles are bigger by
    # r_safe + R = 2 + 5 = 7 m, so there is always room to turn round them.
    p["r_plan"] = p["r_safe"] + p["R"]                     # obstacles are inflated by this for route finding
    # "return" hands the finished settings table back to whoever asked for it.
    return p
