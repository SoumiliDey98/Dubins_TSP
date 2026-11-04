"""Parameters of the model (Dubins_TSP.pdf, Sections 0.2-0.7) and of the algorithms.

All functions receive the parameters as one plain dict. Change any value by
passing it to default_params, e.g. default_params(B=100.0, wheelbase=3.0).
Derived values (R, r_safe, r_plan) are recomputed from the values you pass.
"""
import math


def default_params(**changes):
    p = {
        # Vehicle (Section 0.3)
        "v": 5.0,                          # constant speed v [m/s]
        "wheelbase": 2.5,                  # L [m]
        "delta_max": math.atan(0.5),       # maximum steering angle [rad]
        "width": 2.0,                      # w [m]
        "delta_margin": 1.0,               # extra obstacle margin [m]
        "d_safe": 3.0,                     # minimum distance between two sinks, eq. (20) [m]
        # Mission (Section 0.7)
        "B": 80.0,                         # time budget of every sink, eq. (18) [s]
        "K_max": 30,                       # maximum number of sinks
        "base_radius": 5.0,                # docking zone: eq. (20) is not applied when both sinks are inside it
        # Map used by scenario.py (same recipe as main.py of the original code)
        "base": (50.0, 50.0),
        "map_size": 100.0,
        "n_obstacles": (10, 15),
        "n_sensors": (90, 110),
        "tau": (1.0, 5.0),                 # dwell time tau_i ~ uniform [s]
        "sensor_layout": "obstacles",      # "obstacles": many sensors close to obstacles; "uniform": spread out
        "near_share": 0.6,                 # share of sensors placed close to obstacles
        "near_gap": (3.0, 7.0),            # their distance from the obstacle boundary [m] (lower bound > r_safe)
        "concave_share": 0.5,              # share of those placed around the concave (inner) corners of obstacles
        # Trajectories
        "dt": 0.5,                         # time between frames [s]
        "ds": 0.25,                        # sampling step along a Dubins path [m]
        "dubins_method": "shortest",       # "shortest": best of the six words; "pdf": turn-type rule of Section 0.13
        # Genetic algorithm (Section 0.8)
        "pop_size": 50,
        "generations": 100,
        "mutation": 0.1,
        # Conflict game (Section 0.10)
        "weights": (1.0, 1.0, 1.0),        # (beta, gamma, delta) for extra time, extra distance, waiting
        "max_wait": 20.0,                  # longest WAIT [s]
        "detour_margin": 0.5,              # DETOUR side-step = d_safe + detour_margin [m]
        "curvature_tol": 0.15,             # allowance when checking curvature <= 1/R on sampled points
        "max_games": 500,
        "repair_rounds": 3,                # Step 12: how often affected routes are re-optimised
        "resolver": "game",                # "game": conflict_game.py; "simple": simple_resolver.py
    }
    p.update(changes)
    p["R"] = p["wheelbase"] / math.tan(p["delta_max"])     # minimum turning radius, eq. (7) -> 5 m
    p["r_safe"] = p["width"] / 2 + p["delta_margin"]       # obstacle clearance, eq. (5) -> 2 m
    p["r_plan"] = p["r_safe"] + p["R"]                     # obstacles are inflated by this for route finding
    return p
