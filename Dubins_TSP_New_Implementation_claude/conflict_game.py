# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# All the robots drive at the same time. If two of them would come closer than
# d_safe = 3 m to each other, that is a "conflict" (equation 20 of the paper).
# This file finds conflicts and settles them with a small "game":
#   - The robots in the conflict are the "players".
#   - Each player picks a "strategy": GO (keep the plan), WAIT (stop for a
#     moment, then carry on), or DETOUR (step aside 3.5 m, then rejoin its path).
#   - Each choice has a "cost": a big cost for every second spent too close to
#     another robot, plus small costs for extra time, extra distance and waiting.
#   - Players take turns switching to whatever lowers their OWN cost the most.
#     When nobody can do better by switching, the game stops. That resting
#     point is a "Nash equilibrium". Because both robots of a close pair pay the
#     same danger cost, one total score (the "potential") always goes down,
#     so the game can never go round in circles: it always ends.
#
# A robot's whole drive is a "trajectory": a list of positions, one every dt =
# 0.5 seconds ("frames"), starting at time 0.
# ==============================================================================
"""Conflict detection (eq. 20) and the GO / WAIT / DETOUR conflict game (Section 0.10).

Adapted from theory_claude/corridor_game_gt.py (same game; its theory is in
theory_claude/Corridor_Game_Theory.pdf), with the rules of Dubins_TSP.pdf:
  - eq. (20) applies only while both sinks are on their mission, t <= min(T_k, T_l);
  - a strategy that makes a sink exceed the budget B is rejected (Section 0.10,
    step 6), so WAIT may only use the time left in the budget.
A trajectory is an (F, 2) array: one position every dt seconds, starting at t = 0.

The game, in short. The sinks that are too close in the first unsafe interval
are the players. Each player chooses GO (keep the plan), WAIT (stop briefly,
then continue) or DETOUR (side-step and rejoin the own path, only if feasible).
Its cost is
    J_k = alpha * (seconds closer than d_safe to any other sink)
          + beta * extra time + gamma * extra distance / v + delta * waiting time,
with alpha so large that no amount of time saved outweighs any danger. Players
switch to their best response one at a time until nobody can lower their own
cost: a pure Nash equilibrium. Because the danger of a pair is charged equally
to both sinks, Phi = alpha * (danger of every pair, counted once) + sum of own
costs changes exactly as much as the mover's cost, so this always stops.
"""
# "import" borrows a toolbox. "math" has maths for single numbers.
import math

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# shapely: a toolbox for flat shapes and the distances between them.
import shapely
# LineString: a line made of several straight pieces joined end to end.
from shapely.geometry import LineString

# This project's file for paths; used here to measure how sharply a path bends.
import trajectory

# The three strategy names, written once so they are always spelled the same.
GO, WAIT, DETOUR = "GO", "WAIT", "DETOUR"
# EPS ("epsilon"): a tiny number, 0.000000001, used to ignore rounding errors.
EPS = 1e-9


# ---------------------------------------------------------------------------
# Conflict detection
# ---------------------------------------------------------------------------

# "def" starts a recipe (a "function"). This one measures how close two robots
# come. P and Q are their trajectories (one position per frame).
def closest(P, Q, p):
    """For every interval [frame n, frame n+1] while both sinks are on their mission:
    the closest distance (straight-line motion between frames) and whether both
    sinks are inside the docking zone around the base at that closest moment."""
    # Only compare while BOTH are still driving: up to the shorter trajectory.
    m = min(len(P), len(Q))
    # With fewer than 2 frames there is no time step to look at: hand back two
    # empty arrays.
    if m < 2:
        # Hand back two empty arrays.
        return np.zeros(0), np.zeros(0, dtype=bool)
    # r0: the arrow from Q to P at the start of each half-second step.
    r0 = P[:m - 1] - Q[:m - 1]
    # dr: how that arrow changes by the end of the step.
    dr = (P[1:m] - Q[1:m]) - r0
    # Between two frames each robot moves in a straight line, so the arrow is
    # r0 + tau * dr, with tau going from 0 (start of the step) to 1 (end).
    # den = dr . dr, the "dot product" of dr with itself (its length squared).
    # np.einsum("ij,ij->i", a, b) computes a row-by-row dot product.
    den = np.einsum("ij,ij->i", dr, dr)
    # The tau at which the arrow is shortest: -(r0 . dr) / (dr . dr). If dr is
    # (almost) zero, divide by 1 instead. np.clip keeps tau between 0 and 1, so
    # the moment stays inside the step.
    tau = np.clip(-np.einsum("ij,ij->i", r0, dr) / np.where(den > EPS, den, 1.0), 0.0, 1.0)
    # d: the distance between the robots at that closest moment, for every step.
    d = np.hypot(*(r0 + tau[:, None] * dr).T)
    # The base's position.
    base = np.asarray(p["base"])
    # Where each robot is at that closest moment (pc for P, qc for Q).
    pc = P[:m - 1] + tau[:, None] * (P[1:m] - P[:m - 1])
    # ... and where Q is at that same moment.
    qc = Q[:m - 1] + tau[:, None] * (Q[1:m] - Q[:m - 1])
    # dock: True when BOTH are within 5 m of the base at that moment. Everyone
    # starts there together, so being close there does not count.
    dock = (np.hypot(*(pc - base).T) <= p["base_radius"]) & (np.hypot(*(qc - base).T) <= p["base_radius"])
    # Hand back the closest distance of every step and the docking yes/no.
    return d, dock


# For every half-second step: are the two robots in conflict?
def unsafe(P, Q, p):
    """u[n] = True if the two sinks break eq. (20) between frames n and n+1."""
    # Their closest distance in every step, and whether both are docked.
    d, dock = closest(P, Q, p)
    # Conflict = closer than d_safe (3 m) AND not both in the docking zone.
    # "~" means "not"; "&" means "and".
    return (d < p["d_safe"]) & ~dock


# The danger between two robots: how many seconds they spend in conflict,
# counting only from frame "start" onwards.
def exposure(P, Q, start, p):
    """Collision risk of a pair: seconds closer than d_safe from frame `start` on."""
    # Count the unsafe steps (np.count_nonzero counts the Trues) and multiply
    # by 0.5 s per step.
    return p["dt"] * float(np.count_nonzero(unsafe(P, Q, p)[start:]))


# A report on all robots together. "trajs" is a dictionary: robot number ->
# its trajectory.
def summary(trajs, p):
    """Conflicting pairs, total exposure [s] and closest approach [m] (outside the docking zone)."""
    # The robot numbers in order.
    keys = sorted(trajs)
    # pairs: the pairs in conflict; total: seconds of conflict; closest_d: the
    # smallest distance seen so far, starting at infinity.
    pairs, total, closest_d = [], 0.0, math.inf
    # Every pair (k, j) once: "enumerate" counts i = 0, 1, 2 ... alongside each
    # robot k, and keys[i + 1:] are the robots after it.
    for i, k in enumerate(keys):
        # ... and every robot j after it:
        for j in keys[i + 1:]:
            # Their closest distances, step by step.
            d, dock = closest(trajs[k], trajs[j], p)
            # The steps in conflict.
            bad = (d < p["d_safe"]) & ~dock
            # If there is any, note the pair and add its seconds.
            if bad.any():
                # Note the pair ...
                pairs.append((k, j))
                # ... and add its seconds of conflict (unsafe steps times 0.5 s).
                total += p["dt"] * float(np.count_nonzero(bad))
            # Update the closest approach, counting only moments outside the docking zone.
            if (~dock).any():
                # The smallest distance outside the docking zone, if it beats the old one.
                closest_d = min(closest_d, float(d[~dock].min()))
    # Hand back the report as a dictionary.
    return {"pairs": pairs, "exposure": total, "closest": closest_d}


# ---------------------------------------------------------------------------
# Strategies: each one is a complete new trajectory for one sink
# ---------------------------------------------------------------------------

# The WAIT strategy: the robot stands still at frame s for w extra frames.
def wait_traj(P, s, w):
    """WAIT: stop at the position of frame s for w extra frames, then continue unchanged."""
    # The frames up to s, then frame s repeated w more times (np.repeat), then
    # the rest of the plan. np.vstack stacks the three parts into one list.
    return np.vstack([P[:s + 1], np.repeat(P[s:s + 1], w, axis=0), P[s + 1:]])


# The "_" at the start of the name means "only used inside this file".
# Puts points evenly along a line, "step" metres apart (so one point per frame
# at the robot's speed).
def _resample(points, step):
    # The length of each piece of the line.
    seg = np.hypot(*np.diff(points, axis=0).T)
    # cum: the distance along the line to each point (a running total, which
    # np.cumsum makes), starting from 0.
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    # How many steps fit along the whole line, rounded up (at least 1).
    n = max(1, int(math.ceil(cum[-1] / step - 1e-9)))
    # The distances of the new points: 0, step, 2*step, ..., never past the end.
    s = np.minimum(np.arange(n + 1) * step, cum[-1])
    # The new points, read off the line at those distances (np.interp reads
    # between the old points in a straight line).
    return np.column_stack([np.interp(s, cum, points[:, 0]), np.interp(s, cum, points[:, 1])])


# The DETOUR strategy: step sideways off the planned path, pass the other robot,
# and rejoin. P: the plan; s: the frame where the robot decides; f to f_end: the
# frames of the conflict; e: the frame by which it must be back on its path;
# side: +1 for the right, -1 for the left; geo: the obstacles; p: the settings.
def detour_traj(P, s, f, f_end, e, side, geo, p):
    """DETOUR: side-step h = d_safe + detour_margin to the right (side=+1) or left (-1)
    of the own path during frames f..f_end, rejoining by frame e. Returns
    (trajectory, extra distance) or None if not feasible: the sink stands still
    during s..f_end, the ramps are too short, the turn is sharper than R allows,
    or the new path comes closer than r_safe to an obstacle."""
    # h: how far to step aside, 3 + 0.5 = 3.5 m. R: the turning radius, 5 m.
    h, R = p["d_safe"] + p["detour_margin"], p["R"]
    # Do not plan beyond the last frame.
    e = min(e, len(P) - 1)
    # The frames must come in this order: decide, conflict starts, conflict
    # ends, rejoin. If not, a detour is impossible: hand back None ("nothing").
    if s < 1 or not (s < f <= f_end < e):
        # In the wrong order: no detour is possible.
        return None
    # The length of each step from frame s to frame e.
    steps = np.hypot(*np.diff(P[s:e + 1], axis=0).T)
    # still: True for steps where the robot does not move (it is waiting at a sensor).
    still = steps < 1e-9
    # Places where it stands still for 2 or more steps in a row.
    stops = np.flatnonzero(still[1:] & still[:-1])        # stopped for 2+ frames
    # If there is such a stop ...
    if stops.size:
        # ... during the conflict, a side-step is impossible: give up.
        if s + stops[0] <= f_end:
            return None                                    # cannot side-step while standing still
        # ... after the conflict: rejoin before that stop.
        e = s + int(stops[0])                              # rejoin before the next stop
        # Shorten the step lists to the new rejoin frame.
        steps, still = steps[:e - s], still[:e - s]
    # The piece of the plan that will be changed.
    window = P[s:e + 1]
    # cum: the distance along that piece to each frame.
    cum = np.concatenate([[0.0], np.cumsum(steps)])
    # Keep only frames where the robot moved, so every point is a new position.
    keep = np.concatenate([[True], ~still])
    # Q: the kept points; sig: their distances along the piece.
    Q, sig = window[keep], cum[keep]
    # a: distance from the decision point to where the conflict starts;
    # b: to where it ends; total: to the rejoin point.
    a, b, total = cum[f - s], cum[f_end - s], cum[-1]
    # The side-step must not bend tighter than radius R. A smooth (cosine-shaped)
    # sideways move of h metres needs at least pi * sqrt(h * R / 2) metres of
    # driving to stay that gentle.
    ramp_min = math.pi * math.sqrt(h * R / 2)             # cosine ramp with curvature <= 1/R
    # If there is not enough room before or after the conflict, give up.
    if a < ramp_min or total - b < ramp_min:
        # Not enough room for gentle ramps: give up.
        return None
    # prof: how far to the side at each point, from 0 (on the path) to 1 (fully
    # aside). Fully aside during the conflict ...
    prof = np.ones_like(sig)
    # ... a smooth ramp up before it, and a smooth ramp down after it.
    up, down = sig < a, sig > b
    # The ramp up: (1 - cos(pi * x)) / 2 rises smoothly from 0 to 1.
    prof[up] = 0.5 * (1 - np.cos(np.pi * sig[up] / a))
    # The ramp down: the same, falling back to 0 at the rejoin point.
    prof[down] = 0.5 * (1 - np.cos(np.pi * (total - sig[down]) / (total - b)))
    # tan: the direction of travel at each point (np.gradient compares
    # neighbouring points), shrunk to length 1.
    tan = np.gradient(Q, axis=0)
    # Shrink each direction to length 1 (np.maximum stops a division by 0).
    tan /= np.maximum(np.hypot(*tan.T)[:, None], EPS)
    # Move each point sideways: (tan_y, -tan_x) points to the right of the
    # direction of travel; side = -1 flips it to the left. Multiply by h and by
    # the ramp.
    new_path = Q + side * h * prof[:, None] * np.column_stack([tan[:, 1], -tan[:, 0]])
    # Space the new path so the robot keeps its speed: one point every v * dt = 2.5 m.
    frames = _resample(new_path, p["v"] * p["dt"])
    # The detour is a little longer, so it may need more frames. If it came out
    # SHORTER in frames, pad it by standing still at the end, so the robot is
    # never faster than its plan.
    short = (e - s + 1) - len(frames)                      # never faster than GO
    # Fewer frames than the plan's piece?
    if short > 0:
        # Then repeat the last point to make up the difference.
        frames = np.vstack([frames, np.repeat(frames[-1:], short, axis=0)])
    # The whole new trajectory: the plan before s, the detour, the plan after e.
    new_traj = np.vstack([P[:s], frames, P[e + 1:]])
    # Curvature over the whole trajectory, as the final check measures it: frames next to the
    # detour may be parked at a sensor, and then only earlier frames show the incoming direction.
    # If it bends tighter than the car can (1 / R, plus the 15 % allowance), give up.
    if trajectory.max_curvature(new_traj) > (1 + p["curvature_tol"]) / R:
        # Too sharp a bend: give up.
        return None
    # The detour with one frame either side of it.
    joined = np.vstack([P[s - 1:s], frames, P[e + 1:e + 2]])
    # If it comes closer than r_safe (2 m) to an obstacle, give up.
    if geo["obstacles"] is not None and shapely.distance(LineString(joined), geo["obstacles"]) < p["r_safe"]:
        # Too close to an obstacle: give up.
        return None
    # How much longer the detour is than the piece of plan it replaces.
    extra = float(np.sum(np.hypot(*np.diff(new_path, axis=0).T))) - total
    # Hand back the new trajectory and the extra distance.
    return new_traj, extra


# Makes the list of strategies that robot k may choose from, before the game.
# trajs: everyone's trajectories; T: everyone's mission time; s, f, f_end, e: as
# for detour_traj.
def strategies(k, trajs, T, s, f, f_end, e, geo, p):
    """Strategy set of player k, built once before the game: GO, WAIT (if the budget
    leaves room) and DETOUR (if feasible and within the budget)."""
    # P: robot k's plan; dt: 0.5 s.
    P, dt = trajs[k], p["dt"]
    # GO is always allowed: keep the plan; no extra time (dT), no extra distance
    # (dD), no waiting (W). Each strategy is a dictionary of these facts.
    acts = [{"name": GO, "traj": P, "dT": 0.0, "dD": 0.0, "W": 0.0}]
    # If the robot has already finished its mission, GO is the only choice.
    if s >= len(P) - 1:                                    # already finished
        # Hand back just GO.
        return acts
    # WAIT: the shortest stop at frame s with the lowest risk against all other
    # sinks (ideally none), never longer than max_wait or the budget allows.
    # w_cap: the longest wait in frames: at most max_wait (20 s = 40 frames),
    # and at most the time left before the budget B runs out.
    w_cap = min(int(round(p["max_wait"] / dt)), int((p["B"] - T[k]) / dt + EPS))
    # Everyone else's trajectories.
    others = [trajs[j] for j in trajs if j != k]
    # The best wait so far: 0 frames, with infinite risk.
    best_w, best_risk = 0, math.inf
    # Try waiting 1, 2, 3 ... frames.
    for w in range(1, w_cap + 1):
        # The trajectory with that wait.
        cand = wait_traj(P, s, w)
        # Its risk: unsafe steps against every other robot, around the conflict.
        risk = sum(int(np.count_nonzero(unsafe(cand, Q, p)[s:e + w_cap + 1])) for Q in others)
        # Keep it if it is less risky than the best so far ...
        if risk < best_risk:
            # Remember this wait and its risk.
            best_w, best_risk = w, risk
            # ... and stop as soon as a wait has no risk at all: the shortest safe wait.
            if risk == 0:
                # Stop looking: no wait can do better.
                break
    # If any wait was found, add WAIT: extra time and waiting time are both w * dt.
    if best_w:
        # Add the WAIT strategy to the list.
        acts.append({"name": WAIT, "traj": wait_traj(P, s, best_w), "dT": best_w * dt, "dD": 0.0, "W": best_w * dt})
    # DETOUR: keep right like road traffic; try the left side only if right is infeasible.
    for side in (+1, -1):
        # Try the side-step on this side.
        res = detour_traj(P, s, f, f_end, e, side, geo, p)
        # If that side works ...
        if res is not None:
            # ... its extra time is its extra frames times dt ...
            dT = (len(res[0]) - len(P)) * dt
            # ... and it is only allowed if the robot still finishes within B.
            if T[k] + dT <= p["B"] + EPS:                  # Section 0.10, step 6
                # Add the DETOUR strategy to the list.
                acts.append({"name": DETOUR, "traj": res[0], "dT": dT, "dD": res[1], "W": 0.0})
            # Either way, do not try the other side.
            break
    # Hand back the list of strategies.
    return acts


# ---------------------------------------------------------------------------
# The game
# ---------------------------------------------------------------------------

# Plays one game. players: the robots in the conflict; acts: each one's
# strategies; trajs: everyone's trajectories; s: the decision frame.
def play(players, acts, trajs, s, p):
    """Best-response dynamics from all-GO. Returns (profile {player: strategy index}, record)."""
    # E[(k, j)]: a table of the danger between players k and j, for every
    # combination of their strategies (row: k's choice, column: j's choice).
    E = {}
    for i, k in enumerate(players):                        # shared risk of two players
        # ... and every player j after it:
        for j in players[i + 1:]:
            # The danger for every pair of their strategies (a is k's, b is j's).
            tab = np.array([[exposure(a["traj"], b["traj"], s, p) for b in acts[j]] for a in acts[k]])
            # The same table seen from j's side is flipped (.T, "transposed").
            E[(k, j)], E[(j, k)] = tab, tab.T
    # The robots that are not playing keep their plans.
    fixed = [m for m in trajs if m not in players]         # risk against non-players
    # Bk[k][x]: the danger between player k, using strategy x, and all non-players.
    Bk = {k: np.array([sum(exposure(a["traj"], trajs[m], s, p) for m in fixed) for a in acts[k]]) for k in players}
    # The three weights (all 1.0 by default).
    beta, gamma, delta = p["weights"]                      # own cost, in seconds
    # c[k][x]: player k's own cost of strategy x, in seconds: extra time + extra
    # distance / speed + waiting time, each times its weight.
    c = {k: np.array([beta * a["dT"] + gamma * a["dD"] / p["v"] + delta * a["W"] for a in acts[k]]) for k in players}
    # alpha: the price of one second of danger. It is 1 plus the biggest
    # possible own-cost difference of all players, divided by dt, so avoiding
    # even one half-second of danger is worth more than any amount of own cost.
    alpha = (1 + sum(c[k].max() - c[k].min() for k in players)) / p["dt"]

    # Player k's cost when everyone plays the choices in "a" (a dictionary:
    # player -> strategy number).
    def cost(k, a):
        # The danger with the other players, plus the danger with non-players.
        risk = sum(E[(k, j)][a[k], a[j]] for j in players if j != k) + Bk[k][a[k]]
        # Danger times alpha, plus k's own cost.
        return alpha * risk + c[k][a[k]]

    # The "potential" Phi: alpha times the danger of every pair counted ONCE,
    # plus everyone's own cost. When one player switches, Phi changes by exactly
    # as much as that player's own cost changes, so every improving switch
    # makes Phi smaller. Phi cannot fall for ever, so the game must stop.
    def potential(a):
        # The danger between players, each pair counted once.
        shared = sum(E[(k, j)][a[k], a[j]] for i, k in enumerate(players) for j in players[i + 1:])
        # alpha times all the danger, plus everyone's own cost.
        return alpha * (shared + sum(Bk[k][a[k]] for k in players)) + sum(c[k][a[k]] for k in players)

    # Everyone starts with strategy 0, GO.
    a = {k: 0 for k in players}
    # The order in which ties are broken: shorter trajectories first, then by number.
    order = sorted(players, key=lambda k: (len(trajs[k]), k))   # ties: shorter tour moves first
    # moves: the switches made; trace: Phi after each switch, starting now.
    moves, trace = [], [potential(a)]
    # How many different combinations of choices exist (a safety limit).
    n_profiles = int(np.prod([len(acts[k]) for k in players]))
    # Repeat until nobody wants to switch:
    while True:
        best = None                                        # (player, strategy, gain) with the largest gain
        # Look at every player and every strategy it could switch to.
        for k in order:
            # Player k's cost as things stand.
            now = cost(k, a)
            # Every strategy x that k could switch to:
            for x in range(len(acts[k])):
                # How much k would gain by switching to x, with the others unchanged.
                # ({**a, k: x} is a copy of the choices with k's changed to x.)
                gain = now - cost(k, {**a, k: x})
                # Remember the biggest real gain (bigger than the rounding error EPS).
                if gain > EPS and (best is None or gain > best[2] + EPS):
                    # Remember who, which strategy, and how much they gain.
                    best = (k, x, gain)
        # Nobody can gain: the game has reached a Nash equilibrium. Stop.
        if best is None:                                   # nobody can gain: pure Nash equilibrium
            # Leave the loop.
            break
        # Otherwise the player with the biggest gain switches ...
        a[best[0]] = best[1]
        # ... and we write down who switched, to what, and how much they gained ...
        moves.append((best[0], acts[best[0]][best[1]]["name"], float(best[2])))
        # ... and the new Phi.
        trace.append(potential(a))
        # A potential game never needs more switches than there are
        # combinations; if it did, something would be wrong, so stop with an error.
        if len(moves) >= n_profiles:                       # impossible for a potential game
            # Stop with an error message (this should never happen).
            raise RuntimeError("best-response dynamics revisited a profile")
    # A record of the game: who played, when, the price alpha, the switches,
    # the Phi values, each player's options and final choices.
    record = {"players": players, "decision_frame": s, "alpha": alpha, "moves": moves, "potential": trace,
              "options": {k: [x["name"] for x in acts[k]] for k in players},
              "equilibrium": {k: acts[k][a[k]]["name"] for k in players}}
    # For two players, also keep the whole cost table: for each combination,
    # (cost of the first player, cost of the second).
    if len(players) == 2:                                  # payoff (cost) matrix; utility = -cost
        # The two players.
        k, j = players
        # For every row x (k's strategy) and column y (j's strategy): both costs.
        record["payoff"] = [[(float(cost(k, {k: x, j: y})), float(cost(j, {k: x, j: y})))
                             for y in range(len(acts[j]))] for x in range(len(acts[k]))]
    # Hand back the final choices and the record.
    return a, record


# Finds the earliest conflict from frame n0 onwards.
def _first_conflict(trajs, n0, p):
    # The robot numbers in order.
    keys = sorted(trajs)
    # found: for each pair with any conflict from n0 on, its unsafe list.
    found = {}
    # Every robot k ...
    for i, k in enumerate(keys):
        # ... and every robot j after it:
        for j in keys[i + 1:]:
            # Their unsafe steps.
            u = unsafe(trajs[k], trajs[j], p)
            # Any unsafe step from frame n0 on?
            if u[n0:].any():
                # Then remember the pair and its unsafe list.
                found[(k, j)] = u
    # No conflicts left: hand back None and empty answers.
    if not found:
        # Hand back "none found".
        return None, [], {}
    # f: the earliest unsafe frame of any pair (np.argmax finds the first True).
    f = min(n0 + int(np.argmax(u[n0:])) for u in found.values())
    # Hand back f, the pairs that are unsafe exactly at f (these are in the
    # game), and all the unsafe lists.
    return f, [pair for pair, u in found.items() if len(u) > f and u[f]], found


# The main recipe of this file: settle conflicts one by one, from the earliest
# to the latest. trajs, T, L: everyone's trajectories, times and lengths.
# trace: an optional list; if given, every game's details are written into it
# for the animations.
def resolve(trajs, T, L, geo, p, trace=None):
    """Steps 8-10 of Section 0.11: find the earliest conflict, let the sinks involved
    play the game, apply the equilibrium, continue after it. Returns updated copies
    of trajs, T (mission times), L (path lengths) and the list of game records.

    If trace is a list, one entry per game is appended to it for the animations
    (pipeline_gifs.py): {"trajs" (every sink's trajectory just before the game), "acts"
    (each player's strategies, with their trajectories), "f" (frame of the conflict)}.
    It changes nothing else."""
    # Work on copies, so the dictionaries handed in are not changed.
    trajs, T, L = dict(trajs), dict(T), dict(L)
    # h: the side-step distance, 3.5 m.
    h = p["d_safe"] + p["detour_margin"]
    # lead: how many frames before a conflict the players decide, enough for a
    # smooth detour ramp (the same minimum ramp length as in detour_traj,
    # divided by the 2.5 m driven per frame, rounded up, plus 1).
    lead = int(math.ceil(math.pi * math.sqrt(h * p["R"] / 2) / (p["v"] * p["dt"]))) + 1
    # games: the records; n0: look for conflicts from frame 1 on.
    games, n0 = [], 1
    # Keep going, but never more than 500 games.
    while len(games) < p["max_games"]:
        # The earliest conflict from n0 on.
        f, trigger, U = _first_conflict(trajs, n0, p)
        # None left: finished.
        if f is None:
            # Leave the loop: no conflicts are left.
            break
        # The players: every robot in a pair that is unsafe at frame f.
        players = sorted({k for pair in trigger for k in pair})
        # f_end: the frame where the longest of those conflicts ends.
        f_end = f
        for pair in trigger:                               # end of the longest unsafe run starting at f
            # Start at the conflict's first frame.
            n = f
            # Walk forward while the pair stays unsafe.
            while n + 1 < len(U[pair]) and U[pair][n + 1]:
                # One frame further.
                n += 1
            # Keep the latest end over all the pairs.
            f_end = max(f_end, n)
        # s: the decision frame, "lead" frames before the conflict (at least
        # frame 1). e: by when a detour must have rejoined.
        s, e = max(1, f - lead), f_end + lead              # decide `lead` frames before the conflict
        # Every player's list of strategies.
        acts = {k: strategies(k, trajs, T, s, f, f_end, e, geo, p) for k in players}
        # If a trace list was handed in, write this game's starting point into it.
        if trace is not None:
            # Note everyone's trajectories, the strategies, and the conflict frame.
            trace.append({"trajs": dict(trajs), "acts": acts, "f": f})
        # Play the game.
        prof, rec = play(players, acts, trajs, s, p)
        # Each player takes its chosen strategy: new trajectory, new time T,
        # new length L.
        for k in players:
            # The strategy player k chose.
            x = acts[k][prof[k]]
            # Its trajectory replaces the plan; its extra time and distance are added.
            trajs[k], T[k], L[k] = x["traj"], T[k] + x["dT"], L[k] + x["dD"]
        # Note when the conflict was, in seconds.
        rec["time"] = f * p["dt"]
        # Keep the record.
        games.append(rec)
        # Next time, look for conflicts after this one.
        n0 = f + 1
    # Hand back the new trajectories, times, lengths and all the game records.
    return trajs, T, L, games
