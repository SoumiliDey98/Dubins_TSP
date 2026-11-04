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
import math

import numpy as np
import shapely
from shapely.geometry import LineString

import trajectory

GO, WAIT, DETOUR = "GO", "WAIT", "DETOUR"
EPS = 1e-9


# ---------------------------------------------------------------------------
# Conflict detection
# ---------------------------------------------------------------------------

def closest(P, Q, p):
    """For every interval [frame n, frame n+1] while both sinks are on their mission:
    the closest distance (straight-line motion between frames) and whether both
    sinks are inside the docking zone around the base at that closest moment."""
    m = min(len(P), len(Q))
    if m < 2:
        return np.zeros(0), np.zeros(0, dtype=bool)
    r0 = P[:m - 1] - Q[:m - 1]
    dr = (P[1:m] - Q[1:m]) - r0
    den = np.einsum("ij,ij->i", dr, dr)
    tau = np.clip(-np.einsum("ij,ij->i", r0, dr) / np.where(den > EPS, den, 1.0), 0.0, 1.0)
    d = np.hypot(*(r0 + tau[:, None] * dr).T)
    base = np.asarray(p["base"])
    pc = P[:m - 1] + tau[:, None] * (P[1:m] - P[:m - 1])
    qc = Q[:m - 1] + tau[:, None] * (Q[1:m] - Q[:m - 1])
    dock = (np.hypot(*(pc - base).T) <= p["base_radius"]) & (np.hypot(*(qc - base).T) <= p["base_radius"])
    return d, dock


def unsafe(P, Q, p):
    """u[n] = True if the two sinks break eq. (20) between frames n and n+1."""
    d, dock = closest(P, Q, p)
    return (d < p["d_safe"]) & ~dock


def exposure(P, Q, start, p):
    """Collision risk of a pair: seconds closer than d_safe from frame `start` on."""
    return p["dt"] * float(np.count_nonzero(unsafe(P, Q, p)[start:]))


def summary(trajs, p):
    """Conflicting pairs, total exposure [s] and closest approach [m] (outside the docking zone)."""
    keys = sorted(trajs)
    pairs, total, closest_d = [], 0.0, math.inf
    for i, k in enumerate(keys):
        for j in keys[i + 1:]:
            d, dock = closest(trajs[k], trajs[j], p)
            bad = (d < p["d_safe"]) & ~dock
            if bad.any():
                pairs.append((k, j))
                total += p["dt"] * float(np.count_nonzero(bad))
            if (~dock).any():
                closest_d = min(closest_d, float(d[~dock].min()))
    return {"pairs": pairs, "exposure": total, "closest": closest_d}


# ---------------------------------------------------------------------------
# Strategies: each one is a complete new trajectory for one sink
# ---------------------------------------------------------------------------

def wait_traj(P, s, w):
    """WAIT: stop at the position of frame s for w extra frames, then continue unchanged."""
    return np.vstack([P[:s + 1], np.repeat(P[s:s + 1], w, axis=0), P[s + 1:]])


def _resample(points, step):
    seg = np.hypot(*np.diff(points, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(1, int(math.ceil(cum[-1] / step - 1e-9)))
    s = np.minimum(np.arange(n + 1) * step, cum[-1])
    return np.column_stack([np.interp(s, cum, points[:, 0]), np.interp(s, cum, points[:, 1])])


def detour_traj(P, s, f, f_end, e, side, geo, p):
    """DETOUR: side-step h = d_safe + detour_margin to the right (side=+1) or left (-1)
    of the own path during frames f..f_end, rejoining by frame e. Returns
    (trajectory, extra distance) or None if not feasible: the sink stands still
    during s..f_end, the ramps are too short, the turn is sharper than R allows,
    or the new path comes closer than r_safe to an obstacle."""
    h, R = p["d_safe"] + p["detour_margin"], p["R"]
    e = min(e, len(P) - 1)
    if s < 1 or not (s < f <= f_end < e):
        return None
    steps = np.hypot(*np.diff(P[s:e + 1], axis=0).T)
    still = steps < 1e-9
    stops = np.flatnonzero(still[1:] & still[:-1])        # stopped for 2+ frames
    if stops.size:
        if s + stops[0] <= f_end:
            return None                                    # cannot side-step while standing still
        e = s + int(stops[0])                              # rejoin before the next stop
        steps, still = steps[:e - s], still[:e - s]
    window = P[s:e + 1]
    cum = np.concatenate([[0.0], np.cumsum(steps)])
    keep = np.concatenate([[True], ~still])
    Q, sig = window[keep], cum[keep]
    a, b, total = cum[f - s], cum[f_end - s], cum[-1]
    ramp_min = math.pi * math.sqrt(h * R / 2)             # cosine ramp with curvature <= 1/R
    if a < ramp_min or total - b < ramp_min:
        return None
    prof = np.ones_like(sig)
    up, down = sig < a, sig > b
    prof[up] = 0.5 * (1 - np.cos(np.pi * sig[up] / a))
    prof[down] = 0.5 * (1 - np.cos(np.pi * (total - sig[down]) / (total - b)))
    tan = np.gradient(Q, axis=0)
    tan /= np.maximum(np.hypot(*tan.T)[:, None], EPS)
    new_path = Q + side * h * prof[:, None] * np.column_stack([tan[:, 1], -tan[:, 0]])
    frames = _resample(new_path, p["v"] * p["dt"])
    short = (e - s + 1) - len(frames)                      # never faster than GO
    if short > 0:
        frames = np.vstack([frames, np.repeat(frames[-1:], short, axis=0)])
    new_traj = np.vstack([P[:s], frames, P[e + 1:]])
    # Curvature over the whole trajectory, as the final check measures it: frames next to the
    # detour may be parked at a sensor, and then only earlier frames show the incoming direction.
    if trajectory.max_curvature(new_traj) > (1 + p["curvature_tol"]) / R:
        return None
    joined = np.vstack([P[s - 1:s], frames, P[e + 1:e + 2]])
    if geo["obstacles"] is not None and shapely.distance(LineString(joined), geo["obstacles"]) < p["r_safe"]:
        return None
    extra = float(np.sum(np.hypot(*np.diff(new_path, axis=0).T))) - total
    return new_traj, extra


def strategies(k, trajs, T, s, f, f_end, e, geo, p):
    """Strategy set of player k, built once before the game: GO, WAIT (if the budget
    leaves room) and DETOUR (if feasible and within the budget)."""
    P, dt = trajs[k], p["dt"]
    acts = [{"name": GO, "traj": P, "dT": 0.0, "dD": 0.0, "W": 0.0}]
    if s >= len(P) - 1:                                    # already finished
        return acts
    # WAIT: the shortest stop at frame s with the lowest risk against all other
    # sinks (ideally none), never longer than max_wait or the budget allows.
    w_cap = min(int(round(p["max_wait"] / dt)), int((p["B"] - T[k]) / dt + EPS))
    others = [trajs[j] for j in trajs if j != k]
    best_w, best_risk = 0, math.inf
    for w in range(1, w_cap + 1):
        cand = wait_traj(P, s, w)
        risk = sum(int(np.count_nonzero(unsafe(cand, Q, p)[s:e + w_cap + 1])) for Q in others)
        if risk < best_risk:
            best_w, best_risk = w, risk
            if risk == 0:
                break
    if best_w:
        acts.append({"name": WAIT, "traj": wait_traj(P, s, best_w), "dT": best_w * dt, "dD": 0.0, "W": best_w * dt})
    # DETOUR: keep right like road traffic; try the left side only if right is infeasible.
    for side in (+1, -1):
        res = detour_traj(P, s, f, f_end, e, side, geo, p)
        if res is not None:
            dT = (len(res[0]) - len(P)) * dt
            if T[k] + dT <= p["B"] + EPS:                  # Section 0.10, step 6
                acts.append({"name": DETOUR, "traj": res[0], "dT": dT, "dD": res[1], "W": 0.0})
            break
    return acts


# ---------------------------------------------------------------------------
# The game
# ---------------------------------------------------------------------------

def play(players, acts, trajs, s, p):
    """Best-response dynamics from all-GO. Returns (profile {player: strategy index}, record)."""
    E = {}
    for i, k in enumerate(players):                        # shared risk of two players
        for j in players[i + 1:]:
            tab = np.array([[exposure(a["traj"], b["traj"], s, p) for b in acts[j]] for a in acts[k]])
            E[(k, j)], E[(j, k)] = tab, tab.T
    fixed = [m for m in trajs if m not in players]         # risk against non-players
    Bk = {k: np.array([sum(exposure(a["traj"], trajs[m], s, p) for m in fixed) for a in acts[k]]) for k in players}
    beta, gamma, delta = p["weights"]                      # own cost, in seconds
    c = {k: np.array([beta * a["dT"] + gamma * a["dD"] / p["v"] + delta * a["W"] for a in acts[k]]) for k in players}
    alpha = (1 + sum(c[k].max() - c[k].min() for k in players)) / p["dt"]

    def cost(k, a):
        risk = sum(E[(k, j)][a[k], a[j]] for j in players if j != k) + Bk[k][a[k]]
        return alpha * risk + c[k][a[k]]

    def potential(a):
        shared = sum(E[(k, j)][a[k], a[j]] for i, k in enumerate(players) for j in players[i + 1:])
        return alpha * (shared + sum(Bk[k][a[k]] for k in players)) + sum(c[k][a[k]] for k in players)

    a = {k: 0 for k in players}
    order = sorted(players, key=lambda k: (len(trajs[k]), k))   # ties: shorter tour moves first
    moves, trace = [], [potential(a)]
    n_profiles = int(np.prod([len(acts[k]) for k in players]))
    while True:
        best = None                                        # (player, strategy, gain) with the largest gain
        for k in order:
            now = cost(k, a)
            for x in range(len(acts[k])):
                gain = now - cost(k, {**a, k: x})
                if gain > EPS and (best is None or gain > best[2] + EPS):
                    best = (k, x, gain)
        if best is None:                                   # nobody can gain: pure Nash equilibrium
            break
        a[best[0]] = best[1]
        moves.append((best[0], acts[best[0]][best[1]]["name"], float(best[2])))
        trace.append(potential(a))
        if len(moves) >= n_profiles:                       # impossible for a potential game
            raise RuntimeError("best-response dynamics revisited a profile")
    record = {"players": players, "decision_frame": s, "alpha": alpha, "moves": moves, "potential": trace,
              "options": {k: [x["name"] for x in acts[k]] for k in players},
              "equilibrium": {k: acts[k][a[k]]["name"] for k in players}}
    if len(players) == 2:                                  # payoff (cost) matrix; utility = -cost
        k, j = players
        record["payoff"] = [[(float(cost(k, {k: x, j: y})), float(cost(j, {k: x, j: y})))
                             for y in range(len(acts[j]))] for x in range(len(acts[k]))]
    return a, record


def _first_conflict(trajs, n0, p):
    keys = sorted(trajs)
    found = {}
    for i, k in enumerate(keys):
        for j in keys[i + 1:]:
            u = unsafe(trajs[k], trajs[j], p)
            if u[n0:].any():
                found[(k, j)] = u
    if not found:
        return None, [], {}
    f = min(n0 + int(np.argmax(u[n0:])) for u in found.values())
    return f, [pair for pair, u in found.items() if len(u) > f and u[f]], found


def resolve(trajs, T, L, geo, p):
    """Steps 8-10 of Section 0.11: find the earliest conflict, let the sinks involved
    play the game, apply the equilibrium, continue after it. Returns updated copies
    of trajs, T (mission times), L (path lengths) and the list of game records."""
    trajs, T, L = dict(trajs), dict(T), dict(L)
    h = p["d_safe"] + p["detour_margin"]
    lead = int(math.ceil(math.pi * math.sqrt(h * p["R"] / 2) / (p["v"] * p["dt"]))) + 1
    games, n0 = [], 1
    while len(games) < p["max_games"]:
        f, trigger, U = _first_conflict(trajs, n0, p)
        if f is None:
            break
        players = sorted({k for pair in trigger for k in pair})
        f_end = f
        for pair in trigger:                               # end of the longest unsafe run starting at f
            n = f
            while n + 1 < len(U[pair]) and U[pair][n + 1]:
                n += 1
            f_end = max(f_end, n)
        s, e = max(1, f - lead), f_end + lead              # decide `lead` frames before the conflict
        acts = {k: strategies(k, trajs, T, s, f, f_end, e, geo, p) for k in players}
        prof, rec = play(players, acts, trajs, s, p)
        for k in players:
            x = acts[k][prof[k]]
            trajs[k], T[k], L[k] = x["traj"], T[k] + x["dT"], L[k] + x["dD"]
        rec["time"] = f * p["dt"]
        games.append(rec)
        n0 = f + 1
    return trajs, T, L, games
