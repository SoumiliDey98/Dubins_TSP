"""
corridor_game_gt.py
===================
Game-theoretic conflict resolution for multiple mobile sinks (MS) in the
Dubins-TSP project. It is a drop-in alternative to corridor_game.py, which is
left unchanged:

    from corridor_game_gt import game_theory_resolve
    all_trajectories = game_theory_resolve(env, all_trajectories, safe_dist=3.0)

How it works (equation numbers refer to Corridor_Game_Theory.pdf)
-----------------------------------------------------------------
1. Scan the trajectories for the earliest interval in which two sinks are
   closer than safe_dist (Eq. 1).
2. The sinks that are too close in that interval are the players of one game.
3. Each player gets the strategies GO, WAIT and, only if it is feasible,
   DETOUR. Each strategy is a complete new trajectory for that player only.
4. A player's cost (Eq. 5) adds collision risk (Eq. 2-3) to its extra travel
   time, extra distance and waiting time (Eq. 4).
5. Players switch to their best response (Eq. 6) one at a time until no one
   can lower their own cost: a pure Nash equilibrium (Eq. 7). This always
   stops, because the game has an exact potential function (Eq. 8, Theorem 1).
6. The equilibrium trajectories are applied and the scan moves on.

Running this file prints the payoff matrices of two small two-sink examples.
"""

import math

import numpy as np
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

GO, WAIT, DETOUR = "GO", "WAIT", "DETOUR"
_EPS = 1e-9


# ---------------------------------------------------------------------------
# 1. Conflict geometry
# ---------------------------------------------------------------------------

def _padded(P, n):
    """Extend trajectory P to n frames; a finished sink stays where it ended."""
    if len(P) >= n:
        return P
    return np.vstack([P, np.repeat(P[-1:], n - len(P), axis=0)])


def _closest_per_interval(P, Q, base, base_radius):
    """For every interval [frame n, frame n+1] return the closest distance between
    sinks P and Q (both move in a straight line between frames) and whether both
    are inside the base-station zone at that closest moment."""
    m = max(len(P), len(Q))
    P, Q = _padded(P, m), _padded(Q, m)
    r0 = P[:-1] - Q[:-1]                    # relative position at frame n
    dr = (P[1:] - Q[1:]) - r0               # its change until frame n+1
    den = np.einsum('ij,ij->i', dr, dr)
    tau = np.clip(-np.einsum('ij,ij->i', r0, dr) / np.where(den > _EPS, den, 1.0), 0.0, 1.0)
    d_min = np.linalg.norm(r0 + tau[:, None] * dr, axis=1)
    p_c = P[:-1] + tau[:, None] * (P[1:] - P[:-1])     # positions at the closest moment
    q_c = Q[:-1] + tau[:, None] * (Q[1:] - Q[:-1])
    near_base = (np.linalg.norm(p_c - base, axis=1) <= base_radius) & \
                (np.linalg.norm(q_c - base, axis=1) <= base_radius)
    return d_min, near_base


def unsafe_intervals(P, Q, base, safe_dist, base_radius):
    """u[n] = True if sinks P and Q come closer than safe_dist between frames
    n and n+1 (Eq. 1). Checking the whole interval, not only the sampled frames,
    also catches sinks that pass each other between two frames. As in
    corridor_game.py, sinks that are both near the base station are exempt."""
    if max(len(P), len(Q)) < 2:
        return np.zeros(0, dtype=bool)
    d_min, near_base = _closest_per_interval(P, Q, base, base_radius)
    return (d_min < safe_dist) & ~near_base


def exposure(P, Q, start, dt, base, safe_dist, base_radius):
    """Collision-risk exposure E (Eq. 2): seconds that sinks P and Q spend closer
    than safe_dist from frame `start` on. Symmetric: exposure(P, Q) == exposure(Q, P)."""
    u = unsafe_intervals(P, Q, base, safe_dist, base_radius)
    return dt * float(np.count_nonzero(u[start:]))


def conflict_summary(env, trajectories, safe_dist=3.0, dt=0.5, base_radius=5.0):
    """Independent check of a set of trajectories (used for the before/after report)."""
    base = np.asarray(env.base_station, dtype=float)
    P = {k: np.asarray(t, dtype=float).reshape(-1, 2) for k, t in trajectories.items() if len(t) >= 2}
    keys = sorted(P)
    pairs, total, closest = 0, 0.0, math.inf
    for i, k in enumerate(keys):
        for j in keys[i + 1:]:
            d_min, near = _closest_per_interval(P[k], P[j], base, base_radius)
            unsafe = (d_min < safe_dist) & ~near
            if unsafe.any():
                pairs += 1
                total += dt * float(np.count_nonzero(unsafe))
            if (~near).any():
                closest = min(closest, float(d_min[~near].min()))
    return {"conflicting_pairs": pairs, "exposure_s": total, "min_separation": closest}


# ---------------------------------------------------------------------------
# 2. Strategies: each one is a complete new trajectory for one sink
# ---------------------------------------------------------------------------

class Action:
    """One strategy of one sink and the ingredients of its own cost (Eq. 4)."""

    def __init__(self, name, traj, extra_time, extra_dist, wait_time, note=""):
        self.name = name                # GO, WAIT or DETOUR
        self.traj = traj                # (n, 2) array, one position per frame
        self.extra_time = extra_time    # Delta T_k: extra mission time [s]
        self.extra_dist = extra_dist    # Delta D_k: extra distance [m]
        self.wait_time = wait_time      # W_k: time spent standing still [s]
        self.note = note


def wait_trajectory(P, s, w):
    """WAIT: stop at the position of frame s for w extra frames, then follow the
    original path unchanged (everything after frame s happens w frames later)."""
    return np.vstack([P[:s + 1], np.repeat(P[s:s + 1], w, axis=0), P[s + 1:]])


def _length(points):
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _dedup(points):
    keep = np.concatenate([[True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9])
    return points[keep]


def _resample(points, step):
    """Points `step` metres apart along a polyline (constant speed), ending at its last point."""
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(1, int(math.ceil(cum[-1] / step - 1e-9)))
    s = np.minimum(np.arange(n + 1) * step, cum[-1])
    return np.column_stack([np.interp(s, cum, points[:, 0]), np.interp(s, cum, points[:, 1])])


def _max_curvature(points):
    """Largest discrete curvature (1 / radius of the circle through three consecutive points)."""
    if len(points) < 3:
        return 0.0
    a = np.linalg.norm(points[1:-1] - points[:-2], axis=1)
    b = np.linalg.norm(points[2:] - points[1:-1], axis=1)
    c = np.linalg.norm(points[2:] - points[:-2], axis=1)
    u, v = points[1:-1] - points[:-2], points[2:] - points[:-2]
    twice_area = np.abs(u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0])
    return float(np.max(2.0 * twice_area / np.maximum(a * b * c, _EPS)))


def detour_trajectory(P, s, f, f_end, e, side, offset, R, step, curvature_tol, blocked):
    """DETOUR: side-step `offset` metres to the right (side=+1) or left (side=-1)
    of the sink's own path while the conflict lasts (frames f..f_end), then rejoin
    the path by frame e. Returns (trajectory, extra_distance) or None when the
    side-step is not feasible:
      - the sink is parked or collecting data during frames s..f_end,
      - there is not enough room before/after the conflict for a gentle ramp,
      - the new path turns more sharply than the minimum turning radius R,
      - the new path crosses an obstacle's safety boundary.
    """
    e = min(e, len(P) - 1)
    if s < 1 or not (s < f <= f_end < e):
        return None
    steps = np.linalg.norm(np.diff(P[s:e + 1], axis=0), axis=1)
    still = steps < 1e-9
    stops = np.flatnonzero(still[1:] & still[:-1])     # stopped for 2+ frames: parked or dwelling
    if stops.size:                                      # (one repeated frame is a path-joint artefact)
        if s + stops[0] <= f_end:
            return None                     # cannot side-step while standing still
        e = s + int(stops[0])               # rejoin the path before the next stop
        steps, still = steps[:e - s], still[:e - s]
    window = P[s:e + 1]
    cum = np.concatenate([[0.0], np.cumsum(steps)])     # arc length at each frame of the window
    keep = np.concatenate([[True], ~still])
    Q, sigma = window[keep], cum[keep]
    a, b, total = cum[f - s], cum[f_end - s], cum[-1]
    ramp_min = math.pi * math.sqrt(offset * R / 2.0)    # shortest cosine ramp with curvature <= 1/R
    if a < ramp_min or total - b < ramp_min:
        return None
    # Offset profile: smooth ramp out, full offset during the conflict, smooth ramp back.
    prof = np.ones_like(sigma)
    up, down = sigma < a, sigma > b
    prof[up] = 0.5 * (1.0 - np.cos(np.pi * sigma[up] / a))
    prof[down] = 0.5 * (1.0 - np.cos(np.pi * (total - sigma[down]) / (total - b)))
    tangent = np.gradient(Q, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), _EPS)
    right = np.column_stack([tangent[:, 1], -tangent[:, 0]])
    new_path = Q + side * offset * prof[:, None] * right
    # Feasibility 1: turning radius (small tolerance for sampling, as in the original paths).
    joined = _dedup(np.vstack([P[s - 1:s], new_path, P[e + 1:e + 2]]))
    if _max_curvature(joined) > (1.0 + curvature_tol) / R:
        return None
    # Feasibility 2: stay outside every obstacle's safety boundary.
    if blocked is not None and LineString(new_path).intersects(blocked):
        return None
    # Re-time at constant speed. The original window may contain short pauses at path
    # joints (repeated frames); keep that many frames so a side-step is never faster than GO.
    frames = _resample(new_path, step)
    short = (e - s + 1) - len(frames)
    if short > 0:
        frames = np.vstack([frames, np.repeat(frames[-1:], short, axis=0)])
    traj = np.vstack([P[:s], frames, P[e + 1:]])
    return traj, _length(new_path) - total


def _build_actions(k, trajs, s, f, f_end, e, p, blocked):
    """Strategy set A_k = {GO, WAIT, DETOUR (if feasible)} of player k. It is built
    once, before the game starts, and does not change during the game."""
    P, dt = trajs[k], p["dt"]
    actions = [Action(GO, P, 0.0, 0.0, 0.0, "keep the planned trajectory")]
    if s >= len(P) - 1:                     # this sink has already finished its tour
        return actions

    # WAIT: the shortest stop at frame s that gives the lowest risk against all
    # other sinks, assuming they keep their current trajectories (ideally zero risk).
    others = [trajs[j] for j in trajs if j != k]
    end = e + p["w_max"] + 1
    best_w, best_risk = 1, math.inf
    for w in range(1, p["w_max"] + 1):
        cand = wait_trajectory(P, s, w)
        risk = sum(int(np.count_nonzero(unsafe_intervals(cand, Q, p["base"], p["safe_dist"],
                                                         p["base_radius"])[s:end])) for Q in others)
        if risk < best_risk:
            best_w, best_risk = w, risk
            if risk == 0:
                break
    actions.append(Action(WAIT, wait_trajectory(P, s, best_w), best_w * dt, 0.0, best_w * dt,
                          f"stop {best_w * dt:.1f} s at frame {s}"))

    # DETOUR: keep right like road traffic; try the left side only if right is infeasible.
    for side, label in ((+1, "right"), (-1, "left")):
        res = detour_trajectory(P, s, f, f_end, e, side, p["offset"], p["R"], p["step"],
                                p["curvature_tol"], blocked)
        if res is not None:
            traj, extra_dist = res
            actions.append(Action(DETOUR, traj, (len(traj) - len(P)) * dt, extra_dist, 0.0,
                                  f"side-step {p['offset']:.1f} m to the {label}"))
            break

    if p["budget"] is not None:             # optional hard budget: (n - 1) * dt <= B
        actions = [a for a in actions if a.name == GO or (len(a.traj) - 1) * dt <= p["budget"]]
    return actions


# ---------------------------------------------------------------------------
# 3. The game
# ---------------------------------------------------------------------------

class Game:
    """One conflict game G = (players, strategy sets, costs). A profile is a dict
    {sink: index of its chosen strategy}."""

    def __init__(self, players, actions, trajs, s, p):
        self.players, self.actions, self.s = players, actions, s
        dt, geo = p["dt"], (p["base"], p["safe_dist"], p["base_radius"])
        # Shared risk between two players for every pair of strategies (symmetric).
        self.pair = {}
        for i, k in enumerate(players):
            for j in players[i + 1:]:
                tab = np.array([[exposure(a.traj, b.traj, s, dt, *geo) for b in actions[j]]
                                for a in actions[k]])
                self.pair[(k, j)], self.pair[(j, k)] = tab, tab.T
        # Risk against sinks that are not playing; their trajectories stay fixed.
        fixed = [m for m in trajs if m not in players]
        self.background = {k: np.array([sum(exposure(a.traj, trajs[m], s, dt, *geo) for m in fixed)
                                        for a in actions[k]]) for k in players}
        # Own cost c_k = beta * dT + gamma * dD / v + delta * W, in seconds (Eq. 4).
        beta, gamma, delta = p["weights"]
        self.own = {k: np.array([beta * a.extra_time + gamma * a.extra_dist / p["v"] + delta * a.wait_time
                                 for a in actions[k]]) for k in players}
        # Collision weight alpha: by default just large enough that one interval
        # (dt seconds) of risk outweighs every possible saving in the other terms.
        span = sum(float(c.max() - c.min()) for c in self.own.values())
        self.alpha = p["collision_weight"] if p["collision_weight"] is not None else (1.0 + span) / dt
        self.safety_first = self.alpha * dt > span

    def risk(self, k, prof):
        """R_k(a): collision-risk exposure of player k (Eq. 3)."""
        shared = sum(self.pair[(k, j)][prof[k], prof[j]] for j in self.players if j != k)
        return shared + self.background[k][prof[k]]

    def cost(self, k, prof):
        """J_k(a) = alpha * R_k(a) + c_k(a_k)   (Eq. 5)."""
        return self.alpha * self.risk(k, prof) + self.own[k][prof[k]]

    def potential(self, prof):
        """Phi(a): every shared pair term once, plus every player's own terms (Eq. 8)."""
        shared = sum(self.pair[(k, j)][prof[k], prof[j]]
                     for i, k in enumerate(self.players) for j in self.players[i + 1:])
        alone = sum(self.background[k][prof[k]] for k in self.players)
        return self.alpha * (shared + alone) + sum(self.own[k][prof[k]] for k in self.players)

    def n_profiles(self):
        return int(np.prod([len(self.actions[k]) for k in self.players]))


def best_response(game, k, prof):
    """Eq. 6: the strategy with the lowest cost for k, given the others' strategies.
    Ties keep the current strategy (then prefer GO, WAIT, DETOUR in that order)."""
    best, best_cost = prof[k], game.cost(k, prof)
    for i in range(len(game.actions[k])):
        c = game.cost(k, {**prof, k: i})
        if c < best_cost - _EPS:
            best, best_cost = i, c
    return best, best_cost


def best_response_dynamics(game, order):
    """Start with everyone on GO. In every step each player computes its best
    response; the player that would gain the most switches (ties: first in `order`).
    Stops when nobody can gain, i.e. at a pure Nash equilibrium (Eq. 7)."""
    prof = {k: 0 for k in game.players}
    moves, trace = [], [game.potential(prof)]
    while True:
        mover = None
        for k in order:
            b, c = best_response(game, k, prof)
            gain = game.cost(k, prof) - c
            if gain > _EPS and (mover is None or gain > mover[2] + _EPS):
                mover = (k, b, gain)
        if mover is None:
            return prof, moves, trace
        k, b, gain = mover
        prof[k] = b
        moves.append((k, game.actions[k][b].name, float(gain)))
        trace.append(game.potential(prof))
        # Theorem 1: Phi falls with every move, so no profile can repeat.
        if len(moves) >= game.n_profiles():
            raise RuntimeError("best-response dynamics revisited a profile; potential property violated")


def is_nash(game, prof):
    """Eq. 7: no player can lower its own cost by changing only its own strategy."""
    return all(game.cost(k, {**prof, k: i}) >= game.cost(k, prof) - _EPS
               for k in game.players for i in range(len(game.actions[k])))


def payoff_matrix(game):
    """Two-player games only. Returns (row_names, col_names, J, Phi): J[r][c] is the
    pair (J_row_sink, J_col_sink) of costs when the row sink plays r and the column
    sink plays c. Utility (payoff) = -cost."""
    k, j = game.players
    rows = [a.name for a in game.actions[k]]
    cols = [a.name for a in game.actions[j]]
    J = [[(game.cost(k, {k: r, j: c}), game.cost(j, {k: r, j: c})) for c in range(len(cols))]
         for r in range(len(rows))]
    Phi = [[game.potential({k: r, j: c}) for c in range(len(cols))] for r in range(len(rows))]
    return rows, cols, J, Phi


def format_payoff_matrix(game, prof=None):
    """Text table of the two-player cost matrix; the chosen equilibrium is marked with *."""
    k, j = game.players
    rows, cols, J, Phi = payoff_matrix(game)
    lines = [f"Costs (J_{k}, J_{j}) in seconds; rows = MS {k}, columns = MS {j}; utility = -cost",
             f"{'':>9}" + "".join(f"{c:>22}" for c in cols)]
    for r, name in enumerate(rows):
        cells = []
        for c in range(len(cols)):
            mark = "*" if prof is not None and prof[k] == r and prof[j] == c else " "
            cells.append(f"{mark}({J[r][c][0]:8.2f}, {J[r][c][1]:8.2f})")
        lines.append(f"{name:>9}" + "".join(f"{x:>22}" for x in cells))
    lines.append("Potential Phi:")
    for r, name in enumerate(rows):
        lines.append(f"{name:>9}" + "".join(f"{Phi[r][c]:>22.2f}" for c in range(len(cols))))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 4. Driver: same interface as corridor_game.game_theory_resolve
# ---------------------------------------------------------------------------

def _first_conflict(trajs, n0, p):
    """Earliest unsafe interval f >= n0, the pairs that are unsafe in it, and their u[] arrays."""
    keys = sorted(trajs)
    unsafe = {}
    for i, k in enumerate(keys):
        for j in keys[i + 1:]:
            u = unsafe_intervals(trajs[k], trajs[j], p["base"], p["safe_dist"], p["base_radius"])
            if u[n0:].any():
                unsafe[(k, j)] = u
    if not unsafe:
        return None, [], {}
    f = min(n0 + int(np.argmax(u[n0:])) for u in unsafe.values())
    trigger = [pair for pair, u in unsafe.items() if len(u) > f and u[f]]
    return f, trigger, unsafe


def _run_end(u, n):
    """Last index of the run of True values in u that starts at n."""
    while n + 1 < len(u) and u[n + 1]:
        n += 1
    return n


def game_theory_resolve(env, all_trajectories, safe_dist=3.0, dt=0.5, weights=(1.0, 1.0, 1.0),
                        collision_weight=None, max_wait=20.0, detour_margin=0.5, base_radius=5.0,
                        enforce_budget=False, max_games=500, verbose=True, return_report=False):
    """Resolve inter-sink conflicts with a GO / WAIT / DETOUR best-response game.

    env              : Environment (uses base_station, v, R, d_safe, obstacles, budget)
    all_trajectories : {sink id: [(x, y), ...]}, one position every dt seconds
    safe_dist        : minimum allowed distance between two sinks [m]
    dt               : time between frames [s] (visualize.py uses 0.5)
    weights          : (beta, gamma, delta) for extra time, extra distance, waiting time
    collision_weight : alpha; None = automatic "safety first" value (see Game)
    max_wait         : longest stop a WAIT strategy may use [s]
    detour_margin    : a DETOUR side-steps safe_dist + detour_margin metres
    base_radius      : two sinks both this close to the base station (at their closest moment) are exempt
    enforce_budget   : if True, WAIT/DETOUR are only offered if (n - 1) * dt <= env.budget
    max_games        : upper limit on the number of games (guarantees termination)
    return_report    : if True, return (trajectories, report) instead of trajectories

    The input lists are not modified; a new dict of lists of (x, y) is returned.
    """
    if verbose:
        print("Resolving collisions using Game Theory (GO / WAIT / DETOUR best-response game)...")
    p = {"dt": dt, "safe_dist": safe_dist, "base_radius": base_radius, "weights": tuple(weights),
         "collision_weight": collision_weight, "base": np.asarray(env.base_station, dtype=float),
         "v": float(env.v), "R": float(env.R), "step": float(env.v) * dt,
         "offset": safe_dist + detour_margin, "curvature_tol": 0.15,
         "w_max": max(1, int(round(max_wait / dt))),
         "budget": getattr(env, "budget", None) if enforce_budget else None}
    ramp_min = math.pi * math.sqrt(p["offset"] * p["R"] / 2.0)
    lead = int(math.ceil(ramp_min / p["step"])) + 1     # decide this many frames before the conflict
    obstacles = [Polygon(o.vertices).buffer(env.d_safe) for o in getattr(env, "obstacles", [])]
    blocked = unary_union(obstacles) if obstacles else None

    out = {k: list(t) for k, t in all_trajectories.items()}
    trajs = {k: np.asarray(t, dtype=float).reshape(-1, 2) for k, t in all_trajectories.items() if len(t) >= 2}
    before = conflict_summary(env, trajs, safe_dist, dt, base_radius)

    games, n0, hit_limit = [], 1, False
    while len(trajs) >= 2:
        if len(games) >= max_games:
            hit_limit = True
            break
        f, trigger, unsafe = _first_conflict(trajs, n0, p)
        if f is None:
            break
        players = sorted({k for pair in trigger for k in pair})
        f_end = max(_run_end(unsafe[pair], f) for pair in trigger)
        s, e = max(1, f - lead), f_end + lead
        actions = {k: _build_actions(k, trajs, s, f, f_end, e, p, blocked) for k in players}
        game = Game(players, actions, trajs, s, p)
        # Tie-break between equal gains: the sink with the shorter tour (more spare time) moves first.
        order = sorted(players, key=lambda k: (len(trajs[k]), k))
        prof, moves, trace = best_response_dynamics(game, order)
        for k in players:
            trajs[k] = actions[k][prof[k]].traj

        # Was the conflict that started this game removed (frames s..f)? Either way
        # the scan continues at f + 1, so anything left over starts a new game.
        resolved = not any(unsafe_intervals(trajs[k], trajs[j], p["base"], safe_dist,
                                            base_radius)[s:f + 1].any() for (k, j) in trigger)
        n0 = f + 1

        record = {"frame": f, "time": f * dt, "players": players, "decision_frame": s,
                  "strategies": {k: [(a.name, a.extra_time, a.extra_dist, a.wait_time, a.note)
                                     for a in actions[k]] for k in players},
                  "equilibrium": {k: actions[k][prof[k]].name for k in players},
                  "moves": moves, "potential_trace": trace, "is_nash": is_nash(game, prof),
                  "alpha": game.alpha, "safety_first": game.safety_first,
                  "risk_left": sum(game.risk(k, prof) for k in players), "resolved": resolved}
        if len(players) == 2:
            record["payoff_matrix"] = payoff_matrix(game)
            record["payoff_text"] = format_payoff_matrix(game, prof)
        games.append(record)
        if verbose == 2:
            print(f"  game {len(games)} at t={f * dt:.1f}s players={players} -> {record['equilibrium']}"
                  f"{'' if resolved else '  (conflict remains)'}")

    for k in trajs:
        out[k] = [(float(x), float(y)) for x, y in trajs[k]]
    after = conflict_summary(env, trajs, safe_dist, dt, base_radius)
    budget = getattr(env, "budget", None)
    over = (lambda T: sum((len(t) - 1) * dt > budget for t in T.values())) if budget is not None else (lambda T: None)
    report = {"before": before, "after": after, "games": games, "n_games": len(games),
              "resolved": sum(g["resolved"] for g in games), "hit_max_games": hit_limit,
              "all_nash": all(g["is_nash"] for g in games),
              "total_wait_s": sum(a[3] for g in games for k, n in g["equilibrium"].items()
                                  for a in g["strategies"][k] if a[0] == n),
              "total_extra_dist_m": sum(a[2] for g in games for k, n in g["equilibrium"].items()
                                        for a in g["strategies"][k] if a[0] == n),
              "over_budget_before": over({k: np.asarray(t) for k, t in all_trajectories.items() if len(t)}),
              "over_budget_after": over(trajs)}
    if verbose:
        print(f"Game theory: {len(games)} games, {report['resolved']} conflicts removed, "
              f"all equilibria verified: {report['all_nash']}.")
        print(f"  conflicting pairs {before['conflicting_pairs']} -> {after['conflicting_pairs']}, "
              f"risk exposure {before['exposure_s']:.1f}s -> {after['exposure_s']:.1f}s, "
              f"closest approach {before['min_separation']:.2f} -> {after['min_separation']:.2f} m")
        if hit_limit:
            print(f"  stopped after max_games={max_games}; later conflicts were not examined.")
    return (out, report) if return_report else out


# ---------------------------------------------------------------------------
# 5. Small examples: two sinks, printed payoff matrices
# ---------------------------------------------------------------------------

def _straight(p0, p1, step):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = int(round(np.linalg.norm(p1 - p0) / step))
    return [tuple(p0 + (p1 - p0) * i / n) for i in range(n + 1)]


if __name__ == "__main__":
    from environment import Environment

    # Base station far away so that its exemption zone plays no role in the examples.
    demo_env = Environment(base_station_pos=(50.0, -100.0), budget=1000, velocity=5.0,
                           radius=5.0, min_safety_dist=2.0)
    step = demo_env.v * 0.5
    examples = {
        "Head-on encounter on one road": {0: _straight((0, 50), (100, 50), step),
                                          1: _straight((100, 50), (0, 50), step)},
        "Crossing paths": {0: _straight((0, 50), (100, 50), step),
                           1: _straight((50, 0), (50, 100), step)},
    }
    for title, trajectories in examples.items():
        print(f"\n=== {title} ===")
        _, rep = game_theory_resolve(demo_env, trajectories, return_report=True)
        g = rep["games"][0]
        print(f"Game at t = {g['time']:.1f} s, decision at frame {g['decision_frame']}, alpha = {g['alpha']:.2f}")
        for k, acts in g["strategies"].items():
            for name, dT, dD, W, note in acts:
                print(f"  MS {k} {name:<6}: dT={dT:5.1f}s dD={dD:5.2f}m W={W:5.1f}s  ({note})")
        print(g["payoff_text"])
        print(f"Best-response moves: {g['moves']}  ->  equilibrium {g['equilibrium']}")
