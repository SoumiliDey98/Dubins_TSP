"""The simplest collision resolution: no game theory, just "wait your turn".

The whole rule:
  1. Sinks take turns, longest tour first. That sink has the least spare time, so it never waits.
  2. Each sink looks along its path for the first moment it comes closer than d_safe
     to a sink that has already had its turn.
  3. It stops a little earlier, for the shortest time that clears that meeting (never
     longer than max_wait or the time left in its budget B). Then it looks further on.
  4. If no stop clears a meeting, it leaves that conflict in place and looks further on.
     The checks in planner.py report any conflict that remains.

Stopping never changes the path, only the timing, so no extra distance is added and
obstacle clearance and turning radius stay as planned.

Same inputs and outputs as conflict_game.resolve, so planner.py can use either one
(python planner.py --seed 1 --resolver simple).
"""
import numpy as np

STOP_BEFORE = 5      # stop this many frames before the meeting (2.5 s)
CLEAR_AFTER = 10     # the way must stay clear this many frames after the sink gets there (5 s)


def too_close(P, Q, p):
    """For every step between two frames: do sinks P and Q come closer than d_safe?
    Looks 5 times per step, only while both are on their mission. Sinks that are
    both near the base are allowed to be close, since everyone starts there."""
    n = min(len(P), len(Q)) - 1
    hit = np.zeros(n, dtype=bool)
    base = np.asarray(p["base"])
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        a = P[:n] + t * (P[1:n + 1] - P[:n])
        b = Q[:n] + t * (Q[1:n + 1] - Q[:n])
        close = np.hypot(*(a - b).T) < p["d_safe"]
        at_base = (np.hypot(*(a - base).T) <= p["base_radius"]) & (np.hypot(*(b - base).T) <= p["base_radius"])
        hit |= close & ~at_base
    return hit


def first_meeting(P, others, start, p):
    """First step from `start` on where P is too close to one of the others (None if never)."""
    first = None
    for Q in others:
        hits = np.flatnonzero(too_close(P, Q, p)[start:])
        if hits.size and (first is None or start + hits[0] < first):
            first = start + int(hits[0])
    return first


def resolve(trajs, T, L, geo, p):
    """Give every sink its turn. Returns updated copies of trajs and T, the unchanged
    lengths L, and the list of stops made. (geo is not needed: stopping does not move the path.)"""
    trajs, T = dict(trajs), dict(T)
    dt = p["dt"]
    done, stops = [], []                              # sinks that have had their turn; stops made
    for k in sorted(trajs, key=lambda k: (-T[k], k)):  # longest tour first
        P, start = trajs[k], 1
        while True:
            f = first_meeting(P, done, start, p)
            if f is None:
                break
            s = max(1, f - STOP_BEFORE)               # where to stop
            longest = min(int(round(p["max_wait"] / dt)), int((p["B"] - T[k]) / dt + 1e-9))
            for w in range(1, longest + 1):           # try longer and longer stops
                cand = np.vstack([P[:s + 1], np.repeat(P[s:s + 1], w, axis=0), P[s + 1:]])
                if not any(too_close(cand, Q, p)[s:f + w + CLEAR_AFTER].any() for Q in done):
                    P, T[k] = cand, T[k] + w * dt
                    stops.append({"sink": k, "time": s * dt, "wait": w * dt})
                    start = f + w
                    break
            else:
                start = f + 1                         # no stop helps: leave this one, look further on
        trajs[k] = P
        done.append(P)
    return trajs, T, dict(L), stops
