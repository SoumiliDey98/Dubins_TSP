# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
#
# WHAT THIS FILE IS FOR
# The same job as conflict_game.py (keep robots at least 3 m apart), done the
# simplest possible way, like children taking turns on a slide: the robots go
# one after another, and each one just STOPS for a moment whenever it would get
# too close to a robot that has already had its turn. No game, no detours.
# A robot's drive is a "trajectory": its position every 0.5 s ("frames").
# ==============================================================================
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
# "import" borrows a toolbox. numpy (nicknamed np) works with whole lists
# ("arrays") of numbers at once.
import numpy as np

# Two fixed numbers (written in capitals because they never change):
# stop 5 frames (2.5 s) before the place where the robots would meet ...
STOP_BEFORE = 5      # stop this many frames before the meeting (2.5 s)
# ... and after waiting, the way must stay clear for 10 more frames (5 s).
CLEAR_AFTER = 10     # the way must stay clear this many frames after the sink gets there (5 s)


# "def" starts a recipe (a "function"). This one says, for every half-second
# step, whether robots P and Q come too close. P and Q are trajectories.
def too_close(P, Q, p):
    """For every step between two frames: do sinks P and Q come closer than d_safe?
    Looks 5 times per step, only while both are on their mission. Sinks that are
    both near the base are allowed to be close, since everyone starts there."""
    # n: the number of steps while both are driving (one less than the frames).
    n = min(len(P), len(Q)) - 1
    # hit: one yes/no per step; all "no" (False) to begin with.
    hit = np.zeros(n, dtype=bool)
    # The base's position.
    base = np.asarray(p["base"])
    # Look 5 times within each step: at its start (0), a quarter, half and three
    # quarters of the way, and its end (1).
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        # Where P is at that moment of every step (moving in a straight line between frames).
        a = P[:n] + t * (P[1:n + 1] - P[:n])
        # Where Q is at the same moment.
        b = Q[:n] + t * (Q[1:n + 1] - Q[:n])
        # Too close: less than d_safe (3 m) apart.
        close = np.hypot(*(a - b).T) < p["d_safe"]
        # Both within 5 m of the base: allowed, since everyone starts there.
        at_base = (np.hypot(*(a - base).T) <= p["base_radius"]) & (np.hypot(*(b - base).T) <= p["base_radius"])
        # Mark a step as a hit if, at this look, they are too close and not
        # both at the base. "|=" means "or it into what is already there".
        hit |= close & ~at_base
    # Hand back the yes/no list.
    return hit


# Finds the first step, from step "start" on, where robot P comes too close to
# any of the "others".
def first_meeting(P, others, start, p):
    """First step from `start` on where P is too close to one of the others (None if never)."""
    # None means "no meeting found yet".
    first = None
    # Check against each other robot Q.
    for Q in others:
        # The steps where they are too close, from "start" on (np.flatnonzero
        # lists the positions of the Trues).
        hits = np.flatnonzero(too_close(P, Q, p)[start:])
        # If there is one, and it is earlier than the earliest found so far, keep it.
        if hits.size and (first is None or start + hits[0] < first):
            # Remember the step's number, counted from the start of the trajectory.
            first = start + int(hits[0])
    # Hand back the earliest meeting step, or None.
    return first


# The main recipe: every robot takes its turn. Same inputs and outputs as
# conflict_game.resolve: trajectories, times T, lengths L, and the obstacles
# (geo, not needed here).
def resolve(trajs, T, L, geo, p):
    """Give every sink its turn. Returns updated copies of trajs and T, the unchanged
    lengths L, and the list of stops made. (geo is not needed: stopping does not move the path.)"""
    # Work on copies, so the dictionaries handed in are not changed.
    trajs, T = dict(trajs), dict(T)
    # dt: 0.5 s between frames.
    dt = p["dt"]
    done, stops = [], []                              # sinks that have had their turn; stops made
    # The robots in order of their mission time, longest first (sorting by
    # minus T puts the biggest T first; ties are broken by the robot's number).
    for k in sorted(trajs, key=lambda k: (-T[k], k)):  # longest tour first
        # P: this robot's trajectory; start looking from step 1.
        P, start = trajs[k], 1
        # Repeat until no meeting is left:
        while True:
            # The first meeting with a robot that already had its turn.
            f = first_meeting(P, done, start, p)
            # None: nothing more to fix for this robot.
            if f is None:
                # Leave the loop: this robot is done.
                break
            # Stop 5 frames before the meeting (but not before frame 1).
            s = max(1, f - STOP_BEFORE)               # where to stop
            # The longest allowed stop, in frames: at most max_wait (20 s), and
            # at most the time left before the budget B runs out.
            longest = min(int(round(p["max_wait"] / dt)), int((p["B"] - T[k]) / dt + 1e-9))
            # Try stopping for 1 frame, then 2, then 3, and so on.
            for w in range(1, longest + 1):           # try longer and longer stops
                # The trajectory with that stop: frame s repeated w more times.
                cand = np.vstack([P[:s + 1], np.repeat(P[s:s + 1], w, axis=0), P[s + 1:]])
                # Is the way clear of every finished robot, from the stop until
                # 10 frames after the old meeting place?
                if not any(too_close(cand, Q, p)[s:f + w + CLEAR_AFTER].any() for Q in done):
                    # Yes: keep this stop, and add its time to the robot's mission time.
                    P, T[k] = cand, T[k] + w * dt
                    # Write the stop down: which robot, when, and for how long.
                    stops.append({"sink": k, "time": s * dt, "wait": w * dt})
                    # Look for the next meeting after this one.
                    start = f + w
                    # Leave the "try longer stops" loop.
                    break
            # "else" after a "for" loop runs only if the loop ended WITHOUT a
            # "break": no stop was long enough. Leave that conflict in place and
            # look further on.
            else:
                start = f + 1                         # no stop helps: leave this one, look further on
        # Save this robot's final trajectory ...
        trajs[k] = P
        # ... and it has now had its turn.
        done.append(P)
    # Hand back the trajectories, the times, the lengths (unchanged, because
    # stopping adds no distance), and the list of stops.
    return trajs, T, dict(L), stops
