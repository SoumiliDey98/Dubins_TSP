# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it.
# 
# WHAT THIS FILE IS FOR
# It draws the films (GIF animations) that pipeline_gifs.py makes: robots
# moving and meeting, groups of sensors being split, one robot's route being
# refined, a conflict game, the six Dubins shapes, and distances spreading
# round obstacles. Each film is a recipe called ..._animation. They all work the
# same way: set up a page, then write an "update(frame)" recipe that draws
# picture number "frame"; save_gif (from plot.py) calls it for every picture.
# Drawing words (figure, axes, plot, scatter, zorder, alpha, lw ...) are
# explained at the top of plot.py. Red (CRITICAL) always means a problem and
# green (GOOD) means fixed, and both always come with words, never colour alone.
# ==============================================================================
"""Animations of the whole algorithm (pipeline_gifs.py). Each one shows something getting better:

    conflict_animation   the sinks moving as planned and after the conflict game, side by side (Steps 8-10)
    splitting_animation  groups split until every tour fits the budget B, one step per GA run (Steps 1-7, Section 0.9)
    partition_animation  the same splitting, one step per split: 1 sink, then 2, then 3, ... with every group's tour
    game_animation       one conflict game replayed switch by switch, with the potential Phi falling (Section 0.10.1)
    dubins_rotation_animation  the six Dubins turn types as the end heading of one leg turns (Section 0.13)
    geodesic_wave_animation    the geodesic distance r spreading from the base around the obstacles (eq. 9)
    refine_animation     one sink's tour: the GA's straight lines, the waypoints around the obstacles,
                         and the Dubins path it drives (Sections 0.8, 0.13 and eq. 17)
    pipeline_animation   all of it in one GIF, chapter by chapter, with the objectives updating

They read what planner.run returns, plus the GA log of plan_routes; nothing is planned again
here except trajectory.waypoints, which is deterministic.
"""
# "import" borrows a toolbox. matplotlib draws charts and pictures.
import matplotlib

# "Agg": draw into files only, without opening a window.
matplotlib.use("Agg")
# pyplot (nicknamed plt): matplotlib's drawing commands.
import matplotlib.pyplot as plt
# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np
# Line2D: a line, used to make sample lines for legends.
from matplotlib.lines import Line2D
# ConvexHull: the smallest shape with no dents that holds a set of points,
# like a rubber band stretched round them.
from scipy.spatial import ConvexHull

# This project's conflict game (nicknamed cg).
import conflict_game as cg
# This project's genetic algorithm.
import ga_tour
# This project's drivable paths.
import trajectory
# Colours and drawing helpers shared with plot.py.
from plot import (BEST, DASHES, GRID, INK, MUTED, POP, QUIET, SURFACE, _map, _quiet_axes, _style, ga_summary,
                  save_gif, stop_indices)

CRITICAL, GOOD = "#d03b3b", "#0ca30c"        # status colours; always shown with a label, never alone
PENDING = "#c9c8c1"                          # sensors of groups not planned yet


# ---------------------------------------------------------------------------
# Shared pieces
# ---------------------------------------------------------------------------

# "def" starts a recipe (a "function"). This one writes a page's title and
# subtitle at the top. The "_" at the start means "only used inside this file".
def _header(fig, title, subtitle):
    # The title, bold, at the top left (positions run from 0 to 1 across and up).
    fig.text(0.015, 0.975, title, fontsize=12, fontweight="bold", color=INK, va="top")
    # The subtitle under it, in quieter grey.
    fig.text(0.015, 0.932, subtitle, fontsize=9, color=QUIET, va="top")


# The area of map to show: big enough for all sensors, the base, and any
# extra lines handed in.
def _limits(out, extra=()):
    """Map limits that hold the sensors, the base and every given polyline."""
    # All those points together.
    pts = np.vstack([out["scenario"]["xy"], out["scenario"]["base"][None, :]] + [np.asarray(e) for e in extra])
    # Their smallest and largest x and y, with 4 m to spare.
    lo, hi = pts.min(0) - 4, pts.max(0) + 4
    # Hand back the two corners.
    return lo, hi


# Clears a drawing area and draws the map on it again.
def _draw_map(ax, out, lims):
    """Clear the axes and draw obstacles, their r_safe boundary and the base."""
    # Clear it ("clear axes").
    ax.cla()
    # Obstacles, their 2 m safety band, and the base (from plot.py).
    _map(ax, out)
    # Page-coloured background.
    ax.set_facecolor(SURFACE)
    # No tick marks, small grey numbers.
    ax.tick_params(length=0, labelsize=7, colors=MUTED)
    # The four border lines ("spines") ...
    for side in ax.spines:
        # ... hidden.
        ax.spines[side].set_visible(False)
    # Show the chosen area across ...
    ax.set_xlim(lims[0][0], lims[1][0])
    # ... and up and down.
    ax.set_ylim(lims[0][1], lims[1][1])


# Saves a film (a short name for plot.save_gif).
def _save(fig, update, n_frames, path, fps):
    # Draw every picture and write the GIF file.
    save_gif(fig, update, n_frames, path, fps)


# Works out, for every half-second step, which pairs of robots are in
# conflict, how close the closest two are, and the total conflict time so far.
def conflict_timeline(trajs, p):
    """For every interval [frame n, n+1]: the pairs closer than d_safe (eq. 20, outside the docking
    zone), the closest distance between any two sinks outside the docking zone, and the time in
    conflict summed over pairs up to and including that interval. The total equals the exposure of
    conflict_game.summary."""
    # The robots' numbers, in order.
    keys = sorted(trajs)
    # The number of frames of the longest trajectory.
    n = max(len(f) for f in trajs.values())
    # For each step, a list of the pairs in conflict (all empty at first).
    active = [[] for _ in range(n)]
    # For each step, the closest distance between any two robots (infinity at first).
    closest = np.full(n, np.inf)
    # Every pair of robots (a, b) once:
    for i, a in enumerate(keys):
        # b is every robot after a.
        for b in keys[i + 1:]:
            # Their closest distance in each step, and whether both are near the base.
            d, dock = cg.closest(trajs[a], trajs[b], p)
            # For every step where they are closer than 3 m and not both docked ...
            for t in np.flatnonzero((d < p["d_safe"]) & ~dock):
                # ... note the pair in that step's list.
                active[t].append((a, b))
            # The steps where they are not both docked.
            ok = np.flatnonzero(~dock)
            # Keep the smaller distance for those steps.
            closest[ok] = np.minimum(closest[ok], d[ok])
    # The conflict time so far: pairs in conflict per step, added up as we go
    # (np.cumsum, a running total), times 0.5 s.
    cum = p["dt"] * np.cumsum([len(a) for a in active])
    # Hand back the three lists.
    return {"active": active, "closest": closest, "cum": cum}


# Where a robot is at frame n (its last position if it has finished).
def _position(f, n):
    # Frame n, or the last frame.
    return f[min(n, len(f) - 1)]


# Draws all robots at frame n: their whole paths faintly, their dots, and
# a red link between any two in conflict.
def _draw_moving(ax, trajs, n, active, faint=0.25):
    """Every sink's whole path (faint), its position at frame n, and a red link between any two
    sinks that are closer than d_safe in the interval that starts at frame n."""
    # Every robot's whole path ...
    for k, f in trajs.items():
        # ... as a thin faint line in its colour.
        ax.plot(f[:, 0], f[:, 1], color=_style(k)[0], lw=0.7, alpha=faint, zorder=2)
    # Every pair in conflict at this step:
    for a, b in (active[n] if n < len(active) else []):
        # Where the two are.
        pa, pb = _position(trajs[a], n), _position(trajs[b], n)
        # A thick red line between them ...
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=CRITICAL, lw=2.4, zorder=6, solid_capstyle="round")
        # ... and a red ring round each.
        ax.scatter([pa[0], pb[0]], [pa[1], pb[1]], s=150, facecolor="none", edgecolor=CRITICAL, lw=1.8, zorder=7)
    # Every robot:
    for k, f in trajs.items():
        # Where it is now.
        x, y = _position(f, n)
        # A dot in its colour with a dark rim.
        ax.scatter([x], [y], s=34, color=_style(k)[0], edgecolor=INK, lw=0.6, zorder=8)


# Draws the 5 m docking circle round the base.
def _docking_zone(ax, out):
    # The base and the circle's radius.
    base, rad = out["scenario"]["base"], out["params"]["base_radius"]
    # 100 angles round a full turn.
    t = np.linspace(0, 2 * np.pi, 100)
    # The points of the circle, joined by a dotted line.
    ax.plot(base[0] + rad * np.cos(t), base[1] + rad * np.sin(t), color=MUTED, lw=0.7, ls=(0, (1, 1.5)), zorder=3)


# Rebuilds, for every GA run of the planner, which groups were already kept
# as robots and which were still waiting in the queue.
def split_states(log):
    """For every GA run j of plan_routes, in order: the runs already kept as sinks and the groups
    still waiting when run j starts. Reconstructed from the queue rule of plan_routes (first in,
    first out; a split group's two halves go to the back)."""
    # No states yet, nothing kept, and the queue holds the first group (all sensors).
    states, kept, waiting = [], [], [np.asarray(log[0]["ids"])]
    # For every GA run j, in order:
    for j, run in enumerate(log):
        # The planner took the first group out of the queue ...
        current = waiting.pop(0)
        # ... which must be this run's group. "assert" stops with an error if not.
        assert sorted(current.tolist()) == sorted(np.asarray(run["ids"]).tolist()), "the GA log does not follow the queue"
        # Note what was kept and what was waiting when this run started.
        states.append({"kept": list(kept), "waiting": [np.asarray(w) for w in waiting]})
        # If this run's group was kept ...
        if run["kept"]:
            # ... it is now a robot.
            kept.append(j)
        # Otherwise it was split ...
        else:
            # ... and its two halves went to the back of the queue.
            waiting += [np.asarray(h) for h in run.get("halves", [])]
    # Hand back one state per run.
    return states


# Draws the rubber-band outline round some points. **kw: drawing options
# handed on to plot.
def _hull(ax, pts, **kw):
    # It needs at least 3 points.
    if len(pts) >= 3:
        # "try" runs the next lines, and if they fail ...
        try:
            # The outline.
            h = ConvexHull(pts)
        # ... (for example, all points in a straight line) ...
        except Exception:
            # ... give up quietly.
            return
        # The outline's corners, with the first repeated at the end to close it.
        v = pts[np.append(h.vertices, h.vertices[0])]
        # Draw it.
        ax.plot(v[:, 0], v[:, 1], **kw)


# Draws GA run j of the splitting on the map.
def _draw_split(ax, out, log, states, j, decided):
    """Run j of the splitting: kept sinks in their colours, waiting groups grey inside a dotted outline,
    the current group's tour in black, or once decided in green (kept) or red (split)."""
    # Sensor positions.
    xy = out["scenario"]["xy"]
    # What was kept and waiting at run j.
    st = states[j]
    # Each robot kept so far (k: its number; jj: its run):
    for k, jj in enumerate(st["kept"]):
        # Its colour.
        color = _style(k)[0]
        # Its route.
        ax.plot(*np.asarray(log[jj]["path"]).T, color=color, lw=0.9, alpha=0.85, zorder=3)
        # Its sensors.
        ax.scatter(*xy[log[jj]["ids"]].T, s=12, color=color, zorder=5)
    # Each group still waiting:
    for w in st["waiting"]:
        # Its sensors in light grey ...
        ax.scatter(*xy[w].T, s=10, color=PENDING, zorder=4)
        # ... inside a dotted outline.
        _hull(ax, xy[w], color=QUIET, lw=0.7, ls=(0, (1, 1.5)), zorder=4)
    # This run.
    run = log[j]
    # Its route is black while undecided, then green if kept, red if split.
    color = INK if not decided else (GOOD if run["kept"] else CRITICAL)
    # Draw its route.
    ax.plot(*np.asarray(run["path"]).T, color=color, lw=1.5, zorder=6)
    # Its sensors in black.
    ax.scatter(*xy[run["ids"]].T, s=14, color=INK, zorder=7)
    # If it was split ...
    if decided and not run["kept"]:
        # ... outline each half ...
        for h in run.get("halves", []):
            # ... with a red dashed line.
            _hull(ax, xy[h], color=CRITICAL, lw=1.0, ls=DASHES, zorder=7)


# The caption for run j, and its status colour.
def _split_text(run, j, k_next, B):
    # How many sensors.
    n = len(run["ids"])
    # A single sensor that still takes too long:
    if run["kept"] and run["T"] > B:
        # kept, but reported as a problem (red).
        return (f"GA run {j}: one sensor, T = {run['T']:.1f} s > B = {B:g} s  →  kept as sink {k_next}, "
                f"reported as a problem", CRITICAL)
    # Within the budget:
    if run["kept"]:
        # kept as the next robot (green).
        return f"GA run {j}: {n} sensors, T = {run['T']:.1f} s ≤ B = {B:g} s  →  kept as sink {k_next}", GOOD
    # Otherwise: split in two (red).
    return f"GA run {j}: {n} sensors, T = {run['T']:.1f} s > B = {B:g} s  →  split in two", CRITICAL


# A line of text with a coloured square in front of it.
def _status_line(fig, x, y, size=10):
    """A text line with a coloured square in front of it, for status messages (the colour is never alone)."""
    # The square ...
    mark = fig.text(x, y, "■", fontsize=size + 2, color=INK, va="bottom")
    # ... and the (empty for now) text after it.
    text = fig.text(x + 0.018, y, "", fontsize=size, fontweight="bold", color=INK, va="bottom")
    # Hand both back, so they can be changed later.
    return mark, text


# The chart beside the map: every run's time T so far, against the budget B.
def _split_chart(ax, log, j, B, decided=True):
    """Tour time T of every GA run so far (log scale), green if kept, red if split, with the budget B."""
    # Clear the chart.
    ax.cla()
    # Plain, with horizontal grid lines.
    _quiet_axes(ax, "y")
    # A "log scale" up the side: each step up multiplies (10, 100, 1000 ...),
    # so very long and short times both fit.
    ax.set_yscale("log")
    # Every run's time.
    Ts = np.array([r["T"] for r in log])
    # One place per run across.
    ax.set_xlim(-0.8, len(log) - 0.2)
    # Room above and below all the times and B.
    ax.set_ylim(min(Ts.min(), B) / 1.4, Ts.max() * 1.4)
    # A dashed line at the budget B ...
    ax.axhline(B, color=INK, lw=1, ls=DASHES, zorder=2)
    # ... labelled.
    ax.text(len(log) - 0.4, B * 1.06, f"B = {B:g} s", fontsize=8, color=INK, ha="right", va="bottom")
    # The runs to show: up to j (j itself only once decided).
    shown = range(j + 1) if decided else range(j)
    # For each of them:
    for i in shown:
        # A dot at its time: green if kept, red if split.
        ax.scatter([i], [log[i]["T"]], s=36, color=GOOD if log[i]["kept"] else CRITICAL,
                   edgecolor=SURFACE, lw=1.2, zorder=4)
    # The label along the bottom ...
    ax.set_xlabel("GA run", fontsize=8.5, color=MUTED)
    # ... and up the side.
    ax.set_ylabel("Tour time T [s]", fontsize=8.5, color=MUTED)
    # Write the side numbers plainly ("100", not "10^2").
    ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
    # No numbers on the small in-between marks.
    ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    # The legend: green kept, red split.
    ax.legend(handles=[Line2D([], [], ls="", marker="o", ms=6, color=GOOD, label="kept: T ≤ B"),
                       Line2D([], [], ls="", marker="o", ms=6, color=CRITICAL, label="split: T > B")],
              loc="upper right", frameon=False, fontsize=8, labelcolor=INK)


# ---------------------------------------------------------------------------
# 1. Conflicts before and after the game
# ---------------------------------------------------------------------------

# Film: the robots as planned (left) and after the conflict game (right),
# side by side. step: how many frames to move on per picture; dpi: pixels
# per inch; hold: extra pictures at the end.
def conflict_animation(out, path, fps=10, step=1, dpi=72, hold=20):
    """Two maps side by side: the sinks as planned (Steps 1-7) and after the conflict game (Steps 8-10).
    Red links join two sinks closer than d_safe; below, the closest distance between any two sinks and
    the time spent in conflict so far, for both versions."""
    # Short names for the result and the settings.
    res, p = out["result"], out["params"]
    # The planned frames of every robot.
    before = {k: t["frames"] for k, t in enumerate(res["tours"])}
    # The frames after the game.
    after = res["frames"]
    # Conflicts step by step, before and after.
    tb, ta = conflict_timeline(before, p), conflict_timeline(after, p)
    # The number of steps of the longer of the two.
    n = max(len(tb["cum"]), len(ta["cum"]))
    # The area of map to show.
    lims = _limits(out, list(before.values()) + list(after.values()))
    # An 11 by 7.2 inch page.
    fig = plt.figure(figsize=(11, 7.2), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, "Conflicts between sinks, before and after the conflict game",
            f"All {len(before)} sinks leave the base at t = 0. Red: two sinks closer than d_safe = {p['d_safe']:g} m "
            f"(eq. 20), outside the dotted docking zone round the base.")
    # The left map's drawing area (left edge, bottom, width, height, as page fractions) ...
    left = fig.add_axes([0.02, 0.27, 0.46, 0.55])
    # ... the right map's ...
    right = fig.add_axes([0.52, 0.27, 0.46, 0.55])
    # ... and the chart's, along the bottom.
    chart = fig.add_axes([0.07, 0.06, 0.6, 0.15])
    # A caption above each map, filled in by the film.
    label_l = fig.text(0.02, 0.835, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # (the right one)
    label_r = fig.text(0.52, 0.835, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # The clock.
    clock = fig.text(0.72, 0.17, "", fontsize=11, fontweight="bold", color=INK, va="top")
    # A note explaining the chart ...
    fig.text(0.72, 0.125, "Closest distance between any two sinks;", fontsize=8.5, color=QUIET, va="top")
    # ... in two lines.
    fig.text(0.72, 0.095, "the dashed line is d_safe.", fontsize=8.5, color=QUIET, va="top")
    # The time of each step: 0, 0.5, 1.0 ... seconds.
    time = p["dt"] * np.arange(n)

    # Draws picture number "frame".
    def update(frame):
        # The step to show.
        i = min(frame * step, n - 1)
        # For each side: its map, its trajectories, its conflicts, its caption, its name:
        for ax, trajs, tl, lab, name in ((left, before, tb, label_l, "Planned (Steps 1-7)"),
                                         (right, after, ta, label_r, "After the conflict game (Steps 8-10)")):
            # Draw the map afresh ...
            _draw_map(ax, out, lims)
            # ... with the docking circle ...
            _docking_zone(ax, out)
            # ... and the robots at step i, with red links for conflicts.
            _draw_moving(ax, trajs, i, tl["active"])
            # How many pairs are in conflict right now.
            now = len(tl["active"][i]) if i < len(tl["active"]) else 0
            # The conflict time so far.
            spent = tl["cum"][min(i, len(tl["cum"]) - 1)]
            # The caption.
            lab.set_text(f"{name}\ntime in conflict so far {spent:.1f} s,  pairs in conflict now {now}")
        # Clear the chart ...
        chart.cla()
        # ... make it plain ...
        _quiet_axes(chart, "y")
        # ... time across ...
        chart.set_xlim(0, time[-1])
        # ... 0 to 20 m up ...
        chart.set_ylim(0, 20)
        # ... and a dashed line at 3 m.
        chart.axhline(p["d_safe"], color=INK, lw=1, ls=DASHES)
        # For before (orange, dashed) and after (blue, solid):
        for tl, color, ls, name in ((tb, POP, DASHES, "planned"), (ta, BEST, "-", "after the game")):
            # The closest distance up to now (capped at 20 m to fit) ...
            c = np.minimum(tl["closest"][:i + 1], 20)
            # ... drawn as a line.
            chart.plot(time[:len(c)], c, color=color, lw=2, ls=ls, label=name)
        # Labels along the bottom ...
        chart.set_xlabel("t [s]", fontsize=8.5, color=MUTED)
        # ... and up the side.
        chart.set_ylabel("metres", fontsize=8.5, color=MUTED)
        # The chart's legend.
        chart.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK, ncol=2)
        # The clock.
        clock.set_text(f"t = {time[i]:.1f} s")
        # Nothing to hand back: everything is redrawn.
        return []

    # Make and save the film.
    _save(fig, update, (n - 1) // step + 1 + hold, path, fps)
    # Hand back the total conflict time before and after.
    return {"before": tb["cum"][-1], "after": ta["cum"][-1]}


# ---------------------------------------------------------------------------
# 2. Splitting into sinks
# ---------------------------------------------------------------------------

# Film: one step per GA run, each group kept or split.
def splitting_animation(out, log, path, fps=3, dpi=80, hold=8):
    """One step per GA run: the current group's tour is planned (black), then kept (green, T <= B) or
    split in two (red, with the two halves outlined). Kept sinks stay in their colours; groups still
    waiting are grey. Beside the map, the tour time of every run so far against B."""
    # The settings.
    p = out["params"]
    # What was kept and waiting at every run.
    states = split_states(log)
    # The area of map to show.
    lims = _limits(out, [r["path"] for r in log])
    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, "Splitting into sinks until every tour fits the budget (Steps 1-7)",
            f"One sink with all {len(log[0]['ids'])} sensors would need {log[0]['T']:.0f} s. Every group whose tour takes "
            f"longer than B = {p['B']:g} s is split in two (Section 0.9) and planned again.")
    # The map's drawing area ...
    ax = fig.add_axes([0.045, 0.06, 0.48, 0.78])
    # ... and the chart's.
    chart = fig.add_axes([0.6, 0.3, 0.37, 0.5])
    # The status line (square plus text).
    mark, status = _status_line(fig, 0.015, 0.865)
    # A text for the counts.
    counts = fig.text(0.6, 0.18, "", fontsize=10, color=INK, va="top")
    # The script: for each run j, one picture undecided and two decided.
    frames = [(j, d) for j in range(len(log)) for d in (False, True, True)]

    # Draws picture number "frame".
    def update(frame):
        # Which run, and whether it is decided yet.
        j, decided = frames[min(frame, len(frames) - 1)]
        # The map afresh.
        _draw_map(ax, out, lims)
        # The groups and this run's route.
        _draw_split(ax, out, log, states, j, decided)
        # The chart of times.
        _split_chart(chart, log, j, p["B"], decided)
        # Decided: the run's caption and colour ...
        if decided:
            # (green kept, red split).
            text, color = _split_text(log[j], j, len(states[j]["kept"]), p["B"])
        # Undecided:
        else:
            # "planning ..." in black.
            text, color = f"GA run {j}: planning a group of {len(log[j]['ids'])} sensors", INK
        # Show the caption ...
        status.set_text(text)
        # ... and colour the square.
        mark.set_color(color)
        # Robots kept so far (this one too, once decided and kept).
        kept = len(states[j]["kept"]) + (1 if decided and log[j]["kept"] else 0)
        # Groups waiting (plus this one's halves, once split).
        waiting = len(states[j]["waiting"]) + (len(log[j].get("halves", [])) if decided else 0)
        # Show both counts.
        counts.set_text(f"Sinks kept: {kept}\nGroups still waiting: {waiting}")
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(frames) + hold, path, fps)


# ---------------------------------------------------------------------------
# 2b. The partition, one sink more at every step
# ---------------------------------------------------------------------------

# Rebuilds the set of groups after each split: 1 group, then 2, then 3 ...
def partition_states(log):
    """The partition of the sensors after each split, from one group to the final sinks.

    Returns (states, splits, slot). states[k] lists the k + 1 groups of the partition, each as the
    index of the GA run that planned it (every group in the queue gets one). splits[k] is the run whose
    group is split to go from states[k] to states[k + 1]. slot[j] is a colour number that a group keeps
    until it is split: its first half keeps it and its second half takes the next free one."""
    # Look-up from a group of sensors to the GA run that planned it. (A
    # "frozenset" is an unchangeable set, usable as a dictionary key.)
    run_of = {frozenset(np.asarray(r["ids"]).tolist()): j for j, r in enumerate(log)}
    # Start: one group (run 0) with colour number 0; the next free colour is 1.
    groups, slot, free = [0], {0: 0}, 1
    # The first state is that one group; no splits yet.
    states, splits = [[0]], []
    # For every GA run j:
    for j, r in enumerate(log):
        # Kept groups are not split ...
        if r["kept"]:
            # ... so skip them.
            continue
        # The runs that planned its two halves.
        halves = [run_of.get(frozenset(np.asarray(h).tolist())) for h in r.get("halves", [])]
        # If the halves were never planned ...
        if len(halves) != 2 or None in halves:
            break                                       # the planner stopped before planning both halves (K_max)
        # Where the split group is in the list ...
        i = groups.index(j)
        # ... replace it by its two halves.
        groups[i:i + 1] = halves
        # The first half keeps the colour; the second takes the next free one.
        slot[halves[0]], slot[halves[1]] = slot[j], free
        # That colour is now used.
        free += 1
        # Note which group was split ...
        splits.append(j)
        # ... and the new set of groups.
        states.append(list(groups))
    # Hand back the states, the splits and the colour numbers.
    return states, splits, slot


# Film: the groups growing one robot at a time, each with its route and a bar of its time.
def partition_animation(out, log, path, fps=2, dpi=80, hold=6):
    """The partition growing one sink at a time: 1 sink, then 2, then 3, ... Every group is drawn with
    the Dubins tour its GA run gives it, in a colour it keeps until it is split; beside the map, each
    group's tour time against B. The group that is split next is outlined in red."""
    # Short names for the map and the settings.
    scen, p = out["scenario"], out["params"]
    # Sensor positions, and the budget B.
    xy, B = scen["xy"], p["B"]
    # The groups after each split.
    states, splits, slot = partition_states(log)
    # The area of map to show.
    lims = _limits(out, [r["path"] for r in log])
    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, "Partitioning: one sink, then two, then three, until every tour fits the budget",
            f"Each group's tour is the Dubins tour its own GA run gives it. A group whose tour takes longer than "
            f"B = {B:g} s is split in two (Section 0.9).")
    # The map's drawing area ...
    ax = fig.add_axes([0.045, 0.05, 0.48, 0.79])
    # ... and the bar chart's.
    chart = fig.add_axes([0.6, 0.42, 0.37, 0.38])
    # The status line.
    mark, status = _status_line(fig, 0.045, 0.865)
    # Four big numbers: robots, longest time, total length, tours over B.
    tiles = _tiles(fig, 0.56, 0.25, ("Sinks K", "Longest T_max", "Total L_total", "Tours over B"))
    # A note about the colours.
    fig.text(0.56, 0.09, "Bars use the colours of the map. There are only eight colours, so from the\n"
             "ninth group on they repeat, with dashed tours and hatched bars.", fontsize=8, color=QUIET, va="bottom")
    # The script, built below.
    frames = []
    # For each state k (k + 1 groups):
    for k in range(len(states)):
        # two pictures of it ...
        frames += [(k, False)] * 2
        # ... and, if a split follows ...
        if k < len(splits):
            # ... two more with the group to split outlined.
            frames += [(k, True)] * 2
    # Hold the final state.
    frames += [(len(states) - 1, False)] * hold
    # The longest time of any run (the chart's top).
    Tmax_all = max(r["T"] for r in log)

    # Draws picture number "frame".
    def update(frame):
        # Which state, and whether to outline the next split.
        k, highlight = frames[min(frame, len(frames) - 1)]
        # Its groups.
        groups = states[k]
        # The map afresh.
        _draw_map(ax, out, lims)
        # For each group:
        for j in groups:
            # Its colour and line style.
            color, ls = _style(slot[j])
            # Its route.
            ax.plot(*np.asarray(log[j]["path"]).T, color=color, ls=ls, lw=1.1, zorder=3)
            # Its sensors.
            ax.scatter(*xy[log[j]["ids"]].T, s=13, color=color, zorder=5)
        # If the next split is shown ...
        if highlight:
            # ... the group to split ...
            j = splits[k]
            # ... is outlined in red.
            _hull(ax, xy[log[j]["ids"]], color=CRITICAL, lw=1.8, ls=DASHES, zorder=6)
        # tour time of every group against B
        chart.cla()
        # The bar chart, cleared above: plain, with horizontal grid lines.
        _quiet_axes(chart, "y")
        # A log scale up the side.
        chart.set_yscale("log")
        # Room for every time and B.
        chart.set_ylim(min(min(r["T"] for r in log), B) / 1.5, Tmax_all * 1.6)
        # Room for the most groups there will ever be.
        chart.set_xlim(-0.6, max(len(s) for s in states) - 0.4)
        # The bottom of the chart (bars start here).
        y0 = chart.get_ylim()[0]
        # For each group, at place i:
        for i, j in enumerate(groups):
            # Its colour.
            color = _style(slot[j])[0]
            # Stripes from the ninth colour on, when colours repeat.
            hatch = "////" if slot[j] >= 8 else None
            # Its bar, from the bottom up to its time.
            chart.bar([i], [log[j]["T"] - y0], bottom=y0, width=0.62, color=color, hatch=hatch, edgecolor=SURFACE,
                      lw=0, zorder=3)
            # If it is the group to split next ...
            if highlight and j == splits[k]:
                # ... write "split next" above it.
                chart.text(i, log[j]["T"] * 1.12, "split\nnext", fontsize=7.5, color=INK, ha="center", va="bottom")
        # A dashed line at B ...
        chart.axhline(B, color=INK, lw=1, ls=DASHES, zorder=4)
        # ... labelled.
        chart.text(max(len(s) for s in states) - 0.5, B * 1.06, f"B = {B:g} s", fontsize=8, color=INK, ha="right",
                   va="bottom")
        # Number the groups 1, 2, 3 ... along the bottom.
        chart.set_xticks(range(len(groups)), [str(i + 1) for i in range(len(groups))], fontsize=7)
        # Labels along the bottom ...
        chart.set_xlabel("Group", fontsize=8.5, color=MUTED)
        # ... and up the side.
        chart.set_ylabel("Tour time T [s], log scale", fontsize=8.5, color=MUTED)
        # Plain numbers on the side.
        chart.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        # None on the small in-between marks.
        chart.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        # The groups' times.
        Ts = [log[j]["T"] for j in groups]
        # How many are over the budget.
        over = sum(t > B for t in Ts)
        # The status line, while a split is shown:
        if highlight:
            # the group to split ...
            j = splits[k]
            # ... and why (red).
            status.set_text(f"{len(groups)} sink{'s' if len(groups) > 1 else ''}: the group of {len(log[j]['ids'])} "
                            f"sensors needs T = {log[j]['T']:.0f} s > B = {B:g} s, so it is split in two")
            # Red square.
            mark.set_color(CRITICAL)
        # Some tours still too long:
        elif over:
            # say how many (red).
            status.set_text(f"{len(groups)} sink{'s' if len(groups) > 1 else ''}: {over} tour{'s' if over > 1 else ''} "
                            f"still longer than B = {B:g} s")
            # Red square.
            mark.set_color(CRITICAL)
        # All tours fit:
        else:
            # say so (green).
            status.set_text(f"{len(groups)} sinks: every tour fits within B = {B:g} s (longest {max(Ts):.1f} s)")
            # Green square.
            mark.set_color(GOOD)
        # The four big numbers: robots, longest time, total length, tours over B ...
        for tile, v in zip(tiles, (str(len(groups)), f"{max(Ts):.1f} s", f"{sum(log[j]['L'] for j in groups):.0f} m",
                                   str(over))):
            # ... each written into its tile.
            tile.set_text(v)
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(frames), path, fps)
    # Hand back the states.
    return states


# ---------------------------------------------------------------------------
# 7. From the GA's straight lines to the path the sink drives
# ---------------------------------------------------------------------------

# Where a dense path passes each stop (the same as plot.stop_indices).
def _stops_in(path, stops, start=0):
    """Indices in the polyline `path` where it passes the stops, in order (base, sensors, base)."""
    # Ask plot.py's recipe.
    return stop_indices(path, stops, start)


# Cuts a path at the stops and puts exactly m evenly spaced points on each
# leg, so two versions of a tour can be blended point by point.
def _resample_legs(path, idx, m):
    """The polyline split at the given stop indices, each leg resampled to m points by arc length."""
    # The legs, built below.
    out = []
    # For each leg, from stop position a to stop position b:
    for a, b in zip(idx[:-1], idx[1:]):
        # Its points.
        leg = path[a:b + 1]
        # A leg of just one point ...
        if len(leg) < 2:
            # ... becomes that point twice.
            leg = np.vstack([path[a], path[a]])
        # The distance along the leg to each point.
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(leg, axis=0).T))])
        # m distances spread evenly from the start to the end of the leg.
        u = np.linspace(0, s[-1], m) if s[-1] > 0 else np.zeros(m)
        # The points at those distances (read between the leg's points).
        out.append(np.column_stack([np.interp(u, s, leg[:, 0]), np.interp(u, s, leg[:, 1])]))
    # All legs, one after another.
    return np.vstack(out)


# The three versions of robot k's tour, ready to blend: straight lines,
# waypoints round obstacles, and the Dubins path.
def refine_stages(out, k):
    """The three versions of sink k's tour, each resampled leg by leg so that they can be morphed:
    straight lines (what the GA scores), waypoints around obstacles, and the Dubins path."""
    # Short names for the map, the distances and the settings.
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    # Robot k's tour.
    tour = out["result"]["tours"][k]
    # Its visiting order.
    order = tour["order"]
    # Sensor positions and the base.
    xy, base = scen["xy"], scen["base"]
    # Its stops: base, sensors, base.
    stops = np.vstack([base, xy[order], base])
    # Its waypoints round obstacles.
    pts, _ = trajectory.waypoints(order, geo, p)
    # Its dense Dubins path.
    dense = np.asarray(tour["path"])
    # The length of each version.
    lengths = {"straight": ga_tour.euclid_length(order, xy, base),
               "waypoints": float(np.sum(np.hypot(*np.diff(pts, axis=0).T))), "dubins": tour["L"]}
    # 40 points per leg in every version.
    m = 40
    # Each version cut at the stops and resampled the same way.
    shapes = {"straight": _resample_legs(stops, list(range(len(stops))), m),
              "waypoints": _resample_legs(pts, _stops_in(pts, stops), m),
              "dubins": _resample_legs(dense, _stops_in(dense, stops), m)}
    # Hand back the order, stops, shapes, lengths, time and total waiting time.
    return {"order": order, "stops": stops, "shapes": shapes, "lengths": lengths, "T": tour["T"],
            "dwell": float(np.sum(scen["tau"][order]))}


# Chooses which robot to show: the one whose drivable path is longest
# compared with its straight lines.
def pick_refine_sink(out):
    """The sink whose Dubins path is longest compared with its straight-line tour."""
    # Sensor positions and the base.
    xy, base = out["scenario"]["xy"], out["scenario"]["base"]
    # Dubins length divided by straight-line length, for every robot.
    ratio = [t["L"] / max(ga_tour.euclid_length(t["order"], xy, base), 1e-9) for t in out["result"]["tours"]]
    # The robot with the biggest ratio.
    return int(np.argmax(ratio))


# Film: robot k's tour blending from straight lines to waypoints to the Dubins path.
def refine_animation(out, k, path, fps=12, dpi=80):
    """Sink k's tour in three stages, morphing from one to the next: the GA's straight lines, the
    waypoints around the obstacles, and the Dubins path. Bars show each stage's length."""
    # The settings.
    p = out["params"]
    # The three versions.
    st = refine_stages(out, k)
    # Their shapes and lengths.
    shapes, L = st["shapes"], st["lengths"]
    # All their points.
    allpts = np.vstack(list(shapes.values()))
    # Their range, with 8 m to spare.
    lo, hi = allpts.min(0) - 8, allpts.max(0) + 8
    # The larger of width and height ...
    span = max(hi - lo)
    # ... and the middle ...
    mid = (lo + hi) / 2
    # ... give a square view.
    lims = (mid - span / 2, mid + span / 2)
    # Each version's key and caption.
    names = [("straight", "Straight lines between sensors (what the GA scores)"),
             ("waypoints", "Waypoints around the obstacles (geodesic paths)"),
             ("dubins", "Dubins path with turning radius R (what the sink drives)")]
    # Each version's colour: grey, orange, blue.
    colors = {"straight": QUIET, "waypoints": POP, "dubins": BEST}
    plan = []                                          # (shape a, shape b, fraction, stage reached)
    # The script: for each version i ...
    for i, (key, _) in enumerate(names):
        # ... 14 pictures of it ...
        plan += [(key, key, 0.0, i)] * 14
        # ... then, unless it is the last ...
        if i + 1 < len(names):
            # ... 14 pictures blending it into the next (t from 0 to 1).
            plan += [(key, names[i + 1][0], t, i) for t in np.linspace(0, 1, 16)[1:-1]]
    # End on the Dubins path.
    plan += [("dubins", "dubins", 0.0, 2)] * 24
    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, f"From the GA's straight lines to the path sink {k} drives",
            f"Sink {k} visits {len(st['order'])} sensors in the order the GA chose. The order stays the same; "
            f"only the way between the sensors changes.")
    # The map's drawing area ...
    ax = fig.add_axes([0.045, 0.05, 0.5, 0.8])
    # ... and the bars'.
    bars = fig.add_axes([0.6, 0.45, 0.37, 0.34])
    # The caption, filled in by the film.
    stage_label = fig.text(0.015, 0.87, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # A note for the end.
    note = fig.text(0.6, 0.36, "", fontsize=9.5, color=INK, va="top", linespacing=1.6)

    # Draws picture number "frame".
    def update(frame):
        # From version a to version b, fraction t of the way; versions reached so far.
        a, b, t, reached = plan[min(frame, len(plan) - 1)]
        # Blend: each point moves a fraction t from a's place to b's.
        shape = (1 - t) * shapes[a] + t * shapes[b]
        # The map afresh.
        _draw_map(ax, out, lims)
        # All sensors in light grey.
        ax.scatter(*out["scenario"]["xy"].T, s=8, color=PENDING, zorder=3)
        # The blended tour, in the colour of the nearer version.
        ax.plot(shape[:, 0], shape[:, 1], color=colors[b if t > 0.5 else a], lw=1.8, zorder=5)
        # This robot's sensors in black.
        ax.scatter(*st["stops"][1:-1].T, s=22, color=INK, zorder=6)
        # The caption ("…" while blending).
        stage_label.set_text(names[reached][1] if t == 0 else "…")
        # Clear the bars ...
        bars.cla()
        # ... plain, with vertical grid lines.
        _quiet_axes(bars, "x")
        # The scale's end: the longest version plus 25 %.
        top = max(L.values()) * 1.25
        # Lengths across ...
        bars.set_xlim(0, top)
        # ... room for three bars.
        bars.set_ylim(-0.6, 2.6)
        # For each version:
        for i, (key, title) in enumerate(names):
            # its row (the first at the top).
            y = 2 - i
            # If reached already ...
            if i <= reached:
                # ... a sideways bar as long as its length ...
                bars.barh([y], [L[key]], height=0.5, color=colors[key], zorder=3)
                # ... with the length written after it.
                bars.text(L[key] + 0.01 * top, y, f"{L[key]:.0f} m", va="center", fontsize=9, color=INK)
            # The version's name above its row (the caption up to the bracket).
            bars.text(0, y + 0.36, title.split(" (")[0], fontsize=8.5, color=INK, va="bottom")
        # No side labels.
        bars.set_yticks([])
        # The label along the bottom.
        bars.set_xlabel("Length of the tour [m]", fontsize=8.5, color=MUTED)
        # Once the Dubins path is reached ...
        if reached == 2:
            # ... the note shows the mission time and how much longer than the
            # straight lines it is.
            note.set_text(f"Mission time T = L / v + Σ τ = {L['dubins']:.0f} / {p['v']:g} + {st['dwell']:.1f}"
                          f" = {st['T']:.1f} s\n(budget B = {p['B']:g} s, eqs. 17-18)\n"
                          f"The Dubins path is {100 * (L['dubins'] / L['straight'] - 1):.0f} % longer than\n"
                          f"the straight lines the GA scored.")
        # Before that ...
        else:
            # ... no note.
            note.set_text("")
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(plan), path, fps)
    # Hand back the three versions.
    return st


# ---------------------------------------------------------------------------
# 8. The whole pipeline
# ---------------------------------------------------------------------------

# Four score tiles: a small name, with a big number under it.
def _tiles(fig, x, y, names=("Sinks K", "Longest T_max", "Total L_total", "In conflict")):
    """Four score tiles with the given names (by default K, T_max, L_total and the time in conflict)."""
    # The numbers' text objects, filled in later.
    values = []
    # For each tile:
    for i, name in enumerate(names):
        # its place across the page;
        xi = x + 0.105 * i
        # its name;
        fig.text(xi, y, name, fontsize=8, color=QUIET, va="bottom")
        # an empty big number under it.
        values.append(fig.text(xi, y - 0.01, "", fontsize=13, fontweight="bold", color=INK, va="top"))
    # Hand back the numbers.
    return values


# Film: the whole algorithm in five chapters, with the scores updating.
def pipeline_animation(out, log, path, fps=8, dpi=72, step=2, ga_every=5):
    """The algorithm in five chapters: the GA orders all sensors; groups are split until every tour
    fits B; the final Dubins routes; the sinks moving as planned (conflicts in red); and after the
    conflict game. Score tiles show K, T_max, L_total and the time in conflict as they improve."""
    # Short names for the map, the result and the settings.
    scen, res, p = out["scenario"], out["result"], out["params"]
    # Sensor positions and the base.
    xy, base = scen["xy"], scen["base"]
    # What was kept and waiting at every GA run.
    states = split_states(log)
    # The planned frames ...
    before = {k: t["frames"] for k, t in enumerate(res["tours"])}
    # ... and the frames after the game.
    after = res["frames"]
    # Conflicts, step by step, before and after.
    tb, ta = conflict_timeline(before, p), conflict_timeline(after, p)
    # The area of map to show.
    lims = _limits(out, [r["path"] for r in log] + list(after.values()))
    # The first GA run (all sensors) ...
    first = log[0]
    # ... its generations ...
    gens = first["generations"]
    # ... and their summary.
    s0 = ga_summary(gens, p["R"])
    # The runs that became robots.
    kept_runs = [j for j, r in enumerate(log) if r["kept"]]
    # Every robot's planned time ...
    planned_T = [t["T"] for t in res["tours"]]
    # ... and length.
    planned_L = [t["L"] for t in res["tours"]]

    # The script, built below: (kind, detail) per picture.
    chapters = []
    # Chapter 1: every 5th generation of the first GA run, then the last six times.
    for g in list(range(0, len(gens), ga_every)) + [len(gens) - 1] * 6:
        # (one picture each)
        chapters.append(("ga", g))
    # Chapter 2: every GA run, undecided then decided.
    for j in range(len(log)):
        # (two pictures each)
        chapters += [("split", (j, False)), ("split", (j, True))]
    # Chapter 3: the final routes, 16 pictures.
    chapters += [("routes", None)] * 16
    # How many steps the robots move, before and after.
    n_move_b, n_move_a = len(tb["cum"]), len(ta["cum"])
    # Chapter 4: the robots moving as planned (every 2nd step), then a pause.
    chapters += [("before", i) for i in range(0, n_move_b, step)] + [("before", n_move_b - 1)] * 8
    # Chapter 5: the robots moving after the game, then a long pause.
    chapters += [("after", i) for i in range(0, n_move_a, step)] + [("after", n_move_a - 1)] * 24

    # Each chapter's title and subtitle.
    titles = {
        "ga": ("Steps 1-3: the GA orders all sensors for one sink",
               "Fitness = straight-line length + m(π)·π·R (Section 0.8). The blue line is the best "
               "tour of the generation."),
        "split": ("Steps 4-7: too long, so split",
                  f"A tour longer than B = {p['B']:g} s is split in two (Section 0.9) and each half gets its own GA run."),
        "routes": ("The routes: Dubins paths around the obstacles",
                   f"Every one of the {len(before)} tours fits the budget. Each sink drives a Dubins path with "
                   f"turning radius R = {p['R']:g} m."),
        "before": ("Step 8: conflicts in the plan",
                   f"All sinks leave at t = 0. Red: two sinks closer than d_safe = {p['d_safe']:g} m (eq. 20)."),
        "after": ("Steps 9-10: the conflict game removes them",
                  "Sinks wait or side-step where the game says so (GO / WAIT / DETOUR, Section 0.10.1)."),
    }
    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title, filled in by the film ...
    title = fig.text(0.015, 0.975, "", fontsize=12, fontweight="bold", color=INK, va="top")
    # ... and the subtitle.
    subtitle = fig.text(0.015, 0.932, "", fontsize=9, color=QUIET, va="top")
    # The map's drawing area ...
    ax = fig.add_axes([0.045, 0.04, 0.48, 0.82])
    # ... and the chart's.
    chart = fig.add_axes([0.6, 0.4, 0.37, 0.4])
    # The status line.
    mark, status = _status_line(fig, 0.6, 0.85, size=9.5)
    # The four score tiles.
    tiles = _tiles(fig, 0.56, 0.24)
    # A note with the main settings.
    fig.text(0.56, 0.08, f"{len(xy)} sensors, budget B = {p['B']:g} s, d_safe = {p['d_safe']:g} m.",
             fontsize=8, color=QUIET, va="bottom")

    # Writes the four scores into the tiles.
    def set_tiles(K, T, L, conflict):
        # Each tile with its value ...
        for tile, v in zip(tiles, (K, T, L, conflict)):
            # ... written in.
            tile.set_text(v)

    # Draws picture number "frame", following the script.
    def update(frame):
        # This picture's chapter and detail.
        kind, arg = chapters[min(frame, len(chapters) - 1)]
        # The chapter's title ...
        title.set_text(titles[kind][0])
        # ... and subtitle.
        subtitle.set_text(titles[kind][1])
        # The map afresh.
        _draw_map(ax, out, lims)
        # Clear the chart ...
        chart.cla()
        # ... plain, with horizontal grid lines.
        _quiet_axes(chart, "y")
        # The status square starts black.
        mark.set_color(INK)
        # Chapter 1 (the GA):
        if kind == "ga":
            # the best tour of generation arg ...
            order = gens[arg]["orders"][0]
            # ... as points ...
            pts = np.vstack([base, xy[order], base])
            # ... over all sensors ...
            ax.scatter(*xy.T, s=9, color=INK, zorder=4)
            # ... drawn as a thin blue line.
            ax.plot(pts[:, 0], pts[:, 1], color=BEST, lw=0.7, zorder=3)
            # Generations 0 to arg ...
            x = np.arange(arg + 1)
            # ... and the best fitness as a staircase.
            chart.plot(np.append(x, arg + 1), np.append(s0["best"][:arg + 1], s0["best"][arg]), color=BEST, lw=2,
                       drawstyle="steps-post")
            # Generations across ...
            chart.set_xlim(0, len(gens))
            # ... fitness up.
            chart.set_ylim(s0["best"].min() * 0.98, s0["best"].max() * 1.02)
            # Labels along the bottom ...
            chart.set_xlabel("Generation", fontsize=8.5, color=MUTED)
            # ... and up the side.
            chart.set_ylabel("Fitness of the best tour [m]", fontsize=8.5, color=MUTED)
            # The status text.
            status.set_text(f"Generation {arg}: fitness {s0['best'][arg]:.0f} m")
            # One robot so far; the other scores not known yet.
            set_tiles("1", "—", "—", "—")
        # Chapter 2 (splitting):
        elif kind == "split":
            # which run, and whether decided;
            j, decided = arg
            # the groups and the run's route;
            _draw_split(ax, out, log, states, j, decided)
            # the chart of times.
            _split_chart(chart, log, j, p["B"], decided)
            # Decided:
            if decided:
                # its caption and colour ...
                text, color = _split_text(log[j], j, len(states[j]["kept"]), p["B"])
                # ... on the status square.
                mark.set_color(color)
            # Undecided:
            else:
                # "planning ...".
                text = f"GA run {j}: planning a group of {len(log[j]['ids'])} sensors"
            # Show it (the arrow starts a new line).
            status.set_text(text.replace("  →  ", "\n→ "))
            # The runs kept as robots so far.
            done = states[j]["kept"] + ([j] if decided and log[j]["kept"] else [])
            # Their robot numbers.
            ks = [kept_runs.index(r) for r in done]
            # sinks the plan needs so far: kept groups, waiting groups, and the current group (or its two halves)
            # How many robots the plan needs so far. (The "\" continues the line.)
            groups = len(states[j]["kept"]) + len(states[j]["waiting"]) + \
                (len(log[j]["halves"]) if decided and not log[j]["kept"] else 1)
            # The tiles: robots, longest time and total length of the robots kept.
            set_tiles(str(groups), f"{max(planned_T[k] for k in ks):.1f} s" if ks else "—",
                      f"{sum(planned_L[k] for k in ks):.0f} m" if ks else "—", "—")
        # Chapter 3 (the final routes):
        elif kind == "routes":
            # every robot ...
            for k, t in enumerate(res["tours"]):
                # ... its route ...
                ax.plot(*np.asarray(t["path"]).T, color=_style(k)[0], lw=1.0, zorder=3)
                # ... and its sensors, in its colour.
                ax.scatter(*xy[t["order"]].T, s=12, color=_style(k)[0], zorder=5)
            # A bar per robot: its time.
            chart.bar(np.arange(len(planned_T)), planned_T, width=0.6, color=BEST, zorder=3)
            # A dashed line at B ...
            chart.axhline(p["B"], color=INK, lw=1, ls=DASHES)
            # ... labelled.
            chart.text(len(planned_T) - 0.5, p["B"] * 1.02, f"B = {p['B']:g} s", fontsize=8, color=INK, ha="right",
                       va="bottom")
            # From 0 to a little above B.
            chart.set_ylim(0, p["B"] * 1.2)
            # Labels along the bottom ...
            chart.set_xlabel("Sink", fontsize=8.5, color=MUTED)
            # ... and up the side.
            chart.set_ylabel("Tour time T [s]", fontsize=8.5, color=MUTED)
            # Green: all fit.
            mark.set_color(GOOD)
            # Say so.
            status.set_text(f"All {len(planned_T)} tours within B: longest {max(planned_T):.1f} s")
            # The tiles: robots, longest time, total length.
            set_tiles(str(len(before)), f"{max(planned_T):.1f} s", f"{sum(planned_L):.0f} m", "—")
        # Chapters 4 and 5 (robots moving):
        else:
            # The trajectories and conflicts of this chapter (planned, or after the game).
            trajs, tl = (before, tb) if kind == "before" else (after, ta)
            # The docking circle.
            _docking_zone(ax, out)
            # The robots at this step, with red links for conflicts.
            _draw_moving(ax, trajs, arg, tl["active"])
            # The time of every step, before and after.
            time_b, time_a = p["dt"] * np.arange(len(tb["cum"])), p["dt"] * np.arange(len(ta["cum"]))
            # Chapter 4: the planned conflict time so far, orange dashed.
            if kind == "before":
                # (up to now)
                chart.plot(time_b[:arg + 1], tb["cum"][:arg + 1], color=POP, lw=2, ls=DASHES, label="planned")
            # Chapter 5: the whole planned line ...
            else:
                # (for comparison)
                chart.plot(time_b, tb["cum"], color=POP, lw=2, ls=DASHES, label="planned")
                # ... and the after-game line so far, blue.
                chart.plot(time_a[:arg + 1], ta["cum"][:arg + 1], color=BEST, lw=2, label="after the game")
            # Time across ...
            chart.set_xlim(0, max(time_b[-1], time_a[-1]))
            # ... conflict time up.
            chart.set_ylim(0, max(tb["cum"][-1], 1) * 1.15)
            # Labels along the bottom ...
            chart.set_xlabel("t [s]", fontsize=8.5, color=MUTED)
            # ... and up the side.
            chart.set_ylabel("Time in conflict so far [s]", fontsize=8.5, color=MUTED)
            # The chart's legend.
            chart.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK)
            # The conflict time so far.
            spent = tl["cum"][arg]
            # Pairs in conflict right now.
            now = len(tl["active"][arg])
            # Red if any are in conflict; green after the game when none are.
            mark.set_color(CRITICAL if now else (GOOD if kind == "after" else INK))
            # The status text.
            status.set_text(f"t = {p['dt'] * arg:.1f} s: {now} pair{'s' if now != 1 else ''} in conflict now")
            # The tiles, as planned ...
            if kind == "before":
                # (robots, longest time, total length, conflict time)
                set_tiles(str(len(before)), f"{max(planned_T):.1f} s", f"{sum(planned_L):.0f} m", f"{spent:.1f} s")
            # ... or after the game (with the game's waits and detours).
            else:
                # (robots, longest time, total length, conflict time)
                set_tiles(str(len(after)), f"{max(res['T'].values()):.1f} s", f"{sum(res['L'].values()):.0f} m",
                          f"{spent:.1f} s")
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(chapters), path, fps)
    # Hand back how many pictures it has.
    return len(chapters)


# ---------------------------------------------------------------------------
# 9. Replay of one conflict game, switch by switch
# ---------------------------------------------------------------------------

# Plays round 0 of the conflict games again, recording every game's
# strategies, so they can be shown.
def game_replay_data(out):
    """Round 0 of Steps 9-10 again, on the planned tours, with every game's strategies recorded.
    conflict_game.resolve is deterministic, so these are the planner's own games of that round."""
    # Short names for the result and the settings.
    res, p = out["result"], out["params"]
    # The planned tours.
    tours = res["tours"]
    # The record, filled in by the game.
    trace = []
    # Settle the conflicts exactly as the planner did, recording into trace.
    # ("_" throws away the new trajectories, times and lengths.)
    _, _, _, games = cg.resolve({k: t["frames"] for k, t in enumerate(tours)}, {k: t["T"] for k, t in enumerate(tours)},
                                {k: t["L"] for k, t in enumerate(tours)}, out["geo"], p, trace)
    # Hand back the game records and the recording.
    return games, trace


# Chooses which game to show by default.
def pick_game(games):
    """The game with the most best-response switches (then the most players)."""
    # The one with the most switches (then the most players).
    return max(range(len(games)), key=lambda i: (len(games[i]["moves"]), len(games[i]["players"])))


# Film: one conflict game, switch by switch.
def game_animation(out, path, game=None, fps=10, dpi=80):
    """One conflict game (Section 0.10.1), switch by switch. For every strategy profile, from all-GO to
    the equilibrium: a short clip of the sinks near the conflict (red links: closer than d_safe), a
    timeline per player of when it is stopped and when it is in conflict, and the potential Phi, which
    falls by exactly the switching player's gain at every switch (an exact potential game)."""
    # Rectangle: a filled box, for the timelines.
    from matplotlib.patches import Rectangle
    # The settings.
    p = out["params"]
    # Replay the games.
    games, trace = game_replay_data(out)
    # The game to show.
    gi = pick_game(games) if game is None else game
    # Its record and its recording.
    g, tr = games[gi], trace[gi]
    # Its players, their strategies, everyone's trajectories before it, and the
    # decision frame.
    players, acts, before, s = g["players"], tr["acts"], tr["trajs"], g["decision_frame"]
    # The three cost weights.
    beta, gamma, delta = p["weights"]
    # Each player's strategy names.
    names = {k: [a["name"] for a in acts[k]] for k in players}
    # A "profile" says which strategy every player plays. Start: everyone 0 (GO).
    profiles, pr = [], {k: 0 for k in players}
    # The first profile.
    profiles.append(dict(pr))
    # For each switch (player k changes to strategy "name"):
    for k, name, _ in g["moves"]:
        # Change that player's strategy ...
        pr[k] = names[k].index(name)
        # ... and keep a copy of the new profile.
        profiles.append(dict(pr))
    # The robots not in the game.
    fixed = [m for m in before if m not in players]

    # Everyone's trajectories under a profile.
    def trajs_of(prof):
        # Start from the trajectories before the game ...
        t = dict(before)
        # ... and give each player the trajectory of its chosen strategy.
        t.update({k: acts[k][prof[k]]["traj"] for k in players})
        # Hand them back.
        return t

    # The players' own costs added up (extra time, distance and waiting).
    def own_cost(prof):
        # (each times its weight)
        return sum(beta * a["dT"] + gamma * a["dD"] / p["v"] + delta * a["W"] for a in (acts[k][prof[k]] for k in players))

    # The danger under a profile: seconds of conflict, as the game counts it.
    def risk(prof):
        # Everyone's trajectories.
        t = trajs_of(prof)
        # Between players, each pair once ...
        shared = sum(cg.exposure(t[k], t[j], s, p) for i, k in enumerate(players) for j in players[i + 1:])
        # ... plus between players and everyone else.
        return shared + sum(cg.exposure(t[k], t[m], s, p) for k in players for m in fixed)

    # Facts about every profile, built below.
    info = []
    # For profile i:
    for i, prof in enumerate(profiles):
        # its danger and its own cost;
        rk, ck = risk(prof), own_cost(prof)
        # its potential Phi = alpha * danger + own cost;
        phi = g["alpha"] * rk + ck
        # which must equal the game's own record (else stop with an error);
        assert abs(phi - g["potential"][i]) <= 1e-6 * max(1.0, abs(phi)), "potential does not match the game record"
        # keep all of it, and its conflicts step by step.
        info.append({"trajs": trajs_of(prof), "risk": rk, "cost": ck, "phi": phi,
                     "timeline": conflict_timeline(trajs_of(prof), p)})
    # clip: from t = 0 to a little after the last conflict of the all-GO profile near the decision frame
    act0 = info[0]["timeline"]["active"]
    # The steps after the decision (up to 30 s) where a player is in conflict.
    late = [n for n in range(s, min(len(act0), s + 60)) if any(k in pair for pair in act0[n] for k in players)]
    clip = max(late[-1] if late else s, s + 12) + 6                 # the timeline covers this
    show = min(clip, s + 24)                                         # the map plays about the first 12 s
    # All positions shown on the map, at the start and at the end of the game.
    pts = np.vstack([np.vstack([f[:show + 1] for f in inf["trajs"].values()]) for inf in (info[0], info[-1])])
    # Their range, with 6 m to spare ...
    lo, hi = pts.min(0) - 6, pts.max(0) + 6
    # ... the larger side ...
    span = max(hi - lo)
    # ... the middle ...
    mid = (lo + hi) / 2
    # ... a square view.
    lims = (mid - span / 2, mid + span / 2)

    # The frames from n0 to n1 in which robot P stands still.
    def stopped(P, n0, n1):
        """Frames n in [n0, n1) at which the sink does not move between n and n + 1."""
        # Stop at its last frame.
        m = min(n1, len(P) - 1)
        # Frames where it does not move, plus every frame after it has finished.
        return [n for n in range(n0, m) if np.hypot(*(P[n + 1] - P[n])) < 1e-9] + list(range(max(m, n0), n1))

    # An 11 by 6.8 inch page.
    fig = plt.figure(figsize=(11, 6.8), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, f"The conflict game, switch by switch ({len(players)} players, {len(g['moves'])} switches)",
            f"Game {gi + 1} of {len(games)}: decided at t = {s * p['dt']:.1f} s for a conflict at t = {tr['f'] * p['dt']:.1f} s. "
            f"Every player starts with GO; at each switch the player that gains most changes its strategy.")
    # The caption line with its square.
    mark, caption = _status_line(fig, 0.015, 0.845, size=9.5)
    # The map's drawing area ...
    ax = fig.add_axes([0.03, 0.05, 0.42, 0.74])
    # ... the players' timelines' ...
    rows = fig.add_axes([0.58, 0.39, 0.39, 0.42])
    # ... and the potential chart's.
    phi_ax = fig.add_axes([0.58, 0.07, 0.2, 0.2])
    # Three tiles: risk, cost, Phi.
    tiles = []
    # For each:
    for i, name in enumerate(("Risk", "Cost", "Φ")):
        # its name ...
        fig.text(0.81 + 0.065 * i, 0.235, name, fontsize=7.5, color=QUIET, va="bottom")
        # ... and an empty big number.
        tiles.append(fig.text(0.81 + 0.065 * i, 0.225, "", fontsize=11, fontweight="bold", color=INK, va="top"))
    # A note explaining Phi.
    fig.text(0.81, 0.12, "Φ = α · risk + cost of moves (seconds);\nrisk: seconds closer than d_safe.",
             fontsize=7.5, color=QUIET, va="top")
    # The script: (profile, frame to show).
    frames = []
    # For each profile:
    for i in range(len(profiles)):
        # play the clip from 0 to "show", then pause 5 pictures.
        frames += [(i, n) for n in range(0, show + 1)] + [(i, show)] * 5
    # End with a long pause on the equilibrium.
    frames += [(len(profiles) - 1, show)] * 14
    # For each player and profile: the frames it spends WAITing.
    wait_frames = {k: {i_: set(range(s, s + int(round(acts[k][prof[k]]["W"] / p["dt"]))))
                       for i_, prof in enumerate(profiles)} for k in players}

    # Draws picture number "frame".
    def update(frame):
        # Which profile, and which frame of the clip.
        i, n = frames[min(frame, len(frames) - 1)]
        # That profile's facts.
        inf, prof = info[i], profiles[i]
        # Its trajectories and conflicts.
        trajs, tl = inf["trajs"], inf["timeline"]
        # The map afresh ...
        _draw_map(ax, out, lims)
        # ... with the docking circle.
        _docking_zone(ax, out)
        # Every robot's path over the clip:
        for k, f in trajs.items():
            # players in their colours, others light grey ...
            color = _style(k)[0] if k in players else PENDING
            # ... drawn faintly.
            ax.plot(f[:show + 1, 0], f[:show + 1, 1], color=color, lw=0.8, alpha=0.4, zorder=2)
        # Every pair in conflict at this frame:
        for a, b in (tl["active"][n] if n < len(tl["active"]) else []):
            # where they are;
            pa, pb = _position(trajs[a], n), _position(trajs[b], n)
            # a red link between them.
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=CRITICAL, lw=2.4, zorder=6)
        # Every robot now:
        for k, f in trajs.items():
            # where it is;
            x, y = _position(f, n)
            # a dot (bigger for players).
            ax.scatter([x], [y], s=46 if k in players else 22, color=_style(k)[0] if k in players else PENDING,
                       edgecolor=INK, lw=0.6, zorder=8)
            # Players also get ...
            if k in players:
                # ... their number beside the dot.
                ax.text(x + 0.6, y + 0.6, str(k), fontsize=7, color=INK, zorder=9)
        # The clock, top left of the map.
        ax.text(0.02, 0.97, f"t = {n * p['dt']:.1f} s", transform=ax.transAxes, fontsize=10, fontweight="bold",
                color=INK, va="top")
        # timeline per player: stopped (grey) and in conflict (red), with a cursor at the current time
        rows.cla()
        # The timelines, cleared above: plain.
        _quiet_axes(rows, None)
        # Time across ...
        rows.set_xlim(0, clip * p["dt"])
        # ... one row per player.
        rows.set_ylim(-0.6, len(players) - 0.4)
        # For each player:
        for r_i, k in enumerate(players):
            # its row (the first at the top);
            y = len(players) - 1 - r_i
            # a thin light track;
            rows.add_patch(Rectangle((0, y - 0.08), clip * p["dt"], 0.16, color=GRID_LIGHT, lw=0, zorder=1))
            # its WAIT frames under this profile;
            waits = wait_frames[k][i]
            # every frame it stands still ...
            for m in stopped(trajs[k], 0, clip):
                # ... a box: black for a WAIT, grey for a stop at a sensor.
                rows.add_patch(Rectangle((m * p["dt"], y - 0.22), p["dt"], 0.44,
                                         color=INK if m in waits else PENDING, lw=0, zorder=2))
            # The frames it is in conflict:
            bad = set()
            # every frame of the clip ...
            for t_ in range(min(clip, len(tl["active"]))):
                # ... where it is in a conflicting pair ...
                if any(k in pair for pair in tl["active"][t_]):
                    # ... is noted;
                    bad.add(t_)
            # each one gets ...
            for m in bad:
                # ... a thin red box.
                rows.add_patch(Rectangle((m * p["dt"], y - 0.32), p["dt"], 0.18, color=CRITICAL, lw=0, zorder=3))
        # Each row labelled with the player and its current strategy.
        rows.set_yticks(range(len(players)), [f"sink {k}  {names[k][prof[k]]}" for k in players][::-1], fontsize=7.5)
        # Shade the part the map plays.
        rows.axvspan(0, show * p["dt"], color=GRID_LIGHT, alpha=0.6, lw=0, zorder=0)
        # A line at the current time.
        rows.axvline(n * p["dt"], color=INK, lw=1, zorder=4)
        # The label along the bottom.
        rows.set_xlabel("t [s]  (shaded: the part the map plays)", fontsize=8, color=MUTED)
        # The title, explaining the colours.
        rows.set_title("Players: WAIT (black), stopped at a sensor (grey), too close (red)", loc="left",
                       fontsize=8.5, color=INK, pad=4)
        # the potential, one point per profile
        phi_ax.cla()
        # The Phi chart, cleared above: plain, with horizontal grid lines.
        _quiet_axes(phi_ax, "y")
        # A log scale up the side.
        phi_ax.set_yscale("log")
        # Phi of every profile.
        phis = [x["phi"] for x in info]
        # One place per profile across ...
        phi_ax.set_xlim(-0.5, len(info) - 0.5)
        # ... room above and below.
        phi_ax.set_ylim(min(phis) / 1.6, max(phis) * 1.6)
        # Phi so far, as a staircase ...
        phi_ax.plot(range(i + 1), phis[:i + 1], color=BEST, lw=2, drawstyle="steps-post", zorder=2)
        # ... with a dot per profile.
        phi_ax.scatter(range(i + 1), phis[:i + 1], s=22, color=BEST, edgecolor=SURFACE, lw=1, zorder=3)
        # The label along the bottom.
        phi_ax.set_xlabel("Switch", fontsize=8, color=MUTED)
        # The chart's title.
        phi_ax.set_title("Potential Φ", loc="left", fontsize=8.5, color=INK, pad=4)
        # Whole numbers across.
        phi_ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        # Plain numbers up the side ...
        phi_ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        # ... none on the small marks.
        phi_ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        # The tiles: risk ...
        tiles[0].set_text(f"{inf['risk']:.1f} s")
        # ... cost ...
        tiles[1].set_text(f"{inf['cost']:.1f} s")
        # ... and Phi.
        tiles[2].set_text(f"{inf['phi']:.0f}")
        # At the start:
        if i == 0:
            # everyone GOes;
            caption.set_text(f"Start: all {len(players)} players GO. Φ = {inf['phi']:.0f}.")
            # red if there is danger.
            mark.set_color(CRITICAL if inf["risk"] > 0 else GOOD)
        # After switch i:
        else:
            # who switched, to what, and their gain;
            k, name, gain = g["moves"][i - 1]
            # is it the last switch?
            last = i == len(info) - 1
            # what the player played before;
            was = names[k][profiles[i - 1][k]]
            # the caption (with a note on the equilibrium at the end);
            caption.set_text(f"Switch {i}: sink {k} changes {was} → {name}; its own cost falls by {gain:.0f}, and "
                             f"Φ falls by exactly as much, to {inf['phi']:.0f}."
                             + ("\nNo player can lower its own cost by changing alone: a pure Nash equilibrium. "
                                f"The risk left is for later games." if last and inf["risk"] > 0 else
                                "\nNo player can lower its own cost by changing alone: a pure Nash equilibrium."
                                if last else ""))
            # green at the end.
            mark.set_color(GOOD if last else INK)
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(frames), path, fps)
    # Hand back the game's number and record.
    return gi, g


# A very light grey, for the timelines' tracks.
GRID_LIGHT = "#ecebe6"


# ---------------------------------------------------------------------------
# 10. The six Dubins turn types as the end heading turns
# ---------------------------------------------------------------------------

# Film: the six Dubins shapes of one leg, as its end heading turns round.
def dubins_rotation_animation(path, R=5.0, end=(12.0, 5.0), step_deg=4, fps=8, dpi=80, hold=16):
    """For one leg from (0, 0) heading east to `end`, the end heading turns through 360 degrees. Every frame
    draws all six turn types with their lengths, the shortest (blue, solid) and the one the rule of Section
    0.13 picks (orange, dashed; if its type does not exist, the code falls back to the shortest)."""
    # This project's Dubins paths (nicknamed dp).
    import dubins_path as dp
    # The two methods' colours.
    from plot import FEASIBLE, SHORTEST
    # The start pose: at (0, 0), heading east (angle 0).
    x1, y1, psi1 = 0.0, 0.0, 0.0
    # The end point.
    x2, y2 = end
    # Facts for every end heading, built below.
    data = []
    # For every end heading, every 4 degrees round the circle:
    for deg in np.arange(0, 360, step_deg):
        # the heading in radians;
        psi2 = np.radians(deg)
        # the three lengths of each of the six shapes;
        parts = {t: tuple(float(v) for v in dp.calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in dp.TURN_TYPES}
        # each shape's total length (infinity if it does not exist);
        total = {t: sum(v) for t, v in parts.items()}
        # the shortest shape;
        best = min(total, key=total.get)
        # the shape the paper's rule picks;
        rule = dp.findTurnType(x1, y1, psi1, x2, y2, psi2, R)
        # the one used: the rule's, or the shortest if the rule's does not exist;
        pick = rule if np.isfinite(total[rule]) else best
        # the points of every shape that exists:
        xy = {}
        # for each shape ...
        for t, v in parts.items():
            # ... that exists ...
            if np.isfinite(total[t]):
                # ... its points, 0.1 m apart ...
                xs, ys, _, _ = dp.calculateTrajectory(x1, y1, psi1, R, t, *v, 0.1)
                # ... kept as rows (x, y).
                xy[t] = np.column_stack([xs, ys])
        # Keep everything about this heading.
        data.append({"deg": deg, "psi2": psi2, "total": total, "best": best, "rule": rule, "pick": pick, "xy": xy})
    # All points of all shapes of all headings.
    allpts = np.vstack([a for d in data for a in d["xy"].values()])
    # Their range, with 2 m to spare.
    lo, hi = allpts.min(0) - 2, allpts.max(0) + 2
    # The headings where the shape used is not the shortest ...
    longer = [d for d in data if d["pick"] != d["best"]]
    # ... and where the rule's shape did not exist.
    missing = [d for d in data if d["pick"] != d["rule"]]

    # An 11 by 6.8 inch page.
    fig = plt.figure(figsize=(11, 6.8), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, "The six Dubins turn types as the end heading turns",
            f"From (0, 0) heading east to ({x2:g}, {y2:g}), R = {R:g} m. Blue: the shortest of the six. Orange, dashed: "
            f"the type the rule of Section 0.13 picks (from the signs of cp1, cp2 and S_tan).")
    # The caption, filled in by the film.
    caption = fig.text(0.015, 0.855, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # A big drawing area on the left ...
    big = fig.add_axes([0.02, 0.06, 0.44, 0.76])
    # ... six small ones (2 rows of 3), one per shape ...
    thumbs = [fig.add_axes([0.5 + 0.165 * (i % 3), 0.58 - 0.225 * (i // 3), 0.15, 0.16]) for i in range(6)]
    # ... and a chart along the bottom right.
    chart = fig.add_axes([0.54, 0.07, 0.43, 0.2])
    # The script: every heading once, then a pause on the last.
    frames = list(range(len(data))) + [len(data) - 1] * hold

    # Draws a heading arrow on drawing area a, at (x, y), pointing at angle psi.
    def arrow(a, x, y, psi, color=INK, length=2.6):
        # An arrow from the point, 2.6 m long.
        a.annotate("", xy=(x + length * np.cos(psi), y + length * np.sin(psi)), xytext=(x, y),
                   arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2, mutation_scale=9, shrinkA=0, shrinkB=0))

    # Draws picture number "frame".
    def update(frame):
        # Which heading.
        i = frames[min(frame, len(frames) - 1)]
        # Its facts.
        d = data[i]
        # Clear the big area ...
        big.cla()
        # ... equal scales ...
        big.set_aspect("equal")
        # ... no axes ...
        big.axis("off")
        # ... the same view every time ...
        big.set_xlim(lo[0], hi[0])
        # (up and down too).
        big.set_ylim(lo[1], hi[1])
        # The shortest shape, blue.
        big.plot(*d["xy"][d["best"]].T, color=SHORTEST, lw=2.6, zorder=3)
        # If the rule uses another one ...
        if d["pick"] != d["best"]:
            # ... it too, orange and dashed.
            big.plot(*d["xy"][d["pick"]].T, color=FEASIBLE, lw=2.2, ls=DASHES, zorder=4)
        # The start and the end pose:
        for (x, y, psi) in ((x1, y1, psi1), (x2, y2, d["psi2"])):
            # a dot ...
            big.scatter([x], [y], s=30, color=INK, zorder=5)
            # ... and its heading arrow.
            arrow(big, x, y, psi)
        # The word "start" ...
        big.text(x1 - 0.5, y1 - 1.2, "start", fontsize=8, color=INK, ha="right", va="top")
        # ... and "end" with its heading.
        big.text(x2 + 0.6, y2 + 0.6, f"end, ψ₂ = {d['deg']:.0f}°", fontsize=8, color=INK)
        # Each small area shows one shape t:
        for t, a in zip(dp.TURN_TYPES, thumbs):
            # clear it;
            a.cla()
            # equal scales;
            a.set_aspect("equal")
            # no numbers across ...
            a.set_xticks([])
            # ... or up;
            a.set_yticks([])
            # the same view as the big one ...
            a.set_xlim(lo[0], hi[0])
            # (up and down too);
            a.set_ylim(lo[1], hi[1])
            # a thin grey border:
            for side in a.spines.values():
                # shown,
                side.set_visible(True)
                # grey,
                side.set_color(GRID)
                # thin,
                side.set_linewidth(0.8)
                # solid.
                side.set_linestyle("-")
            # Notes about this shape:
            tags = []
            # the shortest?
            if t == d["best"]:
                # say so ...
                tags.append("shortest")
                # ... and give it a thick blue border.
                for side in a.spines.values():
                    # (blue)
                    side.set_color(SHORTEST)
                    # (thick)
                    side.set_linewidth(2.4)
            # the rule's pick?
            if t == d["rule"]:
                # say so (and if it does not exist) ...
                tags.append("rule's pick" if np.isfinite(d["total"][t]) else "rule's pick, missing")
                # ... and if it is not also the shortest ...
                if t != d["best"]:
                    # ... give it an orange dashed border.
                    for side in a.spines.values():
                        # (orange)
                        side.set_color(FEASIBLE)
                        # (thick)
                        side.set_linewidth(2.2)
                        # (dashed)
                        side.set_linestyle((0, (3, 2)))
            # If the shape exists:
            if t in d["xy"]:
                # draw it (blue if shortest, orange if picked, else black);
                a.plot(*d["xy"][t].T, color=INK if t not in (d["best"], d["pick"]) else
                       (SHORTEST if t == d["best"] else FEASIBLE), lw=1.2)
                # its name and length.
                title = f"{t}  {d['total'][t]:.1f} m"
            # If not:
            else:
                # say so.
                title = f"{t}  does not exist"
            # The title, with the notes on a second line.
            a.set_title(title + ("\n" + ", ".join(tags) if tags else "\n "), fontsize=7.5, color=INK, pad=2)
        # The chart: clear it ...
        chart.cla()
        # ... plain ...
        _quiet_axes(chart, "y")
        # The headings so far.
        degs = [x["deg"] for x in data[:i + 1]]
        # The shortest length at each, blue ...
        chart.plot(degs, [x["total"][x["best"]] for x in data[:i + 1]], color=SHORTEST, lw=2, label="shortest")
        # ... and the length used by the rule, orange dashed.
        chart.plot(degs, [x["total"][x["pick"]] for x in data[:i + 1]], color=FEASIBLE, lw=2, ls=DASHES,
                   label="rule of Section 0.13")
        # Headings 0 to 360 degrees across ...
        chart.set_xlim(0, 360)
        # ... lengths up.
        chart.set_ylim(0, max(x["total"][x["pick"]] for x in data) * 1.08)
        # Marks every 45 degrees.
        chart.set_xticks(range(0, 361, 45))
        # Labels along the bottom ...
        chart.set_xlabel("End heading ψ₂ [degrees]", fontsize=8, color=MUTED)
        # ... and up the side.
        chart.set_ylabel("Length [m]", fontsize=8, color=MUTED)
        # The chart's legend.
        chart.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK, ncol=2)
        # During the final pause: the summary caption.
        if frame >= len(data):
            # (how often the rule was longer, by up to how much, and how often missing)
            caption.set_text(f"Over all {len(data)} headings: the rule picks a longer path at {len(longer)} of them "
                             f"(up to +{100 * max((x['total'][x['pick']] / x['total'][x['best']] - 1) for x in longer):.0f} %),"
                             f"\nand at {len(missing)} its type does not exist, so the code uses the shortest instead.")
        # The rule's shape is longer:
        elif d["pick"] != d["best"]:
            # say by how much.
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the shortest is {d['best']} ({d['total'][d['best']]:.1f} m); "
                             f"the rule picks {d['pick']} ({d['total'][d['pick']]:.1f} m, "
                             f"+{100 * (d['total'][d['pick']] / d['total'][d['best']] - 1):.0f} %).")
        # The rule's shape does not exist:
        elif d["pick"] != d["rule"]:
            # say so.
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the rule picks {d['rule']}, which does not exist here, "
                             f"so the code uses the shortest, {d['best']} ({d['total'][d['best']]:.1f} m).")
        # The rule picks the shortest:
        else:
            # say so.
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the rule picks {d['best']}, the shortest "
                             f"({d['total'][d['best']]:.1f} m).")
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(frames), path, fps)
    # Hand back the counts.
    return {"headings": len(data), "longer": len(longer), "missing": len(missing)}


# ---------------------------------------------------------------------------
# 11. The geodesic distance spreading from the base (Dijkstra)
# ---------------------------------------------------------------------------

# Film: the distance round obstacles spreading from the base like a wave.
def geodesic_wave_animation(out, path, steps=70, fps=10, dpi=80, hold=20):
    """The shortest-path tree of geodesic.build from the base, grown by distance: at every frame each tree
    edge is drawn as far as the geodesic distance rho has reached, and each sensor lights up when the wave
    gets there (its r of eq. 9). The dashed circle is the straight-line distance rho: where an obstacle is in
    the way, the wave lags behind it. Beside the map, r against the straight-line distance of every sensor
    reached so far."""
    # This project's distances round obstacles.
    import geodesic
    # Short names for the map, the distances and the settings.
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    # The graph's points, how many are terminals, and Dijkstra's "came from"
    # list for the run from the base.
    nodes, n_t, pred = geo["nodes"], geo["n_terminals"], geo["pred"][0]
    # The base.
    base = scen["base"]
    # Each point's distance from the base, unknown (infinity) at first ...
    dist = np.full(len(nodes), np.inf)
    # ... except the base itself: 0.
    dist[0] = 0.0
    for v in range(len(nodes)):                             # distance along the tree, by walking up to the base
        # Walk back from point v along "came from" ...
        chain, u = [], v
        # ... until a point with a known distance (or the end):
        while not np.isfinite(dist[u]) and pred[u] >= 0:
            # note the point ...
            chain.append(u)
            # ... and step back.
            u = pred[u]
        # If we reached a known distance ...
        if np.isfinite(dist[u]):
            # ... go forward again along the chain ...
            for w in reversed(chain):
                # ... adding each step's length.
                dist[w] = dist[pred[w]] + float(np.hypot(*(nodes[w] - nodes[pred[w]])))
    # The sensors' distances (terminals 1 to n).
    r = dist[1:n_t]
    # They must equal geodesic.py's own answers (else stop with an error).
    assert np.allclose(r[np.isfinite(r)], geo["D"][0, 1:][np.isfinite(r)]), "the tree does not give D"
    # The straight-line distances, for comparison.
    euclid = np.hypot(*(scen["xy"] - base).T)
    # The obstacles grown by 7 m.
    grown = geodesic.inflate(scen["obstacles"], p["r_plan"])
    # The largest distance of any reachable sensor.
    top = float(np.nanmax(np.where(np.isfinite(r), r, np.nan)))
    # The wave's reach in each picture: 70 steps from 0 to just past the top,
    # then a pause.
    rhos = list(np.linspace(0, top * 1.02, steps)) + [top * 1.02] * hold
    # The area of map to show.
    lims = _limits(out, [nodes[np.isfinite(dist)]])
    # How much longer than the straight line each sensor's distance is (0.1 = 10 %).
    detour = np.where(np.isfinite(r), r / np.maximum(euclid, 1e-9) - 1, np.nan)

    # An 11 by 6.6 inch page.
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    # The title and subtitle.
    _header(fig, "The geodesic distance r spreading from the base (eq. 9, Dijkstra on the visibility graph)",
            "Blue: shortest paths from the base, drawn as far as the distance ρ has reached. They bend only at corners "
            "of the obstacles grown by 7 m (thin outlines) and at sensors.")
    # The caption, filled in by the film.
    caption = fig.text(0.015, 0.865, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    # The map's drawing area ...
    ax = fig.add_axes([0.045, 0.04, 0.5, 0.8])
    # ... and the chart's.
    chart = fig.add_axes([0.62, 0.36, 0.35, 0.44])
    # Three tiles (the fourth is left empty).
    tiles = _tiles(fig, 0.6, 0.2, ("Distance ρ", "Sensors reached", "Largest detour", ""))

    # Draws picture number "frame".
    def update(frame):
        # How far the wave has reached.
        rho = rhos[min(frame, len(rhos) - 1)]
        # The map afresh.
        _draw_map(ax, out, lims)
        # The grown obstacles' outlines:
        for g_ in grown:
            # the x and y values ...
            gx, gy = g_.exterior.xy
            # ... drawn thin and faint.
            ax.plot(gx, gy, color=QUIET, lw=0.5, alpha=0.5, zorder=2)
        # 200 angles round a full turn ...
        t = np.linspace(0, 2 * np.pi, 200)
        # ... make a dashed circle of radius rho: where the wave would be with no obstacles.
        ax.plot(base[0] + rho * np.cos(t), base[1] + rho * np.sin(t), color=MUTED, lw=1.0, ls=DASHES, zorder=3)
        # Every point v except the base:
        for v in range(1, len(nodes)):
            # the point it is reached from;
            u = pred[v]
            # if it is unreachable, or the wave has not reached u yet ...
            if u < 0 or not np.isfinite(dist[v]) or rho <= dist[u]:
                # ... skip it;
                continue
            # how far along the line from u to v the wave has got (0 to 1);
            frac = min(1.0, (rho - dist[u]) / max(dist[v] - dist[u], 1e-9))
            # the point that far along;
            end_ = nodes[u] + frac * (nodes[v] - nodes[u])
            # draw the line that far, in blue.
            ax.plot([nodes[u][0], end_[0]], [nodes[u][1], end_[1]], color=BEST, lw=1.2, zorder=4)
        # The sensors the wave has reached.
        reached = np.isfinite(r) & (r <= rho)
        # Those whose distance is more than 2 % longer than the straight line.
        around = reached & (detour > 0.02)
        # Sensors not reached: light grey.
        ax.scatter(*scen["xy"][~reached].T, s=10, color=PENDING, zorder=5)
        # Reached: black.
        ax.scatter(*scen["xy"][reached].T, s=16, color=INK, zorder=6)
        # Reached the long way round: an orange ring.
        ax.scatter(*scen["xy"][around].T, s=90, facecolor="none", edgecolor=POP, lw=1.6, zorder=7)
        # The corner points.
        corners = np.arange(n_t, len(nodes))
        # Those the wave has reached ...
        hit = corners[dist[corners] <= rho]
        # ... as small blue dots.
        ax.scatter(*nodes[hit].T, s=6, color=BEST, zorder=5)
        # The chart: clear it ...
        chart.cla()
        # ... plain ...
        _quiet_axes(chart, "y")
        # ... the same range both ways ...
        lim = max(top, euclid.max()) * 1.05
        # (across)
        chart.set_xlim(0, lim)
        # (up)
        chart.set_ylim(0, lim)
        # A dashed diagonal: where the two distances are equal.
        chart.plot([0, lim], [0, lim], color=MUTED, lw=1, ls=DASHES)
        # Its label.
        chart.text(lim * 0.97, lim * 0.9, "r = straight-line distance", fontsize=7.5, color=MUTED, ha="right")
        # Each reached sensor: straight-line distance across, distance round
        # obstacles up. Blue if the straight line is clear ...
        chart.scatter(euclid[reached & ~around], r[reached & ~around], s=16, color=BEST, edgecolor=SURFACE, lw=0.6,
                      zorder=3, label="straight line is clear")
        # ... orange if it goes round.
        chart.scatter(euclid[around], r[around], s=22, color=POP, edgecolor=SURFACE, lw=0.6, zorder=4,
                      label="goes round an obstacle (r > 2 % longer)")
        # The chart's legend.
        chart.legend(loc="upper left", frameon=False, fontsize=7.5, labelcolor=INK)
        # Labels along the bottom ...
        chart.set_xlabel("Straight-line distance from the base [m]", fontsize=8, color=MUTED)
        # ... and up the side.
        chart.set_ylabel("Geodesic distance r [m]", fontsize=8, color=MUTED)
        # The chart's title.
        chart.set_title("Each sensor reached: r against straight-line distance", loc="left",
                        fontsize=8.5, color=INK, pad=4)
        # The caption: how far, how many reached, how many went round.
        caption.set_text(f"ρ = {rho:.0f} m: the wave has reached {int(reached.sum())} of {len(r)} sensors; "
                         f"{int(around.sum())} of them (orange rings) had to go round an obstacle.")
        # The tiles: the wave's reach ...
        tiles[0].set_text(f"{rho:.0f} m")
        # ... sensors reached ...
        tiles[1].set_text(f"{int(reached.sum())}")
        # ... and the largest detour so far.
        tiles[2].set_text(f"+{100 * np.nanmax(detour[reached]):.0f} %" if reached.any() else "—")
        # Nothing to hand back.
        return []

    # Make and save the film.
    _save(fig, update, len(rhos), path, fps)
    # Hand back the number of sensors and the largest detour.
    return {"sensors": len(r), "largest_detour": float(np.nanmax(detour))}
