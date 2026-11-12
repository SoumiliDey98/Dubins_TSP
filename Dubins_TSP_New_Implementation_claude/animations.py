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
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from scipy.spatial import ConvexHull

import conflict_game as cg
import ga_tour
import trajectory
from plot import (BEST, DASHES, GRID, INK, MUTED, POP, QUIET, SURFACE, _map, _quiet_axes, _style, ga_summary,
                  save_gif, stop_indices)

CRITICAL, GOOD = "#d03b3b", "#0ca30c"        # status colours; always shown with a label, never alone
PENDING = "#c9c8c1"                          # sensors of groups not planned yet


# ---------------------------------------------------------------------------
# Shared pieces
# ---------------------------------------------------------------------------

def _header(fig, title, subtitle):
    fig.text(0.015, 0.975, title, fontsize=12, fontweight="bold", color=INK, va="top")
    fig.text(0.015, 0.932, subtitle, fontsize=9, color=QUIET, va="top")


def _limits(out, extra=()):
    """Map limits that hold the sensors, the base and every given polyline."""
    pts = np.vstack([out["scenario"]["xy"], out["scenario"]["base"][None, :]] + [np.asarray(e) for e in extra])
    lo, hi = pts.min(0) - 4, pts.max(0) + 4
    return lo, hi


def _draw_map(ax, out, lims):
    """Clear the axes and draw obstacles, their r_safe boundary and the base."""
    ax.cla()
    _map(ax, out)
    ax.set_facecolor(SURFACE)
    ax.tick_params(length=0, labelsize=7, colors=MUTED)
    for side in ax.spines:
        ax.spines[side].set_visible(False)
    ax.set_xlim(lims[0][0], lims[1][0])
    ax.set_ylim(lims[0][1], lims[1][1])


def _save(fig, update, n_frames, path, fps):
    save_gif(fig, update, n_frames, path, fps)


def conflict_timeline(trajs, p):
    """For every interval [frame n, n+1]: the pairs closer than d_safe (eq. 20, outside the docking
    zone), the closest distance between any two sinks outside the docking zone, and the time in
    conflict summed over pairs up to and including that interval. The total equals the exposure of
    conflict_game.summary."""
    keys = sorted(trajs)
    n = max(len(f) for f in trajs.values())
    active = [[] for _ in range(n)]
    closest = np.full(n, np.inf)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            d, dock = cg.closest(trajs[a], trajs[b], p)
            for t in np.flatnonzero((d < p["d_safe"]) & ~dock):
                active[t].append((a, b))
            ok = np.flatnonzero(~dock)
            closest[ok] = np.minimum(closest[ok], d[ok])
    cum = p["dt"] * np.cumsum([len(a) for a in active])
    return {"active": active, "closest": closest, "cum": cum}


def _position(f, n):
    return f[min(n, len(f) - 1)]


def _draw_moving(ax, trajs, n, active, faint=0.25):
    """Every sink's whole path (faint), its position at frame n, and a red link between any two
    sinks that are closer than d_safe in the interval that starts at frame n."""
    for k, f in trajs.items():
        ax.plot(f[:, 0], f[:, 1], color=_style(k)[0], lw=0.7, alpha=faint, zorder=2)
    for a, b in (active[n] if n < len(active) else []):
        pa, pb = _position(trajs[a], n), _position(trajs[b], n)
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=CRITICAL, lw=2.4, zorder=6, solid_capstyle="round")
        ax.scatter([pa[0], pb[0]], [pa[1], pb[1]], s=150, facecolor="none", edgecolor=CRITICAL, lw=1.8, zorder=7)
    for k, f in trajs.items():
        x, y = _position(f, n)
        ax.scatter([x], [y], s=34, color=_style(k)[0], edgecolor=INK, lw=0.6, zorder=8)


def _docking_zone(ax, out):
    base, rad = out["scenario"]["base"], out["params"]["base_radius"]
    t = np.linspace(0, 2 * np.pi, 100)
    ax.plot(base[0] + rad * np.cos(t), base[1] + rad * np.sin(t), color=MUTED, lw=0.7, ls=(0, (1, 1.5)), zorder=3)


def split_states(log):
    """For every GA run j of plan_routes, in order: the runs already kept as sinks and the groups
    still waiting when run j starts. Reconstructed from the queue rule of plan_routes (first in,
    first out; a split group's two halves go to the back)."""
    states, kept, waiting = [], [], [np.asarray(log[0]["ids"])]
    for j, run in enumerate(log):
        current = waiting.pop(0)
        assert sorted(current.tolist()) == sorted(np.asarray(run["ids"]).tolist()), "the GA log does not follow the queue"
        states.append({"kept": list(kept), "waiting": [np.asarray(w) for w in waiting]})
        if run["kept"]:
            kept.append(j)
        else:
            waiting += [np.asarray(h) for h in run.get("halves", [])]
    return states


def _hull(ax, pts, **kw):
    if len(pts) >= 3:
        try:
            h = ConvexHull(pts)
        except Exception:
            return
        v = pts[np.append(h.vertices, h.vertices[0])]
        ax.plot(v[:, 0], v[:, 1], **kw)


def _draw_split(ax, out, log, states, j, decided):
    """Run j of the splitting: kept sinks in their colours, waiting groups grey inside a dotted outline,
    the current group's tour in black, or once decided in green (kept) or red (split)."""
    xy = out["scenario"]["xy"]
    st = states[j]
    for k, jj in enumerate(st["kept"]):
        color = _style(k)[0]
        ax.plot(*np.asarray(log[jj]["path"]).T, color=color, lw=0.9, alpha=0.85, zorder=3)
        ax.scatter(*xy[log[jj]["ids"]].T, s=12, color=color, zorder=5)
    for w in st["waiting"]:
        ax.scatter(*xy[w].T, s=10, color=PENDING, zorder=4)
        _hull(ax, xy[w], color=QUIET, lw=0.7, ls=(0, (1, 1.5)), zorder=4)
    run = log[j]
    color = INK if not decided else (GOOD if run["kept"] else CRITICAL)
    ax.plot(*np.asarray(run["path"]).T, color=color, lw=1.5, zorder=6)
    ax.scatter(*xy[run["ids"]].T, s=14, color=INK, zorder=7)
    if decided and not run["kept"]:
        for h in run.get("halves", []):
            _hull(ax, xy[h], color=CRITICAL, lw=1.0, ls=DASHES, zorder=7)


def _split_text(run, j, k_next, B):
    n = len(run["ids"])
    if run["kept"] and run["T"] > B:
        return (f"GA run {j}: one sensor, T = {run['T']:.1f} s > B = {B:g} s  →  kept as sink {k_next}, "
                f"reported as a problem", CRITICAL)
    if run["kept"]:
        return f"GA run {j}: {n} sensors, T = {run['T']:.1f} s ≤ B = {B:g} s  →  kept as sink {k_next}", GOOD
    return f"GA run {j}: {n} sensors, T = {run['T']:.1f} s > B = {B:g} s  →  split in two", CRITICAL


def _status_line(fig, x, y, size=10):
    """A text line with a coloured square in front of it, for status messages (the colour is never alone)."""
    mark = fig.text(x, y, "■", fontsize=size + 2, color=INK, va="bottom")
    text = fig.text(x + 0.018, y, "", fontsize=size, fontweight="bold", color=INK, va="bottom")
    return mark, text


def _split_chart(ax, log, j, B, decided=True):
    """Tour time T of every GA run so far (log scale), green if kept, red if split, with the budget B."""
    ax.cla()
    _quiet_axes(ax, "y")
    ax.set_yscale("log")
    Ts = np.array([r["T"] for r in log])
    ax.set_xlim(-0.8, len(log) - 0.2)
    ax.set_ylim(min(Ts.min(), B) / 1.4, Ts.max() * 1.4)
    ax.axhline(B, color=INK, lw=1, ls=DASHES, zorder=2)
    ax.text(len(log) - 0.4, B * 1.06, f"B = {B:g} s", fontsize=8, color=INK, ha="right", va="bottom")
    shown = range(j + 1) if decided else range(j)
    for i in shown:
        ax.scatter([i], [log[i]["T"]], s=36, color=GOOD if log[i]["kept"] else CRITICAL,
                   edgecolor=SURFACE, lw=1.2, zorder=4)
    ax.set_xlabel("GA run", fontsize=8.5, color=MUTED)
    ax.set_ylabel("Tour time T [s]", fontsize=8.5, color=MUTED)
    ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
    ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    ax.legend(handles=[Line2D([], [], ls="", marker="o", ms=6, color=GOOD, label="kept: T ≤ B"),
                       Line2D([], [], ls="", marker="o", ms=6, color=CRITICAL, label="split: T > B")],
              loc="upper right", frameon=False, fontsize=8, labelcolor=INK)


# ---------------------------------------------------------------------------
# 1. Conflicts before and after the game
# ---------------------------------------------------------------------------

def conflict_animation(out, path, fps=10, step=1, dpi=72, hold=20):
    """Two maps side by side: the sinks as planned (Steps 1-7) and after the conflict game (Steps 8-10).
    Red links join two sinks closer than d_safe; below, the closest distance between any two sinks and
    the time spent in conflict so far, for both versions."""
    res, p = out["result"], out["params"]
    before = {k: t["frames"] for k, t in enumerate(res["tours"])}
    after = res["frames"]
    tb, ta = conflict_timeline(before, p), conflict_timeline(after, p)
    n = max(len(tb["cum"]), len(ta["cum"]))
    lims = _limits(out, list(before.values()) + list(after.values()))
    fig = plt.figure(figsize=(11, 7.2), dpi=dpi, facecolor=SURFACE)
    _header(fig, "Conflicts between sinks, before and after the conflict game",
            f"All {len(before)} sinks leave the base at t = 0. Red: two sinks closer than d_safe = {p['d_safe']:g} m "
            f"(eq. 20), outside the dotted docking zone round the base.")
    left = fig.add_axes([0.02, 0.27, 0.46, 0.55])
    right = fig.add_axes([0.52, 0.27, 0.46, 0.55])
    chart = fig.add_axes([0.07, 0.06, 0.6, 0.15])
    label_l = fig.text(0.02, 0.835, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    label_r = fig.text(0.52, 0.835, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    clock = fig.text(0.72, 0.17, "", fontsize=11, fontweight="bold", color=INK, va="top")
    fig.text(0.72, 0.125, "Closest distance between any two sinks;", fontsize=8.5, color=QUIET, va="top")
    fig.text(0.72, 0.095, "the dashed line is d_safe.", fontsize=8.5, color=QUIET, va="top")
    time = p["dt"] * np.arange(n)

    def update(frame):
        i = min(frame * step, n - 1)
        for ax, trajs, tl, lab, name in ((left, before, tb, label_l, "Planned (Steps 1-7)"),
                                         (right, after, ta, label_r, "After the conflict game (Steps 8-10)")):
            _draw_map(ax, out, lims)
            _docking_zone(ax, out)
            _draw_moving(ax, trajs, i, tl["active"])
            now = len(tl["active"][i]) if i < len(tl["active"]) else 0
            spent = tl["cum"][min(i, len(tl["cum"]) - 1)]
            lab.set_text(f"{name}\ntime in conflict so far {spent:.1f} s,  pairs in conflict now {now}")
        chart.cla()
        _quiet_axes(chart, "y")
        chart.set_xlim(0, time[-1])
        chart.set_ylim(0, 20)
        chart.axhline(p["d_safe"], color=INK, lw=1, ls=DASHES)
        for tl, color, ls, name in ((tb, POP, DASHES, "planned"), (ta, BEST, "-", "after the game")):
            c = np.minimum(tl["closest"][:i + 1], 20)
            chart.plot(time[:len(c)], c, color=color, lw=2, ls=ls, label=name)
        chart.set_xlabel("t [s]", fontsize=8.5, color=MUTED)
        chart.set_ylabel("metres", fontsize=8.5, color=MUTED)
        chart.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK, ncol=2)
        clock.set_text(f"t = {time[i]:.1f} s")
        return []

    _save(fig, update, (n - 1) // step + 1 + hold, path, fps)
    return {"before": tb["cum"][-1], "after": ta["cum"][-1]}


# ---------------------------------------------------------------------------
# 2. Splitting into sinks
# ---------------------------------------------------------------------------

def splitting_animation(out, log, path, fps=3, dpi=80, hold=8):
    """One step per GA run: the current group's tour is planned (black), then kept (green, T <= B) or
    split in two (red, with the two halves outlined). Kept sinks stay in their colours; groups still
    waiting are grey. Beside the map, the tour time of every run so far against B."""
    p = out["params"]
    states = split_states(log)
    lims = _limits(out, [r["path"] for r in log])
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    _header(fig, "Splitting into sinks until every tour fits the budget (Steps 1-7)",
            f"One sink with all {len(log[0]['ids'])} sensors would need {log[0]['T']:.0f} s. Every group whose tour takes "
            f"longer than B = {p['B']:g} s is split in two (Section 0.9) and planned again.")
    ax = fig.add_axes([0.045, 0.06, 0.48, 0.78])
    chart = fig.add_axes([0.6, 0.3, 0.37, 0.5])
    mark, status = _status_line(fig, 0.015, 0.865)
    counts = fig.text(0.6, 0.18, "", fontsize=10, color=INK, va="top")
    frames = [(j, d) for j in range(len(log)) for d in (False, True, True)]

    def update(frame):
        j, decided = frames[min(frame, len(frames) - 1)]
        _draw_map(ax, out, lims)
        _draw_split(ax, out, log, states, j, decided)
        _split_chart(chart, log, j, p["B"], decided)
        if decided:
            text, color = _split_text(log[j], j, len(states[j]["kept"]), p["B"])
        else:
            text, color = f"GA run {j}: planning a group of {len(log[j]['ids'])} sensors", INK
        status.set_text(text)
        mark.set_color(color)
        kept = len(states[j]["kept"]) + (1 if decided and log[j]["kept"] else 0)
        waiting = len(states[j]["waiting"]) + (len(log[j].get("halves", [])) if decided else 0)
        counts.set_text(f"Sinks kept: {kept}\nGroups still waiting: {waiting}")
        return []

    _save(fig, update, len(frames) + hold, path, fps)


# ---------------------------------------------------------------------------
# 2b. The partition, one sink more at every step
# ---------------------------------------------------------------------------

def partition_states(log):
    """The partition of the sensors after each split, from one group to the final sinks.

    Returns (states, splits, slot). states[k] lists the k + 1 groups of the partition, each as the
    index of the GA run that planned it (every group in the queue gets one). splits[k] is the run whose
    group is split to go from states[k] to states[k + 1]. slot[j] is a colour number that a group keeps
    until it is split: its first half keeps it and its second half takes the next free one."""
    run_of = {frozenset(np.asarray(r["ids"]).tolist()): j for j, r in enumerate(log)}
    groups, slot, free = [0], {0: 0}, 1
    states, splits = [[0]], []
    for j, r in enumerate(log):
        if r["kept"]:
            continue
        halves = [run_of.get(frozenset(np.asarray(h).tolist())) for h in r.get("halves", [])]
        if len(halves) != 2 or None in halves:
            break                                       # the planner stopped before planning both halves (K_max)
        i = groups.index(j)
        groups[i:i + 1] = halves
        slot[halves[0]], slot[halves[1]] = slot[j], free
        free += 1
        splits.append(j)
        states.append(list(groups))
    return states, splits, slot


def partition_animation(out, log, path, fps=2, dpi=80, hold=6):
    """The partition growing one sink at a time: 1 sink, then 2, then 3, ... Every group is drawn with
    the Dubins tour its GA run gives it, in a colour it keeps until it is split; beside the map, each
    group's tour time against B. The group that is split next is outlined in red."""
    scen, p = out["scenario"], out["params"]
    xy, B = scen["xy"], p["B"]
    states, splits, slot = partition_states(log)
    lims = _limits(out, [r["path"] for r in log])
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    _header(fig, "Partitioning: one sink, then two, then three, until every tour fits the budget",
            f"Each group's tour is the Dubins tour its own GA run gives it. A group whose tour takes longer than "
            f"B = {B:g} s is split in two (Section 0.9).")
    ax = fig.add_axes([0.045, 0.05, 0.48, 0.79])
    chart = fig.add_axes([0.6, 0.42, 0.37, 0.38])
    mark, status = _status_line(fig, 0.045, 0.865)
    tiles = _tiles(fig, 0.56, 0.25, ("Sinks K", "Longest T_max", "Total L_total", "Tours over B"))
    fig.text(0.56, 0.09, "Bars use the colours of the map. There are only eight colours, so from the\n"
             "ninth group on they repeat, with dashed tours and hatched bars.", fontsize=8, color=QUIET, va="bottom")
    frames = []
    for k in range(len(states)):
        frames += [(k, False)] * 2
        if k < len(splits):
            frames += [(k, True)] * 2
    frames += [(len(states) - 1, False)] * hold
    Tmax_all = max(r["T"] for r in log)

    def update(frame):
        k, highlight = frames[min(frame, len(frames) - 1)]
        groups = states[k]
        _draw_map(ax, out, lims)
        for j in groups:
            color, ls = _style(slot[j])
            ax.plot(*np.asarray(log[j]["path"]).T, color=color, ls=ls, lw=1.1, zorder=3)
            ax.scatter(*xy[log[j]["ids"]].T, s=13, color=color, zorder=5)
        if highlight:
            j = splits[k]
            _hull(ax, xy[log[j]["ids"]], color=CRITICAL, lw=1.8, ls=DASHES, zorder=6)
        # tour time of every group against B
        chart.cla()
        _quiet_axes(chart, "y")
        chart.set_yscale("log")
        chart.set_ylim(min(min(r["T"] for r in log), B) / 1.5, Tmax_all * 1.6)
        chart.set_xlim(-0.6, max(len(s) for s in states) - 0.4)
        y0 = chart.get_ylim()[0]
        for i, j in enumerate(groups):
            color = _style(slot[j])[0]
            hatch = "////" if slot[j] >= 8 else None
            chart.bar([i], [log[j]["T"] - y0], bottom=y0, width=0.62, color=color, hatch=hatch, edgecolor=SURFACE,
                      lw=0, zorder=3)
            if highlight and j == splits[k]:
                chart.text(i, log[j]["T"] * 1.12, "split\nnext", fontsize=7.5, color=INK, ha="center", va="bottom")
        chart.axhline(B, color=INK, lw=1, ls=DASHES, zorder=4)
        chart.text(max(len(s) for s in states) - 0.5, B * 1.06, f"B = {B:g} s", fontsize=8, color=INK, ha="right",
                   va="bottom")
        chart.set_xticks(range(len(groups)), [str(i + 1) for i in range(len(groups))], fontsize=7)
        chart.set_xlabel("Group", fontsize=8.5, color=MUTED)
        chart.set_ylabel("Tour time T [s], log scale", fontsize=8.5, color=MUTED)
        chart.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        chart.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        Ts = [log[j]["T"] for j in groups]
        over = sum(t > B for t in Ts)
        if highlight:
            j = splits[k]
            status.set_text(f"{len(groups)} sink{'s' if len(groups) > 1 else ''}: the group of {len(log[j]['ids'])} "
                            f"sensors needs T = {log[j]['T']:.0f} s > B = {B:g} s, so it is split in two")
            mark.set_color(CRITICAL)
        elif over:
            status.set_text(f"{len(groups)} sink{'s' if len(groups) > 1 else ''}: {over} tour{'s' if over > 1 else ''} "
                            f"still longer than B = {B:g} s")
            mark.set_color(CRITICAL)
        else:
            status.set_text(f"{len(groups)} sinks: every tour fits within B = {B:g} s (longest {max(Ts):.1f} s)")
            mark.set_color(GOOD)
        for tile, v in zip(tiles, (str(len(groups)), f"{max(Ts):.1f} s", f"{sum(log[j]['L'] for j in groups):.0f} m",
                                   str(over))):
            tile.set_text(v)
        return []

    _save(fig, update, len(frames), path, fps)
    return states


# ---------------------------------------------------------------------------
# 7. From the GA's straight lines to the path the sink drives
# ---------------------------------------------------------------------------

def _stops_in(path, stops, start=0):
    """Indices in the polyline `path` where it passes the stops, in order (base, sensors, base)."""
    return stop_indices(path, stops, start)


def _resample_legs(path, idx, m):
    """The polyline split at the given stop indices, each leg resampled to m points by arc length."""
    out = []
    for a, b in zip(idx[:-1], idx[1:]):
        leg = path[a:b + 1]
        if len(leg) < 2:
            leg = np.vstack([path[a], path[a]])
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(leg, axis=0).T))])
        u = np.linspace(0, s[-1], m) if s[-1] > 0 else np.zeros(m)
        out.append(np.column_stack([np.interp(u, s, leg[:, 0]), np.interp(u, s, leg[:, 1])]))
    return np.vstack(out)


def refine_stages(out, k):
    """The three versions of sink k's tour, each resampled leg by leg so that they can be morphed:
    straight lines (what the GA scores), waypoints around obstacles, and the Dubins path."""
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    tour = out["result"]["tours"][k]
    order = tour["order"]
    xy, base = scen["xy"], scen["base"]
    stops = np.vstack([base, xy[order], base])
    pts, _ = trajectory.waypoints(order, geo, p)
    dense = np.asarray(tour["path"])
    lengths = {"straight": ga_tour.euclid_length(order, xy, base),
               "waypoints": float(np.sum(np.hypot(*np.diff(pts, axis=0).T))), "dubins": tour["L"]}
    m = 40
    shapes = {"straight": _resample_legs(stops, list(range(len(stops))), m),
              "waypoints": _resample_legs(pts, _stops_in(pts, stops), m),
              "dubins": _resample_legs(dense, _stops_in(dense, stops), m)}
    return {"order": order, "stops": stops, "shapes": shapes, "lengths": lengths, "T": tour["T"],
            "dwell": float(np.sum(scen["tau"][order]))}


def pick_refine_sink(out):
    """The sink whose Dubins path is longest compared with its straight-line tour."""
    xy, base = out["scenario"]["xy"], out["scenario"]["base"]
    ratio = [t["L"] / max(ga_tour.euclid_length(t["order"], xy, base), 1e-9) for t in out["result"]["tours"]]
    return int(np.argmax(ratio))


def refine_animation(out, k, path, fps=12, dpi=80):
    """Sink k's tour in three stages, morphing from one to the next: the GA's straight lines, the
    waypoints around the obstacles, and the Dubins path. Bars show each stage's length."""
    p = out["params"]
    st = refine_stages(out, k)
    shapes, L = st["shapes"], st["lengths"]
    allpts = np.vstack(list(shapes.values()))
    lo, hi = allpts.min(0) - 8, allpts.max(0) + 8
    span = max(hi - lo)
    mid = (lo + hi) / 2
    lims = (mid - span / 2, mid + span / 2)
    names = [("straight", "Straight lines between sensors (what the GA scores)"),
             ("waypoints", "Waypoints around the obstacles (geodesic paths)"),
             ("dubins", "Dubins path with turning radius R (what the sink drives)")]
    colors = {"straight": QUIET, "waypoints": POP, "dubins": BEST}
    plan = []                                          # (shape a, shape b, fraction, stage reached)
    for i, (key, _) in enumerate(names):
        plan += [(key, key, 0.0, i)] * 14
        if i + 1 < len(names):
            plan += [(key, names[i + 1][0], t, i) for t in np.linspace(0, 1, 16)[1:-1]]
    plan += [("dubins", "dubins", 0.0, 2)] * 24
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    _header(fig, f"From the GA's straight lines to the path sink {k} drives",
            f"Sink {k} visits {len(st['order'])} sensors in the order the GA chose. The order stays the same; "
            f"only the way between the sensors changes.")
    ax = fig.add_axes([0.045, 0.05, 0.5, 0.8])
    bars = fig.add_axes([0.6, 0.45, 0.37, 0.34])
    stage_label = fig.text(0.015, 0.87, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    note = fig.text(0.6, 0.36, "", fontsize=9.5, color=INK, va="top", linespacing=1.6)

    def update(frame):
        a, b, t, reached = plan[min(frame, len(plan) - 1)]
        shape = (1 - t) * shapes[a] + t * shapes[b]
        _draw_map(ax, out, lims)
        ax.scatter(*out["scenario"]["xy"].T, s=8, color=PENDING, zorder=3)
        ax.plot(shape[:, 0], shape[:, 1], color=colors[b if t > 0.5 else a], lw=1.8, zorder=5)
        ax.scatter(*st["stops"][1:-1].T, s=22, color=INK, zorder=6)
        stage_label.set_text(names[reached][1] if t == 0 else "…")
        bars.cla()
        _quiet_axes(bars, "x")
        top = max(L.values()) * 1.25
        bars.set_xlim(0, top)
        bars.set_ylim(-0.6, 2.6)
        for i, (key, title) in enumerate(names):
            y = 2 - i
            if i <= reached:
                bars.barh([y], [L[key]], height=0.5, color=colors[key], zorder=3)
                bars.text(L[key] + 0.01 * top, y, f"{L[key]:.0f} m", va="center", fontsize=9, color=INK)
            bars.text(0, y + 0.36, title.split(" (")[0], fontsize=8.5, color=INK, va="bottom")
        bars.set_yticks([])
        bars.set_xlabel("Length of the tour [m]", fontsize=8.5, color=MUTED)
        if reached == 2:
            note.set_text(f"Mission time T = L / v + Σ τ = {L['dubins']:.0f} / {p['v']:g} + {st['dwell']:.1f}"
                          f" = {st['T']:.1f} s\n(budget B = {p['B']:g} s, eqs. 17-18)\n"
                          f"The Dubins path is {100 * (L['dubins'] / L['straight'] - 1):.0f} % longer than\n"
                          f"the straight lines the GA scored.")
        else:
            note.set_text("")
        return []

    _save(fig, update, len(plan), path, fps)
    return st


# ---------------------------------------------------------------------------
# 8. The whole pipeline
# ---------------------------------------------------------------------------

def _tiles(fig, x, y, names=("Sinks K", "Longest T_max", "Total L_total", "In conflict")):
    """Four score tiles with the given names (by default K, T_max, L_total and the time in conflict)."""
    values = []
    for i, name in enumerate(names):
        xi = x + 0.105 * i
        fig.text(xi, y, name, fontsize=8, color=QUIET, va="bottom")
        values.append(fig.text(xi, y - 0.01, "", fontsize=13, fontweight="bold", color=INK, va="top"))
    return values


def pipeline_animation(out, log, path, fps=8, dpi=72, step=2, ga_every=5):
    """The algorithm in five chapters: the GA orders all sensors; groups are split until every tour
    fits B; the final Dubins routes; the sinks moving as planned (conflicts in red); and after the
    conflict game. Score tiles show K, T_max, L_total and the time in conflict as they improve."""
    scen, res, p = out["scenario"], out["result"], out["params"]
    xy, base = scen["xy"], scen["base"]
    states = split_states(log)
    before = {k: t["frames"] for k, t in enumerate(res["tours"])}
    after = res["frames"]
    tb, ta = conflict_timeline(before, p), conflict_timeline(after, p)
    lims = _limits(out, [r["path"] for r in log] + list(after.values()))
    first = log[0]
    gens = first["generations"]
    s0 = ga_summary(gens, p["R"])
    kept_runs = [j for j, r in enumerate(log) if r["kept"]]
    planned_T = [t["T"] for t in res["tours"]]
    planned_L = [t["L"] for t in res["tours"]]

    chapters = []
    for g in list(range(0, len(gens), ga_every)) + [len(gens) - 1] * 6:
        chapters.append(("ga", g))
    for j in range(len(log)):
        chapters += [("split", (j, False)), ("split", (j, True))]
    chapters += [("routes", None)] * 16
    n_move_b, n_move_a = len(tb["cum"]), len(ta["cum"])
    chapters += [("before", i) for i in range(0, n_move_b, step)] + [("before", n_move_b - 1)] * 8
    chapters += [("after", i) for i in range(0, n_move_a, step)] + [("after", n_move_a - 1)] * 24

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
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    title = fig.text(0.015, 0.975, "", fontsize=12, fontweight="bold", color=INK, va="top")
    subtitle = fig.text(0.015, 0.932, "", fontsize=9, color=QUIET, va="top")
    ax = fig.add_axes([0.045, 0.04, 0.48, 0.82])
    chart = fig.add_axes([0.6, 0.4, 0.37, 0.4])
    mark, status = _status_line(fig, 0.6, 0.85, size=9.5)
    tiles = _tiles(fig, 0.56, 0.24)
    fig.text(0.56, 0.08, f"{len(xy)} sensors, budget B = {p['B']:g} s, d_safe = {p['d_safe']:g} m.",
             fontsize=8, color=QUIET, va="bottom")

    def set_tiles(K, T, L, conflict):
        for tile, v in zip(tiles, (K, T, L, conflict)):
            tile.set_text(v)

    def update(frame):
        kind, arg = chapters[min(frame, len(chapters) - 1)]
        title.set_text(titles[kind][0])
        subtitle.set_text(titles[kind][1])
        _draw_map(ax, out, lims)
        chart.cla()
        _quiet_axes(chart, "y")
        mark.set_color(INK)
        if kind == "ga":
            order = gens[arg]["orders"][0]
            pts = np.vstack([base, xy[order], base])
            ax.scatter(*xy.T, s=9, color=INK, zorder=4)
            ax.plot(pts[:, 0], pts[:, 1], color=BEST, lw=0.7, zorder=3)
            x = np.arange(arg + 1)
            chart.plot(np.append(x, arg + 1), np.append(s0["best"][:arg + 1], s0["best"][arg]), color=BEST, lw=2,
                       drawstyle="steps-post")
            chart.set_xlim(0, len(gens))
            chart.set_ylim(s0["best"].min() * 0.98, s0["best"].max() * 1.02)
            chart.set_xlabel("Generation", fontsize=8.5, color=MUTED)
            chart.set_ylabel("Fitness of the best tour [m]", fontsize=8.5, color=MUTED)
            status.set_text(f"Generation {arg}: fitness {s0['best'][arg]:.0f} m")
            set_tiles("1", "—", "—", "—")
        elif kind == "split":
            j, decided = arg
            _draw_split(ax, out, log, states, j, decided)
            _split_chart(chart, log, j, p["B"], decided)
            if decided:
                text, color = _split_text(log[j], j, len(states[j]["kept"]), p["B"])
                mark.set_color(color)
            else:
                text = f"GA run {j}: planning a group of {len(log[j]['ids'])} sensors"
            status.set_text(text.replace("  →  ", "\n→ "))
            done = states[j]["kept"] + ([j] if decided and log[j]["kept"] else [])
            ks = [kept_runs.index(r) for r in done]
            # sinks the plan needs so far: kept groups, waiting groups, and the current group (or its two halves)
            groups = len(states[j]["kept"]) + len(states[j]["waiting"]) + \
                (len(log[j]["halves"]) if decided and not log[j]["kept"] else 1)
            set_tiles(str(groups), f"{max(planned_T[k] for k in ks):.1f} s" if ks else "—",
                      f"{sum(planned_L[k] for k in ks):.0f} m" if ks else "—", "—")
        elif kind == "routes":
            for k, t in enumerate(res["tours"]):
                ax.plot(*np.asarray(t["path"]).T, color=_style(k)[0], lw=1.0, zorder=3)
                ax.scatter(*xy[t["order"]].T, s=12, color=_style(k)[0], zorder=5)
            chart.bar(np.arange(len(planned_T)), planned_T, width=0.6, color=BEST, zorder=3)
            chart.axhline(p["B"], color=INK, lw=1, ls=DASHES)
            chart.text(len(planned_T) - 0.5, p["B"] * 1.02, f"B = {p['B']:g} s", fontsize=8, color=INK, ha="right",
                       va="bottom")
            chart.set_ylim(0, p["B"] * 1.2)
            chart.set_xlabel("Sink", fontsize=8.5, color=MUTED)
            chart.set_ylabel("Tour time T [s]", fontsize=8.5, color=MUTED)
            mark.set_color(GOOD)
            status.set_text(f"All {len(planned_T)} tours within B: longest {max(planned_T):.1f} s")
            set_tiles(str(len(before)), f"{max(planned_T):.1f} s", f"{sum(planned_L):.0f} m", "—")
        else:
            trajs, tl = (before, tb) if kind == "before" else (after, ta)
            _docking_zone(ax, out)
            _draw_moving(ax, trajs, arg, tl["active"])
            time_b, time_a = p["dt"] * np.arange(len(tb["cum"])), p["dt"] * np.arange(len(ta["cum"]))
            if kind == "before":
                chart.plot(time_b[:arg + 1], tb["cum"][:arg + 1], color=POP, lw=2, ls=DASHES, label="planned")
            else:
                chart.plot(time_b, tb["cum"], color=POP, lw=2, ls=DASHES, label="planned")
                chart.plot(time_a[:arg + 1], ta["cum"][:arg + 1], color=BEST, lw=2, label="after the game")
            chart.set_xlim(0, max(time_b[-1], time_a[-1]))
            chart.set_ylim(0, max(tb["cum"][-1], 1) * 1.15)
            chart.set_xlabel("t [s]", fontsize=8.5, color=MUTED)
            chart.set_ylabel("Time in conflict so far [s]", fontsize=8.5, color=MUTED)
            chart.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK)
            spent = tl["cum"][arg]
            now = len(tl["active"][arg])
            mark.set_color(CRITICAL if now else (GOOD if kind == "after" else INK))
            status.set_text(f"t = {p['dt'] * arg:.1f} s: {now} pair{'s' if now != 1 else ''} in conflict now")
            if kind == "before":
                set_tiles(str(len(before)), f"{max(planned_T):.1f} s", f"{sum(planned_L):.0f} m", f"{spent:.1f} s")
            else:
                set_tiles(str(len(after)), f"{max(res['T'].values()):.1f} s", f"{sum(res['L'].values()):.0f} m",
                          f"{spent:.1f} s")
        return []

    _save(fig, update, len(chapters), path, fps)
    return len(chapters)


# ---------------------------------------------------------------------------
# 9. Replay of one conflict game, switch by switch
# ---------------------------------------------------------------------------

def game_replay_data(out):
    """Round 0 of Steps 9-10 again, on the planned tours, with every game's strategies recorded.
    conflict_game.resolve is deterministic, so these are the planner's own games of that round."""
    res, p = out["result"], out["params"]
    tours = res["tours"]
    trace = []
    _, _, _, games = cg.resolve({k: t["frames"] for k, t in enumerate(tours)}, {k: t["T"] for k, t in enumerate(tours)},
                                {k: t["L"] for k, t in enumerate(tours)}, out["geo"], p, trace)
    return games, trace


def pick_game(games):
    """The game with the most best-response switches (then the most players)."""
    return max(range(len(games)), key=lambda i: (len(games[i]["moves"]), len(games[i]["players"])))


def game_animation(out, path, game=None, fps=10, dpi=80):
    """One conflict game (Section 0.10.1), switch by switch. For every strategy profile, from all-GO to
    the equilibrium: a short clip of the sinks near the conflict (red links: closer than d_safe), a
    timeline per player of when it is stopped and when it is in conflict, and the potential Phi, which
    falls by exactly the switching player's gain at every switch (an exact potential game)."""
    from matplotlib.patches import Rectangle
    p = out["params"]
    games, trace = game_replay_data(out)
    gi = pick_game(games) if game is None else game
    g, tr = games[gi], trace[gi]
    players, acts, before, s = g["players"], tr["acts"], tr["trajs"], g["decision_frame"]
    beta, gamma, delta = p["weights"]
    names = {k: [a["name"] for a in acts[k]] for k in players}
    profiles, pr = [], {k: 0 for k in players}
    profiles.append(dict(pr))
    for k, name, _ in g["moves"]:
        pr[k] = names[k].index(name)
        profiles.append(dict(pr))
    fixed = [m for m in before if m not in players]

    def trajs_of(prof):
        t = dict(before)
        t.update({k: acts[k][prof[k]]["traj"] for k in players})
        return t

    def own_cost(prof):
        return sum(beta * a["dT"] + gamma * a["dD"] / p["v"] + delta * a["W"] for a in (acts[k][prof[k]] for k in players))

    def risk(prof):
        t = trajs_of(prof)
        shared = sum(cg.exposure(t[k], t[j], s, p) for i, k in enumerate(players) for j in players[i + 1:])
        return shared + sum(cg.exposure(t[k], t[m], s, p) for k in players for m in fixed)

    info = []
    for i, prof in enumerate(profiles):
        rk, ck = risk(prof), own_cost(prof)
        phi = g["alpha"] * rk + ck
        assert abs(phi - g["potential"][i]) <= 1e-6 * max(1.0, abs(phi)), "potential does not match the game record"
        info.append({"trajs": trajs_of(prof), "risk": rk, "cost": ck, "phi": phi,
                     "timeline": conflict_timeline(trajs_of(prof), p)})
    # clip: from t = 0 to a little after the last conflict of the all-GO profile near the decision frame
    act0 = info[0]["timeline"]["active"]
    late = [n for n in range(s, min(len(act0), s + 60)) if any(k in pair for pair in act0[n] for k in players)]
    clip = max(late[-1] if late else s, s + 12) + 6                 # the timeline covers this
    show = min(clip, s + 24)                                         # the map plays about the first 12 s
    pts = np.vstack([np.vstack([f[:show + 1] for f in inf["trajs"].values()]) for inf in (info[0], info[-1])])
    lo, hi = pts.min(0) - 6, pts.max(0) + 6
    span = max(hi - lo)
    mid = (lo + hi) / 2
    lims = (mid - span / 2, mid + span / 2)

    def stopped(P, n0, n1):
        """Frames n in [n0, n1) at which the sink does not move between n and n + 1."""
        m = min(n1, len(P) - 1)
        return [n for n in range(n0, m) if np.hypot(*(P[n + 1] - P[n])) < 1e-9] + list(range(max(m, n0), n1))

    fig = plt.figure(figsize=(11, 6.8), dpi=dpi, facecolor=SURFACE)
    _header(fig, f"The conflict game, switch by switch ({len(players)} players, {len(g['moves'])} switches)",
            f"Game {gi + 1} of {len(games)}: decided at t = {s * p['dt']:.1f} s for a conflict at t = {tr['f'] * p['dt']:.1f} s. "
            f"Every player starts with GO; at each switch the player that gains most changes its strategy.")
    mark, caption = _status_line(fig, 0.015, 0.845, size=9.5)
    ax = fig.add_axes([0.03, 0.05, 0.42, 0.74])
    rows = fig.add_axes([0.58, 0.39, 0.39, 0.42])
    phi_ax = fig.add_axes([0.58, 0.07, 0.2, 0.2])
    tiles = []
    for i, name in enumerate(("Risk", "Cost", "Φ")):
        fig.text(0.81 + 0.065 * i, 0.235, name, fontsize=7.5, color=QUIET, va="bottom")
        tiles.append(fig.text(0.81 + 0.065 * i, 0.225, "", fontsize=11, fontweight="bold", color=INK, va="top"))
    fig.text(0.81, 0.12, "Φ = α · risk + cost of moves (seconds);\nrisk: seconds closer than d_safe.",
             fontsize=7.5, color=QUIET, va="top")
    frames = []
    for i in range(len(profiles)):
        frames += [(i, n) for n in range(0, show + 1)] + [(i, show)] * 5
    frames += [(len(profiles) - 1, show)] * 14
    wait_frames = {k: {i_: set(range(s, s + int(round(acts[k][prof[k]]["W"] / p["dt"]))))
                       for i_, prof in enumerate(profiles)} for k in players}

    def update(frame):
        i, n = frames[min(frame, len(frames) - 1)]
        inf, prof = info[i], profiles[i]
        trajs, tl = inf["trajs"], inf["timeline"]
        _draw_map(ax, out, lims)
        _docking_zone(ax, out)
        for k, f in trajs.items():
            color = _style(k)[0] if k in players else PENDING
            ax.plot(f[:show + 1, 0], f[:show + 1, 1], color=color, lw=0.8, alpha=0.4, zorder=2)
        for a, b in (tl["active"][n] if n < len(tl["active"]) else []):
            pa, pb = _position(trajs[a], n), _position(trajs[b], n)
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=CRITICAL, lw=2.4, zorder=6)
        for k, f in trajs.items():
            x, y = _position(f, n)
            ax.scatter([x], [y], s=46 if k in players else 22, color=_style(k)[0] if k in players else PENDING,
                       edgecolor=INK, lw=0.6, zorder=8)
            if k in players:
                ax.text(x + 0.6, y + 0.6, str(k), fontsize=7, color=INK, zorder=9)
        ax.text(0.02, 0.97, f"t = {n * p['dt']:.1f} s", transform=ax.transAxes, fontsize=10, fontweight="bold",
                color=INK, va="top")
        # timeline per player: stopped (grey) and in conflict (red), with a cursor at the current time
        rows.cla()
        _quiet_axes(rows, None)
        rows.set_xlim(0, clip * p["dt"])
        rows.set_ylim(-0.6, len(players) - 0.4)
        for r_i, k in enumerate(players):
            y = len(players) - 1 - r_i
            rows.add_patch(Rectangle((0, y - 0.08), clip * p["dt"], 0.16, color=GRID_LIGHT, lw=0, zorder=1))
            waits = wait_frames[k][i]
            for m in stopped(trajs[k], 0, clip):
                rows.add_patch(Rectangle((m * p["dt"], y - 0.22), p["dt"], 0.44,
                                         color=INK if m in waits else PENDING, lw=0, zorder=2))
            bad = set()
            for t_ in range(min(clip, len(tl["active"]))):
                if any(k in pair for pair in tl["active"][t_]):
                    bad.add(t_)
            for m in bad:
                rows.add_patch(Rectangle((m * p["dt"], y - 0.32), p["dt"], 0.18, color=CRITICAL, lw=0, zorder=3))
        rows.set_yticks(range(len(players)), [f"sink {k}  {names[k][prof[k]]}" for k in players][::-1], fontsize=7.5)
        rows.axvspan(0, show * p["dt"], color=GRID_LIGHT, alpha=0.6, lw=0, zorder=0)
        rows.axvline(n * p["dt"], color=INK, lw=1, zorder=4)
        rows.set_xlabel("t [s]  (shaded: the part the map plays)", fontsize=8, color=MUTED)
        rows.set_title("Players: WAIT (black), stopped at a sensor (grey), too close (red)", loc="left",
                       fontsize=8.5, color=INK, pad=4)
        # the potential, one point per profile
        phi_ax.cla()
        _quiet_axes(phi_ax, "y")
        phi_ax.set_yscale("log")
        phis = [x["phi"] for x in info]
        phi_ax.set_xlim(-0.5, len(info) - 0.5)
        phi_ax.set_ylim(min(phis) / 1.6, max(phis) * 1.6)
        phi_ax.plot(range(i + 1), phis[:i + 1], color=BEST, lw=2, drawstyle="steps-post", zorder=2)
        phi_ax.scatter(range(i + 1), phis[:i + 1], s=22, color=BEST, edgecolor=SURFACE, lw=1, zorder=3)
        phi_ax.set_xlabel("Switch", fontsize=8, color=MUTED)
        phi_ax.set_title("Potential Φ", loc="left", fontsize=8.5, color=INK, pad=4)
        phi_ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
        phi_ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
        phi_ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        tiles[0].set_text(f"{inf['risk']:.1f} s")
        tiles[1].set_text(f"{inf['cost']:.1f} s")
        tiles[2].set_text(f"{inf['phi']:.0f}")
        if i == 0:
            caption.set_text(f"Start: all {len(players)} players GO. Φ = {inf['phi']:.0f}.")
            mark.set_color(CRITICAL if inf["risk"] > 0 else GOOD)
        else:
            k, name, gain = g["moves"][i - 1]
            last = i == len(info) - 1
            was = names[k][profiles[i - 1][k]]
            caption.set_text(f"Switch {i}: sink {k} changes {was} → {name}; its own cost falls by {gain:.0f}, and "
                             f"Φ falls by exactly as much, to {inf['phi']:.0f}."
                             + ("\nNo player can lower its own cost by changing alone: a pure Nash equilibrium. "
                                f"The risk left is for later games." if last and inf["risk"] > 0 else
                                "\nNo player can lower its own cost by changing alone: a pure Nash equilibrium."
                                if last else ""))
            mark.set_color(GOOD if last else INK)
        return []

    _save(fig, update, len(frames), path, fps)
    return gi, g


GRID_LIGHT = "#ecebe6"


# ---------------------------------------------------------------------------
# 10. The six Dubins turn types as the end heading turns
# ---------------------------------------------------------------------------

def dubins_rotation_animation(path, R=5.0, end=(12.0, 5.0), step_deg=4, fps=8, dpi=80, hold=16):
    """For one leg from (0, 0) heading east to `end`, the end heading turns through 360 degrees. Every frame
    draws all six turn types with their lengths, the shortest (blue, solid) and the one the rule of Section
    0.13 picks (orange, dashed; if its type does not exist, the code falls back to the shortest)."""
    import dubins_path as dp
    from plot import FEASIBLE, SHORTEST
    x1, y1, psi1 = 0.0, 0.0, 0.0
    x2, y2 = end
    data = []
    for deg in np.arange(0, 360, step_deg):
        psi2 = np.radians(deg)
        parts = {t: tuple(float(v) for v in dp.calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in dp.TURN_TYPES}
        total = {t: sum(v) for t, v in parts.items()}
        best = min(total, key=total.get)
        rule = dp.findTurnType(x1, y1, psi1, x2, y2, psi2, R)
        pick = rule if np.isfinite(total[rule]) else best
        xy = {}
        for t, v in parts.items():
            if np.isfinite(total[t]):
                xs, ys, _, _ = dp.calculateTrajectory(x1, y1, psi1, R, t, *v, 0.1)
                xy[t] = np.column_stack([xs, ys])
        data.append({"deg": deg, "psi2": psi2, "total": total, "best": best, "rule": rule, "pick": pick, "xy": xy})
    allpts = np.vstack([a for d in data for a in d["xy"].values()])
    lo, hi = allpts.min(0) - 2, allpts.max(0) + 2
    longer = [d for d in data if d["pick"] != d["best"]]
    missing = [d for d in data if d["pick"] != d["rule"]]

    fig = plt.figure(figsize=(11, 6.8), dpi=dpi, facecolor=SURFACE)
    _header(fig, "The six Dubins turn types as the end heading turns",
            f"From (0, 0) heading east to ({x2:g}, {y2:g}), R = {R:g} m. Blue: the shortest of the six. Orange, dashed: "
            f"the type the rule of Section 0.13 picks (from the signs of cp1, cp2 and S_tan).")
    caption = fig.text(0.015, 0.855, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    big = fig.add_axes([0.02, 0.06, 0.44, 0.76])
    thumbs = [fig.add_axes([0.5 + 0.165 * (i % 3), 0.58 - 0.225 * (i // 3), 0.15, 0.16]) for i in range(6)]
    chart = fig.add_axes([0.54, 0.07, 0.43, 0.2])
    frames = list(range(len(data))) + [len(data) - 1] * hold

    def arrow(a, x, y, psi, color=INK, length=2.6):
        a.annotate("", xy=(x + length * np.cos(psi), y + length * np.sin(psi)), xytext=(x, y),
                   arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2, mutation_scale=9, shrinkA=0, shrinkB=0))

    def update(frame):
        i = frames[min(frame, len(frames) - 1)]
        d = data[i]
        big.cla()
        big.set_aspect("equal")
        big.axis("off")
        big.set_xlim(lo[0], hi[0])
        big.set_ylim(lo[1], hi[1])
        big.plot(*d["xy"][d["best"]].T, color=SHORTEST, lw=2.6, zorder=3)
        if d["pick"] != d["best"]:
            big.plot(*d["xy"][d["pick"]].T, color=FEASIBLE, lw=2.2, ls=DASHES, zorder=4)
        for (x, y, psi) in ((x1, y1, psi1), (x2, y2, d["psi2"])):
            big.scatter([x], [y], s=30, color=INK, zorder=5)
            arrow(big, x, y, psi)
        big.text(x1 - 0.5, y1 - 1.2, "start", fontsize=8, color=INK, ha="right", va="top")
        big.text(x2 + 0.6, y2 + 0.6, f"end, ψ₂ = {d['deg']:.0f}°", fontsize=8, color=INK)
        for t, a in zip(dp.TURN_TYPES, thumbs):
            a.cla()
            a.set_aspect("equal")
            a.set_xticks([])
            a.set_yticks([])
            a.set_xlim(lo[0], hi[0])
            a.set_ylim(lo[1], hi[1])
            for side in a.spines.values():
                side.set_visible(True)
                side.set_color(GRID)
                side.set_linewidth(0.8)
                side.set_linestyle("-")
            tags = []
            if t == d["best"]:
                tags.append("shortest")
                for side in a.spines.values():
                    side.set_color(SHORTEST)
                    side.set_linewidth(2.4)
            if t == d["rule"]:
                tags.append("rule's pick" if np.isfinite(d["total"][t]) else "rule's pick, missing")
                if t != d["best"]:
                    for side in a.spines.values():
                        side.set_color(FEASIBLE)
                        side.set_linewidth(2.2)
                        side.set_linestyle((0, (3, 2)))
            if t in d["xy"]:
                a.plot(*d["xy"][t].T, color=INK if t not in (d["best"], d["pick"]) else
                       (SHORTEST if t == d["best"] else FEASIBLE), lw=1.2)
                title = f"{t}  {d['total'][t]:.1f} m"
            else:
                title = f"{t}  does not exist"
            a.set_title(title + ("\n" + ", ".join(tags) if tags else "\n "), fontsize=7.5, color=INK, pad=2)
        chart.cla()
        _quiet_axes(chart, "y")
        degs = [x["deg"] for x in data[:i + 1]]
        chart.plot(degs, [x["total"][x["best"]] for x in data[:i + 1]], color=SHORTEST, lw=2, label="shortest")
        chart.plot(degs, [x["total"][x["pick"]] for x in data[:i + 1]], color=FEASIBLE, lw=2, ls=DASHES,
                   label="rule of Section 0.13")
        chart.set_xlim(0, 360)
        chart.set_ylim(0, max(x["total"][x["pick"]] for x in data) * 1.08)
        chart.set_xticks(range(0, 361, 45))
        chart.set_xlabel("End heading ψ₂ [degrees]", fontsize=8, color=MUTED)
        chart.set_ylabel("Length [m]", fontsize=8, color=MUTED)
        chart.legend(loc="upper left", frameon=False, fontsize=8, labelcolor=INK, ncol=2)
        if frame >= len(data):
            caption.set_text(f"Over all {len(data)} headings: the rule picks a longer path at {len(longer)} of them "
                             f"(up to +{100 * max((x['total'][x['pick']] / x['total'][x['best']] - 1) for x in longer):.0f} %),"
                             f"\nand at {len(missing)} its type does not exist, so the code uses the shortest instead.")
        elif d["pick"] != d["best"]:
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the shortest is {d['best']} ({d['total'][d['best']]:.1f} m); "
                             f"the rule picks {d['pick']} ({d['total'][d['pick']]:.1f} m, "
                             f"+{100 * (d['total'][d['pick']] / d['total'][d['best']] - 1):.0f} %).")
        elif d["pick"] != d["rule"]:
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the rule picks {d['rule']}, which does not exist here, "
                             f"so the code uses the shortest, {d['best']} ({d['total'][d['best']]:.1f} m).")
        else:
            caption.set_text(f"ψ₂ = {d['deg']:.0f}°: the rule picks {d['best']}, the shortest "
                             f"({d['total'][d['best']]:.1f} m).")
        return []

    _save(fig, update, len(frames), path, fps)
    return {"headings": len(data), "longer": len(longer), "missing": len(missing)}


# ---------------------------------------------------------------------------
# 11. The geodesic distance spreading from the base (Dijkstra)
# ---------------------------------------------------------------------------

def geodesic_wave_animation(out, path, steps=70, fps=10, dpi=80, hold=20):
    """The shortest-path tree of geodesic.build from the base, grown by distance: at every frame each tree
    edge is drawn as far as the geodesic distance rho has reached, and each sensor lights up when the wave
    gets there (its r of eq. 9). The dashed circle is the straight-line distance rho: where an obstacle is in
    the way, the wave lags behind it. Beside the map, r against the straight-line distance of every sensor
    reached so far."""
    import geodesic
    scen, geo, p = out["scenario"], out["geo"], out["params"]
    nodes, n_t, pred = geo["nodes"], geo["n_terminals"], geo["pred"][0]
    base = scen["base"]
    dist = np.full(len(nodes), np.inf)
    dist[0] = 0.0
    for v in range(len(nodes)):                             # distance along the tree, by walking up to the base
        chain, u = [], v
        while not np.isfinite(dist[u]) and pred[u] >= 0:
            chain.append(u)
            u = pred[u]
        if np.isfinite(dist[u]):
            for w in reversed(chain):
                dist[w] = dist[pred[w]] + float(np.hypot(*(nodes[w] - nodes[pred[w]])))
    r = dist[1:n_t]
    assert np.allclose(r[np.isfinite(r)], geo["D"][0, 1:][np.isfinite(r)]), "the tree does not give D"
    euclid = np.hypot(*(scen["xy"] - base).T)
    grown = geodesic.inflate(scen["obstacles"], p["r_plan"])
    top = float(np.nanmax(np.where(np.isfinite(r), r, np.nan)))
    rhos = list(np.linspace(0, top * 1.02, steps)) + [top * 1.02] * hold
    lims = _limits(out, [nodes[np.isfinite(dist)]])
    detour = np.where(np.isfinite(r), r / np.maximum(euclid, 1e-9) - 1, np.nan)

    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    _header(fig, "The geodesic distance r spreading from the base (eq. 9, Dijkstra on the visibility graph)",
            "Blue: shortest paths from the base, drawn as far as the distance ρ has reached. They bend only at corners "
            "of the obstacles grown by 7 m (thin outlines) and at sensors.")
    caption = fig.text(0.015, 0.865, "", fontsize=10, fontweight="bold", color=INK, va="bottom")
    ax = fig.add_axes([0.045, 0.04, 0.5, 0.8])
    chart = fig.add_axes([0.62, 0.36, 0.35, 0.44])
    tiles = _tiles(fig, 0.6, 0.2, ("Distance ρ", "Sensors reached", "Largest detour", ""))

    def update(frame):
        rho = rhos[min(frame, len(rhos) - 1)]
        _draw_map(ax, out, lims)
        for g_ in grown:
            gx, gy = g_.exterior.xy
            ax.plot(gx, gy, color=QUIET, lw=0.5, alpha=0.5, zorder=2)
        t = np.linspace(0, 2 * np.pi, 200)
        ax.plot(base[0] + rho * np.cos(t), base[1] + rho * np.sin(t), color=MUTED, lw=1.0, ls=DASHES, zorder=3)
        for v in range(1, len(nodes)):
            u = pred[v]
            if u < 0 or not np.isfinite(dist[v]) or rho <= dist[u]:
                continue
            frac = min(1.0, (rho - dist[u]) / max(dist[v] - dist[u], 1e-9))
            end_ = nodes[u] + frac * (nodes[v] - nodes[u])
            ax.plot([nodes[u][0], end_[0]], [nodes[u][1], end_[1]], color=BEST, lw=1.2, zorder=4)
        reached = np.isfinite(r) & (r <= rho)
        around = reached & (detour > 0.02)
        ax.scatter(*scen["xy"][~reached].T, s=10, color=PENDING, zorder=5)
        ax.scatter(*scen["xy"][reached].T, s=16, color=INK, zorder=6)
        ax.scatter(*scen["xy"][around].T, s=90, facecolor="none", edgecolor=POP, lw=1.6, zorder=7)
        corners = np.arange(n_t, len(nodes))
        hit = corners[dist[corners] <= rho]
        ax.scatter(*nodes[hit].T, s=6, color=BEST, zorder=5)
        chart.cla()
        _quiet_axes(chart, "y")
        lim = max(top, euclid.max()) * 1.05
        chart.set_xlim(0, lim)
        chart.set_ylim(0, lim)
        chart.plot([0, lim], [0, lim], color=MUTED, lw=1, ls=DASHES)
        chart.text(lim * 0.97, lim * 0.9, "r = straight-line distance", fontsize=7.5, color=MUTED, ha="right")
        chart.scatter(euclid[reached & ~around], r[reached & ~around], s=16, color=BEST, edgecolor=SURFACE, lw=0.6,
                      zorder=3, label="straight line is clear")
        chart.scatter(euclid[around], r[around], s=22, color=POP, edgecolor=SURFACE, lw=0.6, zorder=4,
                      label="goes round an obstacle (r > 2 % longer)")
        chart.legend(loc="upper left", frameon=False, fontsize=7.5, labelcolor=INK)
        chart.set_xlabel("Straight-line distance from the base [m]", fontsize=8, color=MUTED)
        chart.set_ylabel("Geodesic distance r [m]", fontsize=8, color=MUTED)
        chart.set_title("Each sensor reached: r against straight-line distance", loc="left",
                        fontsize=8.5, color=INK, pad=4)
        caption.set_text(f"ρ = {rho:.0f} m: the wave has reached {int(reached.sum())} of {len(r)} sensors; "
                         f"{int(around.sum())} of them (orange rings) had to go round an obstacle.")
        tiles[0].set_text(f"{rho:.0f} m")
        tiles[1].set_text(f"{int(reached.sum())}")
        tiles[2].set_text(f"+{100 * np.nanmax(detour[reached]):.0f} %" if reached.any() else "—")
        return []

    _save(fig, update, len(rhos), path, fps)
    return {"sensors": len(r), "largest_detour": float(np.nanmax(detour))}
