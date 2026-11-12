"""Animations of the whole algorithm (pipeline_gifs.py). Each one shows something getting better:

    conflict_animation   the sinks moving as planned and after the conflict game, side by side (Steps 8-10)
    splitting_animation  groups split until every tour fits the budget B (Steps 1-7, Section 0.9)
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
from plot import BEST, DASHES, INK, MUTED, POP, QUIET, SURFACE, _map, _quiet_axes, _style, ga_summary, save_gif

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
# 7. From the GA's straight lines to the path the sink drives
# ---------------------------------------------------------------------------

def _stops_in(path, stops, start=0):
    """Indices in the polyline `path` where it passes the stops, in order (base, sensors, base)."""
    idx, i = [], start
    for s in stops:
        d = np.hypot(*(path[i:] - s).T)
        hit = np.flatnonzero(d < 1e-6)
        i = i + (int(hit[0]) if len(hit) else int(np.argmin(d)))
        idx.append(i)
    return idx


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

def _tiles(fig, x, y):
    """Four score tiles: K, T_max, L_total and the time in conflict."""
    names = ["Sinks K", "Longest T_max", "Total L_total", "In conflict"]
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
