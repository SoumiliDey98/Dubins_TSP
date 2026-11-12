"""Figures: the planned routes (PNG), an optional animation (GIF), and the comparison of the two Dubins methods."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
from matplotlib.patches import Patch, PathPatch
from matplotlib.patches import Polygon as PolygonPatch
from matplotlib.path import Path
from shapely.geometry import Polygon

# Eight distinguishable colours; more sinks reuse them with another line style.
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
STYLES = ["-", "--", ":"]
INK, MUTED = "#14171c", "#6b6f76"


def _style(k):
    return COLORS[k % len(COLORS)], STYLES[(k // len(COLORS)) % len(STYLES)]


def _map(ax, out):
    scen, p = out["scenario"], out["params"]
    for o in scen["obstacles"]:
        ax.add_patch(PolygonPatch(o, closed=True, color="#9a9ea6", alpha=0.8, lw=0))
        x, y = Polygon(o).buffer(p["r_safe"]).exterior.xy
        ax.plot(x, y, color=MUTED, lw=0.6, ls="--")
    ax.plot(*scen["base"], marker="^", ms=11, color=INK, zorder=10)
    ax.set_aspect("equal")
    ax.grid(color="#e4e3df", lw=0.5)
    ax.set_axisbelow(True)


def _sensors(ax, out, marker="o", size=14):
    """The static sensors, each in the colour of the sink that serves it."""
    scen = out["scenario"]
    for k, tour in enumerate(out["result"]["tours"]):
        ax.scatter(*scen["xy"][tour["order"]].T, s=size, marker=marker, color=_style(k)[0], zorder=5)


def routes(out, path):
    """Save a PNG with obstacles, the r_safe boundary, every sink's trajectory and its sensors."""
    scen, res, c = out["scenario"], out["result"], out["checks"]
    fig, ax = plt.subplots(figsize=(9, 8))
    _map(ax, out)
    for k, tour in enumerate(res["tours"]):
        color, ls = _style(k)
        f = res["frames"][k]
        ax.plot(f[:, 0], f[:, 1], color=color, ls=ls, lw=1.4, label=f"MS {k}")
        far =scen["xy"][tour["order"]][np.argmax(np.hypot(*(scen["xy"][tour["order"]] - scen["base"]).T))]
        ax.annotate(str(k), far, xytext=(4, 4), textcoords="offset points", fontsize=8, color=INK)
    _sensors(ax, out)
    ax.set_title(f"K = {c['K']} sinks, T_max = {c['T_max']:.1f} s, L_total = {c['L_total']:.0f} m, "
                 f"remaining conflicts = {c['conflict_pairs']}", fontsize=10, color=INK)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def animation(out, path, fps=12):
    """Save a GIF of all sinks moving (one image per frame of dt seconds)."""
    import matplotlib.animation as anim
    res, p = out["result"], out["params"]
    frames = res["frames"]
    n = max(len(f) for f in frames.values())
    fig, ax = plt.subplots(figsize=(7, 7))
    _map(ax, out)
    for k, f in frames.items():
        ax.plot(f[:, 0], f[:, 1], color=_style(k)[0], lw=0.6, alpha=0.4)
    _sensors(ax, out, marker="s", size=18)
    ax.legend(handles=[plt.Line2D([], [], ls="", marker="s", ms=5, color=MUTED, label="sensor (colour of its sink)"),
                       plt.Line2D([], [], ls="", marker="o", ms=7, color=MUTED, mec=INK, mew=0.5, label="mobile sink"),
                       plt.Line2D([], [], ls="", marker="^", ms=8, color=INK, label="base station")],
              loc="upper right", fontsize=7, framealpha=0.9)
    dots = {k: ax.plot([], [], marker="o", ms=7, color=_style(k)[0], mec=INK, mew=0.5, zorder=8)[0] for k in frames}
    label = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=9, va="top")

    def update(i):
        for k, f in frames.items():
            x, y = f[min(i, len(f) - 1)]
            dots[k].set_data([x], [y])
        label.set_text(f"t = {i * p['dt']:.1f} s")
        return list(dots.values()) + [label]

    ani = anim.FuncAnimation(fig, update, frames=n, blit=True)
    ani.save(path, writer="pillow", fps=fps)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Comparison of the two Dubins methods (compare_dubins.py --figures)
# ---------------------------------------------------------------------------

SHORTEST, FEASIBLE = COLORS[0], COLORS[1]     # each method keeps its colour in every figure
SURFACE, GRID, BASELINE, QUIET = "#fcfcfb", "#e1e0d9", "#c3c2b7", "#52514e"
KIND_COLOR = {"shortest": BASELINE, "longer": FEASIBLE, "missing": QUIET}
KIND_LABEL = {"shortest": "Rule's type is the shortest", "longer": "Rule's type exists but is longer", "missing": "Rule's type does not exist (fallback)"}
KIND_INK = {"shortest": INK, "longer": "#ffffff", "missing": "#ffffff"}
DASHES = (0, (3.5, 2.5))                      # feasible paths are dashed too, so they do not rely on colour alone


def _start(size, title, subtitle, left=0.1, right=0.97, bottom=0.12, top=0.8, **grid):
    """A figure with its title and subtitle. The layout is fixed here, because _bar measures pixels."""
    fig, axes = plt.subplots(figsize=size, dpi=150, facecolor=SURFACE, **grid)
    fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top)
    fig.text(0.02, 1 - 0.14 / size[1], title, fontsize=12, fontweight="bold", color=INK, va="top")
    fig.text(0.02, 1 - 0.40 / size[1], subtitle, fontsize=9, color=QUIET, va="top")
    return fig, axes


def _quiet_axes(ax, grid_axis=None):
    """Recessive axes: no box, hairline grid, muted ticks."""
    ax.set_facecolor(SURFACE)
    for side in ax.spines:
        ax.spines[side].set_visible(False)
    ax.tick_params(length=0, labelsize=8.5, colors=MUTED)
    ax.set_axisbelow(True)
    if grid_axis:
        ax.grid(axis=grid_axis, color=GRID, lw=0.7)


def _bar(ax, x0, x1, y0, y1, color, round_end=None):
    """A bar given in data coordinates, rounded by about 4 px at round_end ("right" or "top") and square at its baseline."""
    to_data = ax.transData.inverted()
    (a0, b0), (a1, b1) = to_data.transform([(0, 0), (6, 6)])
    rx, ry = min(abs(a1 - a0), abs(x1 - x0) / 2), min(abs(b1 - b0), abs(y1 - y0) / 2)
    if round_end == "right":
        verts = [(x0, y0), (x1 - rx, y0), (x1, y0), (x1, y0 + ry), (x1, y1 - ry), (x1, y1), (x1 - rx, y1), (x0, y1), (x0, y0)]
        codes = [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY]
    elif round_end == "top":
        verts = [(x0, y0), (x1, y0), (x1, y1 - ry), (x1, y1), (x1 - rx, y1), (x0 + rx, y1), (x0, y1), (x0, y1 - ry), (x0, y0)]
        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CURVE3, Path.CURVE3, Path.CLOSEPOLY]
    else:
        verts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    ax.add_patch(PathPatch(Path(verts, codes), facecolor=color, lw=0, zorder=3))


def _method_legend(fig, y):
    fig.legend(handles=[Line2D([], [], color=SHORTEST, lw=2, label="Shortest (best of six)"),
                        Line2D([], [], color=FEASIBLE, lw=2, ls=DASHES, label="Feasible (rule of Section 0.13)")],
               loc="upper left", bbox_to_anchor=(0.012, y), ncol=2, frameon=False, fontsize=9, labelcolor=INK, handlelength=2.6)


def _kind_legend(fig, y):
    fig.legend(handles=[Patch(color=KIND_COLOR[k], label=KIND_LABEL[k]) for k in KIND_COLOR],
               loc="upper left", bbox_to_anchor=(0.012, y), ncol=3, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=1.2, columnspacing=1.4)


def compare_example_legs(legs, R, path):
    """Four legs, all at one scale: both paths between the same two poses."""
    fig, axes = _start((9, 7.6), "The rule often finds the shortest path, but not always",
                       f"Four example legs with R = {R:.0f} m, all at the same scale. Arrows show the start and end headings.",
                       left=0.03, right=0.97, bottom=0.03, top=0.825, nrows=2, ncols=2)
    fig.subplots_adjust(hspace=0.3, wspace=0.08)
    _method_legend(fig, 1 - 0.62 / 7.6)
    span = 1.08 * max(max(np.ptp(leg[m][a]) for m in ("shortest", "feasible") for a in "xy") for leg in legs)
    for ax, leg in zip(axes.flat, legs):
        ax.set_facecolor(SURFACE)
        ax.axis("off")
        ax.set_aspect("equal")
        xs, ys = np.concatenate([leg["shortest"]["x"], leg["feasible"]["x"]]), np.concatenate([leg["shortest"]["y"], leg["feasible"]["y"]])
        cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
        ax.set_xlim(cx - span / 2, cx + span / 2)
        ax.set_ylim(cy - span / 2, cy + span / 2)
        ax.plot(leg["shortest"]["x"], leg["shortest"]["y"], color=SHORTEST, lw=2, solid_capstyle="round", zorder=3)
        ax.plot(leg["feasible"]["x"], leg["feasible"]["y"], color=FEASIBLE, lw=2, ls=DASHES, zorder=4)
        for q, name in ((leg["q0"], "start"), (leg["q1"], "end")):
            tip = (q[0] + 0.9 * R * np.cos(q[2]), q[1] + 0.9 * R * np.sin(q[2]))
            ax.annotate("", xy=tip, xytext=q[:2], arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.1, mutation_scale=11), zorder=5)
            ax.plot(*q[:2], marker="o", ms=5, color=INK, mec=SURFACE, mew=1.2, zorder=6)
            spots = [(q[0] + dx, q[1] + dy) for dx in (-3.5, 3.5) for dy in (-2.2, 2.2)]     # the clearest spot beside the point
            label = max(spots, key=lambda c: np.min(np.hypot(xs - c[0], ys - c[1])))
            ax.text(*label, name, ha="center", va="center", fontsize=8, color=MUTED, zorder=6)
        a, b = leg["shortest"], leg["feasible"]
        extra = f"  (+{100 * (b['length'] / a['length'] - 1):.0f} %)" if b["length"] > a["length"] + 1e-6 else ""
        ax.set_title(f"{leg['title']}\n", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=4)
        ax.text(0, 1.012, f"Shortest {a['turnType']} {a['length']:.1f} m   |   Feasible {b['turnType']} {b['length']:.1f} m{extra}",
                transform=ax.transAxes, fontsize=8.5, color=QUIET, va="bottom")
        x0, y0 = cx - span / 2 + 0.02 * span, cy - span / 2 + 0.03 * span
        ax.plot([x0, x0 + 10], [y0, y0], color=MUTED, lw=1.2)
        ax.text(x0 + 5, y0 + 0.012 * span, "10 m", ha="center", va="bottom", fontsize=7.5, color=MUTED)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def compare_extra_length(rows, path):
    """Extra total length of the feasible plan on each map, largest first; maps where the number of sinks changed stand out."""
    short = {r["seed"]: r for r in rows if r["method"] == "shortest"}
    items = sorted(((100 * (r["L"] / short[r["seed"]]["L"] - 1), r["seed"], short[r["seed"]]["K"], r["K"]) for r in rows if r["method"] == "pdf"))
    total = 100 * (sum(r["L"] for r in rows if r["method"] == "pdf") / sum(r["L"] for r in short.values()) - 1)
    height = 1.45 + 0.34 * len(items)
    fig, ax = _start((7.5, height), f"Feasible plans are longer on every map: +{total:.1f} % in total",
                     "Extra total path length compared with the shortest-path plan, for each method's own routes",
                     left=0.1, right=0.97, bottom=0.42 / height, top=1 - 1.0 / height)
    _quiet_axes(ax, "x")
    fig.legend(handles=[Patch(color=FEASIBLE, label="Number of sinks changed"), Patch(color=BASELINE, label="Same number of sinks")],
               loc="upper left", bbox_to_anchor=(0.012, 1 - 0.62 / height), ncol=2, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=1.2)
    ax.set_xlim(0, max(i[0] for i in items) * 1.42)
    ax.set_ylim(-0.6, len(items) - 0.4)
    ax.set_yticks(range(len(items)), [f"Seed {i[1]}" for i in items], color=INK)
    ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
    for y, (extra, _, k0, k1) in enumerate(items):
        _bar(ax, 0, extra, y - 0.31, y + 0.31, FEASIBLE if k0 != k1 else BASELINE, "right")
        note = f"   {k0} \u2192 {k1} sinks" if k0 != k1 else ""
        ax.text(extra, y, f"  +{extra:.1f} %{note}", va="center", fontsize=8.5, color=INK, zorder=4)
    ax.axvline(0, color=BASELINE, lw=0.8, zorder=4)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def compare_same_tour(out, tour, path):
    """One tour of the default plan, on its map, with both methods using the same visiting order."""
    a, b = tour["shortest"], tour["feasible"]
    size = (7.5, 7.6)
    fig, ax = _start(size, "The same visiting order, drawn with both methods",
                     f"Seed {tour['seed']}, sink {tour['sink']}, the tour that changes most: shortest {a['L']:.0f} m, feasible {b['L']:.0f} m "
                     f"(+{100 * (b['L'] / a['L'] - 1):.0f} %)", left=0.07, right=0.97, bottom=0.05, top=0.87)
    _method_legend(fig, 1 - 0.62 / size[1])
    _map(ax, out)
    ax.set_facecolor(SURFACE)
    for side in ax.spines:
        ax.spines[side].set_visible(False)
    ax.tick_params(length=0, labelsize=8, colors=MUTED)
    ax.plot(a["path"][:, 0], a["path"][:, 1], color=SHORTEST, lw=2, solid_joinstyle="round", zorder=4)
    ax.plot(b["path"][:, 0], b["path"][:, 1], color=FEASIBLE, lw=2, ls=DASHES, zorder=5)
    ax.scatter(*out["scenario"]["xy"][a["order"]].T, s=36, color=INK, edgecolors=SURFACE, linewidths=1.2, zorder=6)
    both = np.vstack([a["path"], b["path"]])
    low, high = both.min(axis=0), both.max(axis=0)
    centre, half = (low + high) / 2, (high - low).max() / 2 + 6
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def _stack(ax, shares, y, thickness):
    """One horizontal stacked bar of the three kinds of leg, with 2 px gaps; a share is printed inside only if it fits."""
    a0, a1 = ax.transData.inverted().transform([(0, 0), (3, 3)])[:, 0]
    gap = abs(a1 - a0)
    pos = 0.0
    last = [k for k in KIND_COLOR if shares[k] > 0][-1]
    for k in KIND_COLOR:
        share = 100 * shares[k]
        if share <= 0:
            continue
        end = pos + share
        tip = end - (0 if k == last else gap)
        tip = max(tip, pos + gap / 2)
        _bar(ax, pos, tip, y - thickness / 2, y + thickness / 2, KIND_COLOR[k], "right" if k == last else None)
        if share >= 9:
            ax.text((pos + tip) / 2, y, f"{share:.1f} %", ha="center", va="center", fontsize=8.5, color=KIND_INK[k], zorder=4)
        pos = end


def compare_leg_types(rows, path):
    """How the rule's own turn type does on the legs of each final plan."""
    totals = {}
    for method in ("shortest", "pdf"):
        legs = [r["legs"] for r in rows if r["method"] == method]
        n = sum(sum(x.values()) for x in legs)
        totals[method] = ({k: sum(x[k] for x in legs) / n for k in KIND_COLOR}, n)
    size = (7.5, 3.2)
    fig, ax = _start(size, "The heading search steers feasible plans to poses that suit the rule",
                     f"Legs where the rule's type exists but is longer fall from {100 * totals['shortest'][0]['longer']:.1f} % to {100 * totals['pdf'][0]['longer']:.1f} %",
                     left=0.2, right=0.92, bottom=0.17, top=0.66)
    _quiet_axes(ax, "x")
    _kind_legend(fig, 1 - 0.62 / size[1])
    ax.set_xlim(0, 100)
    ax.set_ylim(-0.6, 1.6)
    ax.set_yticks([1, 0], [f"Shortest plan\n({totals['shortest'][1]} legs)", f"Feasible plan\n({totals['pdf'][1]} legs)"], color=INK)
    ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
    for y, method in ((1, "shortest"), (0, "pdf")):
        _stack(ax, totals[method][0], y, 0.36)
        ax.text(100.8, y, f"{100 * totals[method][0]['missing']:.1f} %", va="center", fontsize=8.5, color=INK, clip_on=False)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def compare_by_distance(bands, R, path):
    """Random poses by distance: what the rule's type does, and the extra length it costs. Two measures, so two panels."""
    size = (9, 3.7)
    fig, (left, right) = _start(size, "The rule's cost falls mainly on short legs",
                                f"Random poses, R = {R:.0f} m. Not the planner's cost: the heading search avoids many of these poses.",
                                left=0.1, right=0.97, bottom=0.11, top=0.63, ncols=2)
    fig.subplots_adjust(wspace=0.14)
    _kind_legend(fig, 1 - 0.62 / size[1])
    rows = list(range(len(bands)))[::-1]                      # the shortest legs at the top
    top = max(b["extra"] for b in bands) * 100
    for ax, title, limit in ((left, "Share of legs, by distance between the points", 100), (right, "Mean extra length of the feasible path", top * 1.2)):
        _quiet_axes(ax, "x")
        ax.set_title(title, loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
        ax.set_xlim(0, limit)
        ax.set_ylim(-0.6, len(bands) - 0.4)
        ax.xaxis.set_major_formatter(lambda v, _: f"{v:.0f} %")
        ax.axvline(0, color=BASELINE, lw=0.8, zorder=4)
    left.set_yticks(rows, [b["label"] for b in bands], color=INK)
    right.set_yticks([])
    for y, b in zip(rows, bands):
        _stack(left, b, y, 0.56)
        extra = 100 * b["extra"]
        _bar(right, 0, extra, y - 0.28, y + 0.28, FEASIBLE, "right")
        right.text(extra, y, f"  +{extra:.1f} %", va="center", fontsize=8.5, color=INK, zorder=4)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def save_gif(fig, update, n_frames, path, fps):
    """Save update(0), ..., update(n_frames - 1) as a looping GIF and close the figure. All frames share
    one palette, taken from eight frames spread over the animation, so that a colour used by only a few
    pixels in some frame (a green dot, say) keeps its colour there. Frames are drawn one at a time, and
    equal neighbours are merged by Pillow into one longer frame."""
    from PIL import Image

    def render(i):
        update(i)
        fig.canvas.draw()
        return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())

    sample = [np.asarray(render(i)) for i in np.unique(np.linspace(0, n_frames - 1, 8).astype(int))]
    palette = Image.fromarray(np.concatenate(sample, axis=0)).quantize(colors=255, method=Image.Quantize.MEDIANCUT)
    quantize = lambda im: im.quantize(palette=palette, dither=Image.Dither.NONE)
    first = quantize(render(0))
    first.save(path, save_all=True, append_images=(quantize(render(i)) for i in range(1, n_frames)),
               duration=round(1000 / fps), loop=0)
    plt.close(fig)


# ---------------------------------------------------------------------------
# How the genetic algorithm improves its tours (ga_evolution.py)
# ---------------------------------------------------------------------------

BEST, POP = COLORS[0], COLORS[1]              # blue: the best tour of a generation; orange: the whole population


def ga_summary(gens, R):
    """Numbers for titles and the run table, from one GA run's history (ga_tour.optimize)."""
    best = np.array([g["fitness"][0] for g in gens])
    improved = [i for i in range(1, len(best)) if best[i] < best[i - 1] - 1e-9]
    return {"best": best, "mean": np.array([g["fitness"].mean() for g in gens]),
            "worst": np.array([g["fitness"][-1] for g in gens]),
            "length": np.array([g["length"][0] for g in gens]), "peaks": np.array([g["peaks"][0] for g in gens]),
            "mean_peaks": np.array([g["peaks"].mean() for g in gens]),
            "distinct": np.array([len({tuple(t) for t in g["orders"]}) for g in gens]),
            "gain": 100 * (1 - best[-1] / best[0]), "improved": improved, "penalty": np.pi * R}


def _steps(values):
    """x and y for a steps-post line in which every generation, the last one included, has a step of width 1."""
    return np.arange(len(values) + 1), np.append(values, values[-1])


def _generation_axis(ax, n_gens):
    ax.set_xlim(0, n_gens)
    ax.set_xlabel("Generation", fontsize=9, color=MUTED)


def ga_convergence(gens, R, run_label, path):
    """The objective (fitness) of the best tour, the population mean and the population's range, per generation."""
    s = ga_summary(gens, R)
    n_imp = len(s["improved"])
    title = (f"The best tour improves by {s['gain']:.1f} % over {len(gens) - 1} generations" if s["gain"] > 0
             else "The first population already holds the best tour this run finds")
    size = (9, 4.6)
    fig, ax = _start(size, title,
                     f"{run_label}.\nLower is better. The GA keeps the better half of each generation, so the best never "
                     f"gets worse; it improved in {n_imp} of {len(gens) - 1}.",
                     left=0.1, right=0.97, bottom=0.13, top=0.74)
    _quiet_axes(ax, "y")
    x, worst = _steps(s["worst"])
    ax.fill_between(x, _steps(s["best"])[1], worst, step="post", color=POP, alpha=0.10, lw=0,
                    label="Population: best to worst")
    ax.plot(*_steps(s["mean"]), color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    ax.plot(*_steps(s["best"]), color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    ax.plot(s["improved"], s["best"][s["improved"]], ls="", marker="o", ms=5.5, color=BEST, mec=SURFACE, mew=1.5,
            zorder=5, label="Generation where the best improved")
    _generation_axis(ax, len(gens))
    ax.set_ylabel("Fitness = length + m(π)·π·R  [m]", fontsize=9, color=MUTED)
    ax.annotate(f"{s['best'][0]:.0f} m", (0, s["best"][0]), xytext=(2, 5), textcoords="offset points",
                ha="left", va="bottom", fontsize=8.5, color=INK)
    ax.annotate(f"{s['best'][-1]:.0f} m", (len(gens), s["best"][-1]), xytext=(0, -6), textcoords="offset points",
                ha="right", va="top", fontsize=8.5, color=INK)
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.13), ncol=4, frameon=False, fontsize=8.5, labelcolor=INK,
              handlelength=2.4, columnspacing=1.4)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def ga_fitness_split(gens, R, run_label, path):
    """The best tour's fitness split into straight-line length and peak penalty, and its number of peaks m(pi)."""
    s = ga_summary(gens, R)
    m0, m1 = int(s["peaks"][0]), int(s["peaks"][-1])
    saved = s["length"][0] - s["length"][-1]
    if m1 > m0:
        title = f"The GA trades radial unimodality for length: {m0} peak to {m1}, {saved:.0f} m shorter"
    elif saved > 0:
        title = f"The best tour gets {saved:.0f} m shorter and keeps {m1} peak" + ("s" if m1 != 1 else "")
    else:
        title = "The best tour does not change in this run"
    size = (9, 6.2)
    fig, (top, bottom) = _start(size, title,
                                f"{run_label}.\nEach peak (the tour turns back towards the base) costs π·R = "
                                f"{np.pi * R:.1f} m. Generation 0 has m(π) = 1 (Section 0.8.2).",
                                left=0.1, right=0.97, bottom=0.09, top=0.8, nrows=2, sharex=True,
                                gridspec_kw={"height_ratios": [2, 1.15], "hspace": 0.32})
    for ax in (top, bottom):
        _quiet_axes(ax, "y")
    top.set_title("Fitness of the best tour = straight-line length + peak penalty", loc="left", fontsize=9.5,
                  fontweight="bold", color=INK, pad=6)
    x, length = _steps(s["length"])
    top.fill_between(x, length, _steps(s["best"])[1], step="post", color=BEST, alpha=0.18, lw=0,
                     label="Peak penalty m(π)·π·R")
    top.plot(*_steps(s["best"]), color=BEST, lw=2, drawstyle="steps-post", label="Fitness")
    top.plot(x, length, color=BEST, lw=2, ls=DASHES, drawstyle="steps-post", label="Straight-line length")
    top.margins(y=0.08)
    top.set_ylabel("Metres", fontsize=9, color=MUTED)
    top.legend(loc="upper right", ncol=3, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=2.4)
    bottom.set_title("Number of peaks m(π)", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    bottom.plot(*_steps(s["mean_peaks"]), color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    bottom.plot(*_steps(s["peaks"]), color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    bottom.set_ylim(0, max(s["mean_peaks"].max(), s["peaks"].max()) * 1.15 + 0.5)
    bottom.yaxis.set_major_locator(MaxNLocator(integer=True))
    bottom.legend(loc="upper left", ncol=2, frameon=False, fontsize=8.5, labelcolor=INK, handlelength=2.4)
    _generation_axis(bottom, len(gens))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def ga_animation(out, run, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: every tour of the population (thin grey) and the best tour (blue) on the map, generation by
    generation, with the fitness and the number of peaks drawn alongside. Tours are drawn as the GA scores
    them: straight lines between the sensors, through obstacles if need be."""
    scen, p = out["scenario"], out["params"]
    gens, ids = run["generations"], np.asarray(run["ids"])
    s = ga_summary(gens, p["R"])
    xy, base = scen["xy"], scen["base"]
    n = len(gens)
    fig = plt.figure(figsize=(11, 6.6), dpi=dpi, facecolor=SURFACE)
    fig.text(0.015, 0.975, "How the genetic algorithm improves its tours", fontsize=12, fontweight="bold", color=INK, va="top")
    fig.text(0.015, 0.935, f"{run_label}. Tours are drawn as the GA scores them: straight lines between sensors.",
             fontsize=9, color=QUIET, va="top")
    ax = fig.add_axes([0.015, 0.1, 0.5, 0.74])
    _map(ax, out)
    ax.set_facecolor(SURFACE)
    ax.tick_params(length=0, labelsize=7, colors=MUTED)
    for side in ax.spines:
        ax.spines[side].set_visible(False)
    others = np.setdiff1d(np.arange(len(xy)), ids)
    if len(others):
        ax.scatter(*xy[others].T, s=8, color="#c9c8c1", zorder=4)
    ax.scatter(*xy[ids].T, s=14, color=INK, zorder=6)
    pts = np.vstack([xy, base[None, :]])
    lo, hi = pts.min(0) - 4, pts.max(0) + 4
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    pop_lines = [ax.plot([], [], color=QUIET, lw=0.7, alpha=0.08, zorder=2)[0] for _ in gens[0]["orders"][1:]]
    best_line, = ax.plot([], [], color=BEST, lw=1.8, zorder=7)
    ax.legend(handles=[Line2D([], [], color=BEST, lw=1.8, label="Best tour of the generation"),
                       Line2D([], [], color=QUIET, lw=0.9, alpha=0.5, label="The other tours of the population"),
                       Line2D([], [], ls="", marker="o", ms=4, color=INK, label="Sensors of this GA run"),
                       Line2D([], [], ls="", marker="^", ms=7, color=INK, label="Base station")],
              loc="upper left", bbox_to_anchor=(0.0, -0.005), ncol=2, frameon=False, fontsize=8, labelcolor=INK)

    top = fig.add_axes([0.6, 0.5, 0.37, 0.3])
    bottom = fig.add_axes([0.6, 0.12, 0.37, 0.25])
    for a in (top, bottom):
        _quiet_axes(a, "y")
        a.set_xlim(0, n)
    bottom.yaxis.set_major_locator(MaxNLocator(integer=True))
    top.set_title("Fitness [m]", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    span = s["mean"].max() - s["best"].min()
    top.set_ylim(s["best"].min() - 0.08 * span, s["mean"].max() + 0.08 * span)
    bottom.set_title("Number of peaks m(π)", loc="left", fontsize=9.5, fontweight="bold", color=INK, pad=6)
    bottom.set_ylim(0, max(s["mean_peaks"].max(), s["peaks"].max()) * 1.15 + 0.5)
    bottom.set_xlabel("Generation", fontsize=9, color=MUTED)
    mean_f, = top.plot([], [], color=POP, lw=2, ls=DASHES, drawstyle="steps-post", label="Population mean")
    best_f, = top.plot([], [], color=BEST, lw=2, drawstyle="steps-post", label="Best tour")
    dot_f, = top.plot([], [], ls="", marker="o", ms=6, color=BEST, mec=SURFACE, mew=1.5, zorder=5)
    mean_m, = bottom.plot([], [], color=POP, lw=2, ls=DASHES, drawstyle="steps-post")
    best_m, = bottom.plot([], [], color=BEST, lw=2, drawstyle="steps-post")
    top.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK, handlelength=2.4)
    label = fig.text(0.015, 0.89, "", fontsize=10, fontweight="bold", color=INK, va="top")

    def tour_xy(order):
        return np.vstack([base, xy[order], base]).T

    def update(frame):
        i = min(frame, n - 1)
        gen = gens[i]
        for line, order in zip(pop_lines, gen["orders"][1:]):
            line.set_data(*tour_xy(order))
        best_line.set_data(*tour_xy(gen["orders"][0]))
        mean_f.set_data(*_steps(s["mean"][:i + 1]))
        best_f.set_data(*_steps(s["best"][:i + 1]))
        dot_f.set_data([i], [s["best"][i]])
        mean_m.set_data(*_steps(s["mean_peaks"][:i + 1]))
        best_m.set_data(*_steps(s["peaks"][:i + 1]))
        label.set_text(f"Generation {i} of {n - 1}:  best fitness {s['best'][i]:.0f} m = {s['length'][i]:.0f} m"
                       f" + {int(s['peaks'][i])} × {np.pi * p['R']:.1f} m,  {s['distinct'][i]} different tours")
        return pop_lines + [best_line, mean_f, best_f, dot_f, mean_m, best_m, label]

    save_gif(fig, update, n + hold, path, fps)


def ga_population_animation(gens, R, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: every tour of the population as a dot, straight-line length across and number of peaks m(pi)
    up, one frame per generation. Grey diagonals are lines of equal fitness L + m(pi)*pi*R: one more
    peak is worth pi*R metres of length, so the cloud must move left (or down) to improve."""
    s = ga_summary(gens, R)
    n = len(gens)
    pen = np.pi * R
    L_all = np.concatenate([g["length"] for g in gens])
    m_all = np.concatenate([g["peaks"] for g in gens])
    x0, x1 = L_all.min(), L_all.max()
    x0, x1 = x0 - 0.04 * (x1 - x0), x1 + 0.04 * (x1 - x0)
    y1 = m_all.max() + 1
    fig = plt.figure(figsize=(9, 6), dpi=dpi, facecolor=SURFACE)
    fig.text(0.02, 0.975, "The population moves towards lower fitness", fontsize=12, fontweight="bold", color=INK, va="top")
    fig.text(0.02, 0.93, f"{run_label}. Each dot is one of the {len(gens[0]['orders'])} tours, spread a little up and "
             f"down within each m(π).\nGrey diagonals join tours of equal fitness L + m(π)·π·R: one more peak is "
             f"worth π·R = {pen:.1f} m of length.", fontsize=9, color=QUIET, va="top")
    ax = fig.add_axes([0.09, 0.1, 0.88, 0.66])
    _quiet_axes(ax, None)
    ax.set_xlim(x0, x1)
    ax.set_ylim(-0.5, y1)
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("Straight-line length of the tour [m]", fontsize=9, color=MUTED)
    ax.set_ylabel("Number of peaks m(π)", fontsize=9, color=MUTED)
    # lines of equal fitness F: m = (F - L) / (pi R)
    F0, F1 = x0, x1 + y1 * pen
    step = 10 ** np.floor(np.log10((F1 - F0) / 6))
    step *= 2 if (F1 - F0) / step > 12 else 1
    for F in np.arange(np.ceil(F0 / step) * step, F1, step):
        ax.plot([F + 0.5 * pen, F - y1 * pen], [-0.5, y1], color=GRID, lw=0.8, zorder=1)
    best_iso, = ax.plot([], [], color=BEST, lw=1.2, ls=DASHES, zorder=2, label="Equal fitness to the best tour")
    iso_label = ax.text(0, 0, "", fontsize=8, color=INK, ha="left", va="center")
    trail, = ax.plot([], [], color=BEST, lw=1.2, alpha=0.6, zorder=3, label="Path of the best tour so far")
    jitter = (np.arange(len(gens[0]["orders"])) % 7 - 3) * 0.07
    dots = ax.scatter([], [], s=22, color=POP, alpha=0.75, lw=0, zorder=4, label="A tour of the population")
    best = ax.scatter([], [], s=70, color=BEST, edgecolor=SURFACE, lw=1.8, zorder=5, label="Best tour")
    ax.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=INK)
    label = fig.text(0.09, 0.8, "", fontsize=10, fontweight="bold", color=INK, va="bottom")

    def update(frame):
        i = min(frame, n - 1)
        g = gens[i]
        dots.set_offsets(np.column_stack([g["length"][1:], g["peaks"][1:] + jitter[1:]]))
        best.set_offsets([[g["length"][0], g["peaks"][0]]])
        F = s["best"][i]
        best_iso.set_data([F + 0.5 * pen, F - y1 * pen], [-0.5, y1])
        iso_label.set_position((F - (y1 - 0.7) * pen + 0.012 * (x1 - x0), y1 - 0.7))
        iso_label.set_text(f"fitness {F:.0f} m")
        trail.set_data(s["length"][:i + 1], s["peaks"][:i + 1])
        label.set_text(f"Generation {i} of {n - 1}:  best fitness {F:.0f} m = {s['length'][i]:.0f} m + "
                       f"{int(s['peaks'][i])} × {pen:.1f} m,  {s['distinct'][i]} different tours")
        return [dots, best, best_iso, iso_label, trail, label]

    save_gif(fig, update, n + hold, path, fps)


def ga_radial_animation(gens, r, R, run_label, path, fps=8, hold=16, dpi=80):
    """GIF: distance from the base (geodesic, eq. 9) along the tour, sensor by sensor, one frame per
    generation. Generation 0 is radially unimodal (one hill, Section 0.8.2); every extra hill is a peak
    that costs pi*R in the fitness."""
    s = ga_summary(gens, R)
    n = len(gens)
    size = len(gens[0]["orders"][0])
    x = np.arange(size + 2)
    profile = lambda order: np.concatenate([[0.0], r[order], [0.0]])
    top = max(r[o].max() for g in gens for o in g["orders"]) * 1.12
    fig = plt.figure(figsize=(9, 5.4), dpi=dpi, facecolor=SURFACE)
    fig.text(0.02, 0.975, "Radial unimodality, generation by generation", fontsize=12, fontweight="bold", color=INK, va="top")
    fig.text(0.02, 0.925, f"{run_label}. Distance from the base of each sensor, in the order the tour visits them.\n"
             f"Generation 0 has one hill (Section 0.8.2); each extra hill is a peak that costs π·R = "
             f"{np.pi * R:.1f} m in the fitness.", fontsize=9, color=QUIET, va="top")
    ax = fig.add_axes([0.09, 0.12, 0.88, 0.62])
    _quiet_axes(ax, "y")
    ax.set_xlim(0, size + 1)
    ax.set_ylim(0, top)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("Position in the tour (0 and the last are the base)", fontsize=9, color=MUTED)
    ax.set_ylabel("Distance from the base r [m]", fontsize=9, color=MUTED)
    pop_lines = [ax.plot([], [], color=QUIET, lw=0.7, alpha=0.1, zorder=2)[0] for _ in gens[0]["orders"][1:]]
    best_line, = ax.plot([], [], color=BEST, lw=2, marker="o", ms=4, zorder=4, label="Best tour")
    peaks_mark = ax.scatter([], [], s=60, marker="v", color=INK, zorder=5, label="Peak (radial maximum)")
    ax.legend(handles=[best_line, peaks_mark, Line2D([], [], color=QUIET, lw=0.9, alpha=0.5, label="Other tours of the population")],
              loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=3, frameon=False, fontsize=8, labelcolor=INK)
    label = fig.text(0.09, 0.81, "", fontsize=10, fontweight="bold", color=INK, va="bottom")

    def update(frame):
        i = min(frame, n - 1)
        g = gens[i]
        for line, order in zip(pop_lines, g["orders"][1:]):
            line.set_data(x, profile(order))
        d = profile(g["orders"][0])
        best_line.set_data(x, d)
        at = np.flatnonzero((d[1:-1] > d[:-2]) & (d[1:-1] > d[2:])) + 1      # the radial maxima counted by ga_tour.peaks
        peaks_mark.set_offsets(np.column_stack([x[at], d[at] + 0.03 * top]) if len(at) else np.zeros((0, 2)))
        label.set_text(f"Generation {i} of {n - 1}:  the best tour has m(π) = {int(s['peaks'][i])} "
                       f"peak{'s' if s['peaks'][i] != 1 else ''},  fitness {s['best'][i]:.0f} m")
        return pop_lines + [best_line, peaks_mark, label]

    save_gif(fig, update, n + hold, path, fps)
