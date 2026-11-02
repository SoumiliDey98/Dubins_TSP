"""Figures: the planned routes (PNG) and an optional animation (GIF)."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as PolygonPatch
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
